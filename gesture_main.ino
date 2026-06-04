// ============================================================
// gesture_main.ino
// Pico 2W: RAW 9216 UART 수신 + Edge Impulse + 센서/서보 액션 최종본
//
// 핵심 수정:
//   - get_signal_data() = RGB888 packed grayscale
//   - 기존처럼 판/틸트 좌표 수신, RAW 수신, 추론 후 센서 액션 수행
//
// ESP32-CAM -> Pico:
//   AA 55 + size_lo + size_hi + RAW 9216 + 55 AA
// ============================================================

#include <gesture_recognition_inferencing.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <ScioSense_ENS16x.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

#define PAN_PIN       0
#define TILT_PIN      11
#define LED_PIN       6
#define LED_COUNT     5

#define UART_RX_PIN   9
#define UART_TX_PIN   8
#define UART_BAUD     115200

#define CAM_UART Serial2

#define CAM_W 320
#define CAM_H 240

#define PAN_CENTER     90
#define PAN_RANGE      40
#define TILT_CENTER    90
#define TILT_RANGE     25

#define SMOOTH_PAN     0.15f
#define SMOOTH_TILT    0.08f

#define DEADZONE_X     25
#define DEADZONE_Y     50

#define ANGLE_TH_PAN   0.8f
#define ANGLE_TH_TILT  1.5f

#define CONFIDENCE_THRESHOLD  0.5f
#define RESULT_DISPLAY_MS     5000

#define RAW_MARKER_1   0xAA
#define RAW_MARKER_2   0x55

#define RAW_W 96
#define RAW_H 96
#define RAW_SIZE 9216

enum State {
  STATE_TRACKING,
  STATE_RECEIVING_RAW,
  STATE_SHOWING_RESULT
};

enum RawRecvState {
  RR_WAIT_SIZE_LO,
  RR_WAIT_SIZE_HI,
  RR_WAIT_DATA,
  RR_WAIT_END_1,
  RR_WAIT_END_2
};

State currentState = STATE_TRACKING;
RawRecvState rawState = RR_WAIT_SIZE_LO;

unsigned long stateEnteredAt = 0;

Servo panServo;
Servo tiltServo;

Adafruit_BME280 bme;
BH1750 lightMeter;
ENS160 ens160;
Adafruit_SSD1306 oled(128, 64, &Wire);
Adafruit_NeoPixel leds(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

float current_pan = PAN_CENTER;
float current_tilt = TILT_CENTER;
float last_set_pan = PAN_CENTER;
float last_set_tilt = TILT_CENTER;

String uart_line_buffer = "";

uint8_t raw_buffer[RAW_SIZE];
uint8_t decoded_gray[RAW_SIZE];

size_t raw_expected_size = 0;
size_t raw_received = 0;

int last_predicted_class = -1;
float last_confidence = 0.0f;

bool bme_available = false;
bool bh_available = false;
bool ens_available = false;
bool oled_available = false;

uint32_t COLOR_FIST      = 0x0000FF;
uint32_t COLOR_OFF       = 0x000000;
uint32_t COLOR_INDEX     = 0x00FF00;
uint32_t COLOR_VICTORY   = 0xFFFF00;
uint32_t COLOR_OK        = 0xFFFFFF;
uint32_t COLOR_IDLE      = 0x002200;
uint32_t COLOR_UNCERTAIN = 0xFF0000;
uint32_t COLOR_RECEIVING = 0xFFAA00;

const char* class_names[4] = {
  "fist",
  "index",
  "ok",
  "victory"
};

void setAllLeds(uint32_t color) {
  for (int i = 0; i < LED_COUNT; i++) {
    leds.setPixelColor(i, color);
  }
  leds.show();
}

// ============================================================
// Edge Impulse 이미지 입력
// 매우 중요:
// 0~1 정규화가 아니라 grayscale 값을 RGB888 packed로 전달.
// v -> 0xRRGGBB = v,v,v
// ============================================================
int get_signal_data(size_t offset, size_t length, float* out_ptr) {
  for (size_t i = 0; i < length; i++) {
    uint8_t v = decoded_gray[offset + i];

    uint32_t rgb =
      ((uint32_t)v << 16) |
      ((uint32_t)v << 8)  |
      ((uint32_t)v);

    out_ptr[i] = (float)rgb;
  }

  return 0;
}

void setServoAngle(Servo& s, float angle) {
  if (angle < 10) angle = 10;
  if (angle > 170) angle = 170;
  s.write((int)angle);
}

void showUncertain();


void drawTitle(const char* title, int x) {
  oled.setTextSize(2);
  oled.setCursor(x, 0);
  oled.println(title);
  oled.drawLine(0, 19, 127, 19, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 25);
}

void action_fist() {
  if (oled_available) {
    oled.clearDisplay();
    drawTitle("FIST", 38);

    if (ens_available) {
      if (ens160.update() == RESULT_OK && ens160.hasNewData()) {
        oled.print("AQI : ");
        oled.println((int)ens160.getAirQualityIndex_UBA());

        oled.print("CO2 : ");
        oled.print(ens160.getEco2());
        oled.println(" ppm");

        oled.print("TVOC: ");
        oled.print(ens160.getTvoc());
        oled.println(" ppb");
      } else {
        oled.println("ENS160 warming...");
        oled.println("Please wait");
      }
    } else {
      oled.println("ENS160 not ready");
    }

    oled.display();
  }

  // FIST만 파란색 LED 켜기
  setAllLeds(COLOR_FIST);
}

void action_index() {
  if (oled_available) {
    oled.clearDisplay();
    drawTitle("INDEX", 30);

    if (bme_available) {
      bme.takeForcedMeasurement();

      float temp = bme.readTemperature();
      float hum  = bme.readHumidity();

      oled.print("Temp: ");
      oled.print(temp, 1);
      oled.println(" C");

      oled.print("Hum : ");
      oled.print(hum, 1);
      oled.println(" %");
    } else {
      oled.println("BME280 not ready");
    }

    oled.display();
  }

  // FIST 이외에는 LED OFF
  setAllLeds(COLOR_OFF);
}

void action_victory() {
  if (oled_available) {
    oled.clearDisplay();
    drawTitle("VICTORY", 18);

    if (bh_available) {
      float lux = lightMeter.readLightLevel();

      oled.print("Light: ");
      oled.print(lux, 0);
      oled.println(" lx");
    } else {
      oled.println("BH1750 not ready");
    }

    oled.display();
  }

  // FIST 이외에는 LED OFF
  setAllLeds(COLOR_OFF);
}

void action_ok() {
  if (oled_available) {
    oled.clearDisplay();
    drawTitle("OK", 52);

    oled.println("System confirmed");
    oled.println("Gesture accepted");
    oled.print("Conf: ");
    oled.print(last_confidence, 2);

    oled.display();
  }

  // FIST 이외에는 LED OFF
  setAllLeds(COLOR_OFF);
}

typedef void (*ActionFn)();

ActionFn action_table[4] = {
  action_fist,
  action_index,
  action_ok,
  action_victory
};

void showUncertain() {
  if (oled_available) {
    oled.clearDisplay();

    oled.setTextSize(2);
    oled.setCursor(8, 16);
    oled.println("TRY");
    oled.setCursor(8, 36);
    oled.println("AGAIN");

    oled.setTextSize(1);
    oled.setCursor(80, 54);
    oled.print(last_confidence, 2);

    oled.display();
  }

  setAllLeds(COLOR_OFF);
}

void printLabelCheck() {
  Serial.println("=== LABEL CHECK ===");

  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(ei_classifier_inferencing_categories[i]);
  }

  Serial.print("EI_CLASSIFIER_INPUT_WIDTH = ");
  Serial.println(EI_CLASSIFIER_INPUT_WIDTH);

  Serial.print("EI_CLASSIFIER_INPUT_HEIGHT = ");
  Serial.println(EI_CLASSIFIER_INPUT_HEIGHT);

  Serial.print("EI_CLASSIFIER_LABEL_COUNT = ");
  Serial.println(EI_CLASSIFIER_LABEL_COUNT);

  Serial.print("EI_CLASSIFIER_SENSOR = ");
  Serial.println(EI_CLASSIFIER_SENSOR);
}

void runInference() {
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &get_signal_data;

  ei_impulse_result_t result;
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

  if (err != EI_IMPULSE_OK) {
    Serial.printf("[INFER] failed: %d\n", err);
    showUncertain();
    return;
  }

  float best_score = 0.0f;
  int best_class = -1;

  Serial.print("[INFER] ");

  for (int i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    Serial.printf("%s: %.4f  ",
                  result.classification[i].label,
                  result.classification[i].value);

    if (result.classification[i].value > best_score) {
      best_score = result.classification[i].value;
      best_class = i;
    }
  }

  Serial.println();

  last_predicted_class = best_class;
  last_confidence = best_score;

  if (best_score < CONFIDENCE_THRESHOLD || best_class < 0) {
    Serial.printf("[INFER] low confidence: %.4f\n", best_score);
    showUncertain();
    return;
  }

  Serial.printf("[RESULT] predicted=%s score=%.4f\n",
                result.classification[best_class].label,
                best_score);

  action_table[best_class]();
}


// ============================================================
// Pico가 실제 추론에 사용한 decoded_gray 이미지를 USB Serial로 PC에 전송
// PC는 이 데이터를 pico_received.png로 저장합니다.
//
// Binary protocol:
//   PRAW + size_lo + size_hi + 9216 bytes + DONE
// ============================================================
void sendPicoReceivedImageToPC() {
  const uint8_t header[4] = { 'P', 'R', 'A', 'W' };
  const uint8_t footer[4] = { 'D', 'O', 'N', 'E' };

  Serial.write(header, 4);
  Serial.write((uint8_t)(RAW_SIZE & 0xFF));
  Serial.write((uint8_t)((RAW_SIZE >> 8) & 0xFF));
  Serial.write(decoded_gray, RAW_SIZE);
  Serial.write(footer, 4);
  Serial.flush();

  Serial.println();
  Serial.println("[USB] pico_received image sent to PC");
}

void decodeAndInfer() {
  Serial.printf("[RAW] received=%d expected=%d\n",
                raw_received,
                raw_expected_size);

  if (raw_expected_size != RAW_SIZE || raw_received != RAW_SIZE) {
    Serial.println("[RAW] invalid size");
    showUncertain();
    return;
  }

  memcpy(decoded_gray, raw_buffer, RAW_SIZE);

  // PC에서 실제 Pico 추론 입력 이미지를 확인할 수 있도록 USB로 전송
  sendPicoReceivedImageToPC();

  uint8_t minv = 255;
  uint8_t maxv = 0;
  uint32_t checksum = 0;

  for (int i = 0; i < RAW_SIZE; i++) {
    uint8_t v = decoded_gray[i];
    checksum += v;
    if (v < minv) minv = v;
    if (v > maxv) maxv = v;
  }

  Serial.printf("[RAW] checksum=%lu min=%d max=%d avg=%lu\n",
                checksum, minv, maxv, checksum / RAW_SIZE);

  Serial.println("[INPUT] mode=RGB888 packed grayscale");

  runInference();
}

void handleCoordLine(String line) {
  line.trim();

  if (!line.startsWith("P,")) return;

  int c1 = line.indexOf(',');
  int c2 = line.indexOf(',', c1 + 1);

  if (c1 < 0 || c2 < 0) return;

  int x = line.substring(c1 + 1, c2).toInt();
  int y = line.substring(c2 + 1).toInt();

  int err_x = x - CAM_W / 2;
  int err_y = y - CAM_H / 2;

  if (abs(err_x) < DEADZONE_X) err_x = 0;
  if (abs(err_y) < DEADZONE_Y) err_y = 0;

  float target_pan =
    PAN_CENTER - (err_x * (float)PAN_RANGE / (CAM_W / 2));

  float target_tilt =
    TILT_CENTER + (err_y * (float)TILT_RANGE / (CAM_H / 2));

  current_pan =
    current_pan * (1 - SMOOTH_PAN) + target_pan * SMOOTH_PAN;

  current_tilt =
    current_tilt * (1 - SMOOTH_TILT) + target_tilt * SMOOTH_TILT;

  if (fabs(current_pan - last_set_pan) > ANGLE_TH_PAN) {
    setServoAngle(panServo, current_pan);
    last_set_pan = current_pan;
  }

  if (fabs(current_tilt - last_set_tilt) > ANGLE_TH_TILT) {
    setServoAngle(tiltServo, current_tilt);
    last_set_tilt = current_tilt;
  }
}

void resetRawReceiver() {
  currentState = STATE_TRACKING;
  rawState = RR_WAIT_SIZE_LO;
  raw_expected_size = 0;
  raw_received = 0;
  setAllLeds(COLOR_OFF);
}

void handleTracking() {
  int processed = 0;
  static int prev_byte = -1;

  while (CAM_UART.available() && processed < 512) {
    processed++;

    int b = CAM_UART.read();

    if (prev_byte == RAW_MARKER_1 && b == RAW_MARKER_2) {
      Serial.println("[MARKER] AA 55 detected");
      Serial.println("[STATE] TRACKING -> RECEIVING_RAW");

      currentState = STATE_RECEIVING_RAW;
      rawState = RR_WAIT_SIZE_LO;
      stateEnteredAt = millis();

      raw_expected_size = 0;
      raw_received = 0;
      uart_line_buffer = "";
      prev_byte = -1;

      setAllLeds(COLOR_RECEIVING);
      return;
    }

    prev_byte = b;

    if (b == '\n') {
      handleCoordLine(uart_line_buffer);
      uart_line_buffer = "";
      continue;
    }

    if (b == '\r') continue;

    if (uart_line_buffer.length() == 0) {
      if (b == 'P' || b == 'N') {
        uart_line_buffer += (char)b;
      }
    } else {
      uart_line_buffer += (char)b;

      if (uart_line_buffer.length() > 64) {
        uart_line_buffer = "";
      }
    }
  }
}

void handleReceivingRaw() {
  if (millis() - stateEnteredAt > 5000) {
    Serial.printf("[RAW] timeout: received=%d expected=%d\n",
                  raw_received,
                  raw_expected_size);
    resetRawReceiver();
    return;
  }

  int processed = 0;

  while (CAM_UART.available() && processed < 2048) {
    processed++;

    uint8_t b = CAM_UART.read();

    switch (rawState) {
      case RR_WAIT_SIZE_LO:
        raw_expected_size = b;
        rawState = RR_WAIT_SIZE_HI;
        break;

      case RR_WAIT_SIZE_HI:
        raw_expected_size |= ((uint16_t)b << 8);

        Serial.printf("[RAW] expected size=%d\n", raw_expected_size);

        if (raw_expected_size != RAW_SIZE) {
          Serial.printf("[RAW] size error: %d\n", raw_expected_size);
          resetRawReceiver();
          return;
        }

        raw_received = 0;
        rawState = RR_WAIT_DATA;
        break;

      case RR_WAIT_DATA:
        raw_buffer[raw_received++] = b;

        if (raw_received >= RAW_SIZE) {
          rawState = RR_WAIT_END_1;
        }
        break;

      case RR_WAIT_END_1:
        if (b == RAW_MARKER_2) {
          rawState = RR_WAIT_END_2;
        } else {
          Serial.printf("[RAW] end marker 1 error: 0x%02X\n", b);
          resetRawReceiver();
          return;
        }
        break;

      case RR_WAIT_END_2:
        if (b == RAW_MARKER_1) {
          Serial.println("[RAW] end marker OK");
          decodeAndInfer();

          currentState = STATE_SHOWING_RESULT;
          stateEnteredAt = millis();
          return;
        } else {
          Serial.printf("[RAW] end marker 2 error: 0x%02X\n", b);
          resetRawReceiver();
          return;
        }
        break;
    }
  }
}

void handleShowingResult() {
  int processed = 0;

  while (CAM_UART.available() && processed < 512) {
    CAM_UART.read();
    processed++;
  }

  if (millis() - stateEnteredAt >= RESULT_DISPLAY_MS) {
    Serial.println("[STATE] SHOWING_RESULT -> TRACKING (display kept)");

    currentState = STATE_TRACKING;
    rawState = RR_WAIT_SIZE_LO;
    raw_expected_size = 0;
    raw_received = 0;

    // 중요:
    // OLED와 LED는 다음 추론 결과가 나올 때까지 유지합니다.
    // 여기서 OLED clearDisplay() 또는 LED OFF를 하지 않습니다.
  }
}

void setup() {
  delay(2000);

  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=== Pico 2W RAW UART Receiver Final ===");

  CAM_UART.setTX(UART_TX_PIN);
  CAM_UART.setRX(UART_RX_PIN);
  CAM_UART.begin(UART_BAUD);

  Serial.printf("[UART] Serial2 @ %d bps, TX=%d RX=%d\n",
                UART_BAUD, UART_TX_PIN, UART_RX_PIN);

  printLabelCheck();

  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  Wire.setClock(400000);

  oled_available = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  if (oled_available) {
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.println("Edge AI 3");
    oled.println("Waiting...");
    oled.display();
  }

  bme_available = bme.begin(0x76);

  if (!bme_available) {
    bme_available = bme.begin(0x77);
  }

  if (bme_available) {
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::FILTER_OFF);
  }

  bh_available = lightMeter.begin();

  ens160.begin(&Wire, 0x53);

  unsigned long ens_start = millis();
  ens_available = false;

  while (millis() - ens_start < 3000) {
    if (ens160.init() == true) {
      ens_available = true;
      break;
    }

    delay(100);
  }

  if (ens_available) {
    ens160.startStandardMeasure();
  }

  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);

  setServoAngle(panServo, PAN_CENTER);
  setServoAngle(tiltServo, TILT_CENTER);

  leds.begin();
  leds.setBrightness(50);
  setAllLeds(COLOR_OFF);

  currentState = STATE_TRACKING;
  rawState = RR_WAIT_SIZE_LO;
  stateEnteredAt = millis();

  Serial.println("=== Setup done. TRACKING ===");
}

void loop() {
  switch (currentState) {
    case STATE_TRACKING:
      handleTracking();
      break;

    case STATE_RECEIVING_RAW:
      handleReceivingRaw();
      break;

    case STATE_SHOWING_RESULT:
      handleShowingResult();
      break;
  }

  delay(1);
}
