/********************************************************************
 * Integrated VCU Throttle / Mode / Gear Sender + Speed Meter
 *  ─ for Arduino Mega 2560   (VESS 추가 버전)
 *******************************************************************/
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/* ───────── I2C LCD ───────── */
// 필요 시 0x27로 변경
#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);

/* ───────── PIN MAP ───────── */
#define THROTTLE_PIN  A7      // 0–5 V
#define ESTOP1_PIN    52
#define ESTOP2_PIN    53
#define ASMS_PIN      A8
#define GEAR_PIN      A9
#define SERVO_PIN     46      // 브레이크 서보
#define VESS_PIN      9       // ESS-DUAL+ (가상 엔진음 PWM)
#define RF_ESTOP_PIN  A12
#define HALL_PIN      2       // 외부 인터럽트0

/* ── 스로틀 → RPM 스케일 ── */
const float V_MIN   = 0.8,  V_MAX   = 4.4;   // [V]
const float RPM_MIN = 0.0,  RPM_MAX = 2000.0;

/* ── 주기 ── */
const uint32_t PERIOD_MS    = 10;   // 100 Hz
const uint32_t SPD_INTERVAL = 500;  // 0.5 s

/* ── 서보 각도 ── */
#define ANG_BRAKE   45
#define ANG_RELEASE 80

/* ── 속도 계산 ── */
const int   PPR          = 6;
const float TIRE_DIAM_CM = 47.0;
const float TRANS_RATIO  = 4.60;

/* ── VESS 매핑 ── */
const int   SERVO_IDLE_US = 1600;   // 0 km/h → 시동음 최소화
const int   SERVO_MAX_US  = 2000;   // 최고속
const float KMH_MAX       = 100.0;  // 이 이상은 MAX 고정

/* ── MODE enum ── */
enum Mode { AS_AUTO, AS_MANUAL, AS_EMERGENCY };
volatile Mode currentMode = AS_AUTO;

/* ── 플래그 ── */
volatile bool estopFlag   = false;
volatile bool isManual    = true;
volatile bool reverseFlag = false;

/* ── 전역 ── */
Servo brakeServo;
Servo exhaust;          // VESS 서보
int rawMin, rawMax;

static float rpmPrev = 0;
static float qError = 0;    // EMA, Error dithering (미사용)

/* Hall 펄스 카운터 */
volatile unsigned long pulseCnt = 0;
unsigned long lastPulseCnt = 0;
unsigned long lastSpdCalc  = 0;

/* ───────── 인터럽트 ───────── */
void hallISR() { pulseCnt++; }

void setupInterrupts() {
  pinMode(ESTOP1_PIN, INPUT_PULLUP);
  pinMode(ESTOP2_PIN, INPUT_PULLUP);
  pinMode(ASMS_PIN,   INPUT_PULLUP);
  pinMode(GEAR_PIN,   INPUT_PULLUP);
  pinMode(RF_ESTOP_PIN, INPUT_PULLUP);

  /* --- PCINT 설정 --- */
  // PORTB (D52=PB1, D53=PB0)
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT1);
  PCIFR  |= (1 << PCIF0);
  PCICR  |= (1 << PCIE0);

  // PORTK (A8=PK0, A9=PK1, A12=PK4)
  PCMSK2 |= (1 << PCINT16) | (1 << PCINT17) | (1 << PCINT20);
  PCIFR  |= (1 << PCIF2);
  PCICR  |= (1 << PCIE2);

  /* Hall 센서 인터럽트 */
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, RISING);

  sei();
}

/* --- PORTB 인터럽트 --- */
ISR(PCINT0_vect) {
  bool e1 = (digitalRead(ESTOP1_PIN) == LOW);
  bool e2 = (digitalRead(ESTOP2_PIN) == LOW);
  bool rf = (digitalRead(RF_ESTOP_PIN) == LOW);
  estopFlag = (e1 || e2 || rf);
}

/* --- PORTK 인터럽트 --- */
ISR(PCINT2_vect) {
  bool rf = (digitalRead(RF_ESTOP_PIN) == LOW);
  bool e1 = (digitalRead(ESTOP1_PIN) == LOW);
  bool e2 = (digitalRead(ESTOP2_PIN) == LOW);

  isManual    = (digitalRead(ASMS_PIN) == HIGH);  // HIGH = MANUAL
  reverseFlag = (digitalRead(GEAR_PIN) == LOW);  // LOW = REV

  estopFlag = (e1 || e2 || rf);
}

/* ── MODE 전환 ── */
void enterEmergency() {
  if (currentMode != AS_EMERGENCY) Serial.println(F(">>> EMERGENCY <<<"));
  currentMode = AS_EMERGENCY;
  brakeServo.write(ANG_BRAKE);
}
void enterAutomatic() { currentMode = AS_AUTO;   brakeServo.write(ANG_RELEASE); }
void enterManual()    { currentMode = AS_MANUAL; brakeServo.write(ANG_RELEASE); }

/* ── 공용 ADC 읽기 ── */
int readThrottle(int n = 8) {
  long sum = 0;
  for (int i = 0; i < n; ++i) {
    sum += analogRead(THROTTLE_PIN);
    delayMicroseconds(400);
  }
  return sum / n;
}

/* ───────── SETUP ───────── */
void setup() {
  Serial.begin(115200);
  setupInterrupts();

  brakeServo.attach(SERVO_PIN);                // 브레이크
  brakeServo.write(ANG_RELEASE);

  exhaust.attach(VESS_PIN, 1000, 2000);        // VESS (pin 9)

  /* 스로틀 ADC 경계 */
  const float ADC_REF = 5.0;
  const int   ADC_RES = 1023;
  rawMin = int((V_MIN / ADC_REF) * ADC_RES + 0.5);
  rawMax = int((V_MAX / ADC_REF) * ADC_RES + 0.5);

  /* LCD (init/backlight 방식) */
  delay(200);           // 전원 안정화 권장
  lcd.init();           // ★ 변경 포인트
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("SPEED: --.- km/h   ");
  lcd.setCursor(0,1); lcd.print("MODE : MANUAL      ");
  lcd.setCursor(0,2); lcd.print("GEAR : FWD         ");
  lcd.setCursor(0,3); lcd.print("ESTOP: OFF         ");

  Serial.println(F("Throttle sender + VESS ready"));
}

/* ───────── LOOP ───────── */
void loop() {
  const unsigned long now = millis();

  /* 1) 100 Hz 패킷 처리 */
  static uint32_t tPkt = 0;
  if (now - tPkt >= PERIOD_MS) {
    tPkt = now;

    /* MODE 결정 */
    if (estopFlag)        enterEmergency();
    else if (isManual)    enterManual();
    else                  enterAutomatic();

    /* 스로틀 → rpm16 (0~2000)
      - 하한(rawMin) 클램프 제거: 휴지 전압이 V_MIN보다 낮을 때도 음수→0으로 처리
      - 데드밴드: 아주 작은 값(노이즈/양자화)을 0으로 소거
      - 0 근처 반올림 억제: +0.5f 제거(0.6→1 같은 현상 방지) */
    int raw = readThrottle();
    int raw_hi = min(raw, rawMax);                 // 상한만 제한, 하한 미제한
    float voltage = raw_hi * (5.0f / 1023.0f);
    float rpm_f = (voltage - V_MIN) / (V_MAX - V_MIN) * (RPM_MAX - RPM_MIN) + RPM_MIN;
    if (rpm_f < 1.5f) rpm_f = 0.0f;                // 데드밴드(필요시 1.0~2.0 조정)
    rpm_f = constrain(rpm_f, RPM_MIN, RPM_MAX);

    uint16_t rpm16 = uint16_t(rpm_f + 1e-6f);      // 0 인근에서 올림 방지

    uint8_t flags =
      (reverseFlag << 2) |
      (estopFlag   << 1) |
      (isManual    << 0);

    Serial.write(0xAA);
    Serial.write(uint8_t(rpm16 >> 8));
    Serial.write(uint8_t(rpm16 & 0xFF));
    Serial.write(flags);
  }

  /* 2) 속도 계산 + VESS PWM */
  if (now - lastSpdCalc >= SPD_INTERVAL) {
    noInterrupts();
    unsigned long delta = pulseCnt - lastPulseCnt;
    lastPulseCnt = pulseCnt;
    interrupts();

    float motorRpm = (delta * 60.0f * 1000.0f) / (PPR * (now - lastSpdCalc));
    float wheelRpm = motorRpm / TRANS_RATIO;
    float kmh      = (wheelRpm * 3.1416f * TIRE_DIAM_CM * 60.0f) / 100000.0f;

    lastSpdCalc = now;

    /* --- LCD 속도 표시 --- */
    lcd.setCursor(7,0);
    if (kmh < 10) lcd.print(' ');
    lcd.print(kmh, 1); lcd.print(" km/h  ");

    /* --- VESS PWM 매핑 --- */
    float kmhClamp = constrain(kmh, 0.0f, KMH_MAX);
    int   thUs = SERVO_IDLE_US +
                 int(kmhClamp * (SERVO_MAX_US - SERVO_IDLE_US) / KMH_MAX);
    exhaust.writeMicroseconds(thUs);
  }

  /* 3) LCD 상태 업데이트 */
  static Mode lastMode   = AS_AUTO;
  static bool lastGearRv = false;
  static bool lastEstop  = false;

  if (currentMode != lastMode) {
    lcd.setCursor(7,1);         // "MODE : "
    switch (currentMode) {
      case AS_AUTO:      lcd.print("AUTO      "); break;
      case AS_MANUAL:    lcd.print("MANUAL    "); break;
      case AS_EMERGENCY: lcd.print("EMERGENCY "); break;
    }
    lastMode = currentMode;
  }

  if (reverseFlag != lastGearRv) {
    lcd.setCursor(7,2);         // "GEAR : "
    lcd.print(reverseFlag ? "REV " : "FWD ");
    lcd.print("      ");
    lastGearRv = reverseFlag;
  }

  if (estopFlag != lastEstop) {
    lcd.setCursor(7,3);         // "ESTOP: "
    lcd.print(estopFlag ? "ON " : "OFF");
    lcd.print("       ");
    lastEstop = estopFlag;
  }
}
