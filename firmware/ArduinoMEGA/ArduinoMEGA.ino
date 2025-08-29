/********************************************************************
 * Integrated VCU Throttle / Mode / Gear Sender + Speed Meter
 *  ─ for Arduino Mega 2560   (VESS 안정화 + 정지 감지 + 빠른 피드백)
 *  변경점:
 *   - Hall ISR 데글리치(MIN_PULSE_US)
 *   - 속도 지수이동평균(EMA) kmhFilt
 *   - LCD 10 Hz 갱신 + 임계 변경폭(LCD_THRESH)
 *   - I2C Wire.begin()/setClock 추가
 *   - ★ 정지 감지(STOP_US) 추가 → 무펄스 일정시간 지나면 kmhFilt=0
 *   - ★ 피드백 가속: SPD_INTERVAL=300ms, ALPHA=0.35
 *******************************************************************/
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

/* ───────── I2C LCD ───────── */
#define LCD_ADDR 0x27   // 필요 시 0x3F
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
const float RPM_MIN = 0.0,  RPM_MAX = 500.0;

/* ── 주기 ── */
const uint32_t PERIOD_MS    = 10;    // 100 Hz 시리얼 패킷
const uint32_t SPD_INTERVAL = 300;   // ★ 0.3 s (기존 500ms → 더 빠른 응답)

/* ── 서보 각도 ── */
#define ANG_BRAKE   60
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

/* 브레이크 서보 속도 제한(슬루 레이트) */
int  brakeTarget = ANG_RELEASE;   // 목표 각도
int  brakeCurrent = ANG_RELEASE;  // 현재 명령 각도
unsigned long lastBrakeStepMs = 0;
const uint8_t  BRAKE_STEP_DEG = 1;        // 스텝당 이동 각도(°)
const uint16_t BRAKE_STEP_INTERVAL_MS = 20; // 스텝 간격(ms) → 1°/20ms = 50°/s

/* Hall 펄스 카운터 + 데글리치 */
volatile unsigned long pulseCnt   = 0;
volatile unsigned long lastEdgeUs = 0;
const unsigned long    MIN_PULSE_US = 300;  // 200~500 µs 사이에서 조정
unsigned long lastPulseCnt = 0;
unsigned long lastSpdCalc  = 0;

/* 속도 필터(EMA) & LCD 갱신 제어 */
float kmhFilt = 0.0f;
const float ALPHA = 0.35f;           // ★ 0.35로 상향(더 빠른 반응)
unsigned long lastLcdMs = 0;
float lastShown = -1.0f;             // 처음엔 무조건 갱신되도록
const float LCD_THRESH = 0.2f;       // 0.2 km/h 이상 변화 시 표시

/* ── 정지 감지 파라미터 ── */
const unsigned long STOP_US = 800000UL;   // ★ 0.8 s 무펄스면 정지로 판정

/* ───────── 인터럽트 ───────── */
void hallISR() {
  unsigned long t = micros();
  if (t - lastEdgeUs >= MIN_PULSE_US) {  // 노이즈성 짧은 에지 차단
    pulseCnt++;
    lastEdgeUs = t;
  }
}

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
  reverseFlag = (digitalRead(GEAR_PIN) == LOW);   // LOW = REV

  estopFlag = (e1 || e2 || rf);
}

/* ── MODE 전환 ── */
void enterEmergency() {
  if (currentMode != AS_EMERGENCY) Serial.println(F(">>> EMERGENCY <<<"));
  currentMode = AS_EMERGENCY;
  brakeTarget = ANG_BRAKE;  // 즉시 쓰지 않고 슬루 레이트로 이동
}
void enterAutomatic() { currentMode = AS_AUTO;   brakeTarget = ANG_RELEASE; }
void enterManual()    { currentMode = AS_MANUAL; brakeTarget = ANG_RELEASE; }

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
  brakeCurrent = ANG_RELEASE;
  brakeTarget  = ANG_RELEASE;
  brakeServo.write(brakeCurrent);

  exhaust.attach(VESS_PIN, 1000, 2000);        // VESS (pin 9)

  /* 스로틀 ADC 경계 */
  const float ADC_REF = 5.0;
  const int   ADC_RES = 1023;
  rawMin = int((V_MIN / ADC_REF) * ADC_RES + 0.5);
  rawMax = int((V_MAX / ADC_REF) * ADC_RES + 0.5);

  /* LCD (init/backlight 방식) */
  delay(200);           // 전원 안정화 권장
  Wire.begin();
  Wire.setClock(100000);   // 필요시 50000으로 낮출 수 있음
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("SPEED: --.- km/h   ");
  lcd.setCursor(0,1); lcd.print("MODE : MANUAL      ");
  lcd.setCursor(0,2); lcd.print("GEAR : FWD         ");
  lcd.setCursor(0,3); lcd.print("ESTOP: OFF         ");

  lastSpdCalc = millis();
  lastLcdMs   = millis();

  Serial.println(F("Throttle sender + VESS ready"));
}

/* ───────── LOOP ───────── */
void loop() {
  const unsigned long now = millis();

  /* 0) 브레이크 서보 슬루 레이트 적용 (논블로킹) */
  if (now - lastBrakeStepMs >= BRAKE_STEP_INTERVAL_MS) {
    lastBrakeStepMs = now;
    if (brakeCurrent != brakeTarget) {
      if (brakeCurrent < brakeTarget) {
        brakeCurrent = min(brakeCurrent + BRAKE_STEP_DEG, brakeTarget);
      } else {
        brakeCurrent = max(brakeCurrent - BRAKE_STEP_DEG, brakeTarget);
      }
      brakeServo.write(brakeCurrent);
    }
  }

  /* 1) 100 Hz 패킷 처리 (스로틀 → rpm16, 플래그) */
  static uint32_t tPkt = 0;
  if (now - tPkt >= PERIOD_MS) {
    tPkt = now;

    /* MODE 결정 */
    if (estopFlag)        enterEmergency();
    else if (isManual)    enterManual();
    else                  enterAutomatic();

    /* 스로틀 → rpm16 (0~2000) */
    int raw = readThrottle();
    int raw_hi = min(raw, rawMax);
    float voltage = raw_hi * (5.0f / 1023.0f);
    float rpm_f = (voltage - V_MIN) / (V_MAX - V_MIN) * (RPM_MAX - RPM_MIN) + RPM_MIN;
    if (rpm_f < 1.5f) rpm_f = 0.0f;
    rpm_f = constrain(rpm_f, RPM_MIN, RPM_MAX);
    uint16_t rpm16 = uint16_t(rpm_f + 1e-6f);

    uint8_t flags =
      (reverseFlag << 2) |
      (estopFlag   << 1) |
      (isManual    << 0);

    Serial.write(0xAA);
    Serial.write(uint8_t(rpm16 >> 8));
    Serial.write(uint8_t(rpm16 & 0xFF));
    Serial.write(flags);
  }

  /* 2) 속도 계산 & VESS PWM */
  if (now - lastSpdCalc >= SPD_INTERVAL) {
    noInterrupts();
    unsigned long delta = pulseCnt - lastPulseCnt;
    lastPulseCnt = pulseCnt;
    unsigned long lastEdgeCopy = lastEdgeUs;     // ★ 원자적 읽기용 복사
    interrupts();

    float dt_ms    = (now - lastSpdCalc);
    float motorRpm = (delta * 60.0f * 1000.0f) / (PPR * dt_ms);
    float wheelRpm = motorRpm / TRANS_RATIO;
    float kmh      = (wheelRpm * 3.1416f * TIRE_DIAM_CM * 60.0f) / 100000.0f;

    // EMA 갱신(빠른 반응을 위해 ALPHA=0.35)
    kmhFilt += ALPHA * (kmh - kmhFilt);

    // ★ 정지 감지: 최근 Hall 에지가 STOP_US 동안 없으면 0으로 클램프
    if ((micros() - lastEdgeCopy) > STOP_US) {
      kmhFilt = 0.0f;
    }

    float kmhClamp = constrain(kmhFilt, 0.0f, KMH_MAX);
    int   thUs = SERVO_IDLE_US +
                 int(kmhClamp * (SERVO_MAX_US - SERVO_IDLE_US) / KMH_MAX);
    exhaust.writeMicroseconds(thUs);

    lastSpdCalc = now;
  }

  /* 3) LCD 속도 표시 */
  if (now - lastLcdMs >= 100) {
    lastLcdMs = now;
    if (lastShown < 0.0f || fabs(kmhFilt - lastShown) >= LCD_THRESH) {
      lcd.setCursor(7,0);
      if (kmhFilt < 10.0f) lcd.print(' ');
      lcd.print(kmhFilt, 1); lcd.print(" km/h  ");
      lastShown = kmhFilt;
    }
  }

  /* 4) LCD 상태 업데이트 */
  static Mode lastMode   = AS_AUTO;
  static bool lastGearRv = false;
  static bool lastEstop  = false;

  if (currentMode != lastMode) {
    lcd.setCursor(7,1);
    switch (currentMode) {
      case AS_AUTO:      lcd.print("AUTO      "); break;
      case AS_MANUAL:    lcd.print("MANUAL    "); break;
      case AS_EMERGENCY: lcd.print("EMERGENCY "); break;
    }
    lastMode = currentMode;
  }

  if (reverseFlag != lastGearRv) {
    lcd.setCursor(7,2);
    lcd.print(reverseFlag ? "REV " : "FWD ");
    lcd.print("      ");
    lastGearRv = reverseFlag;
  }

  if (estopFlag != lastEstop) {
    lcd.setCursor(7,3);
    lcd.print(estopFlag ? "ON " : "OFF");
    lcd.print("       ");
    lastEstop = estopFlag;
  }
}
