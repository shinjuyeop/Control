/********************************************************************
 * Speed → VESS PWM + 랩타이머 (LCD 안정화 버전, 정지 감지 추가)
 *  - 개선점: ISR 데글리치, 지수이동평균(EMA), LCD 갱신 제한(10Hz)
 *  - 정지 감지(방법2): 최근 펄스 무(무Hall) 일정 시간 경과 시 0으로 강제 고정
 *******************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <math.h>

/* ── LCD & 사운드(PWM) ─────────────────── */
// 필요 시 0x27로 변경
#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);
Servo exhaust;

/* ── 핀 설정 ────────────────────────────── */
const byte HALL_PIN  = 2;     // Hall 센서 (INT0)
const byte SERVO_PIN = 9;     // ESS-DUAL+ PWM
const byte BTN_PIN   = 3;     // 랩 버튼

/* ── 속도 파라미터 ──────────────────────── */
const int   PPR           = 6;        // 모터 1회전 당 펄스수
const float TIRE_DIAM_CM  = 47.0;
const float TRANS_RATIO   = 4.60;
const unsigned long SPD_INTERVAL = 500;   // ms (표본창)

/* ── PWM 매핑 (아이들~풀) ───────────────── */
const int   SERVO_IDLE_US = 1600;     // 0 km/h → 1600 µs
const int   SERVO_MAX_US  = 2000;     // 풀 스로틀
const float KMH_MAX       = 100.0;    // 최고속(필요 시 변경)

/* ── 랩·스톱워치 변수 ───────────────────── */
bool          running      = false;
unsigned long startMs      = 0;
unsigned long lastLapMs    = 0;
unsigned long bestLapMs    = 0;
unsigned long lapCount     = 0;

/* ── 인터럽트 & 내부 변수 ───────────────── */
// 펄스 카운트 + 데글리치(최소 에지 간격)
volatile unsigned long pulseCnt   = 0;
volatile unsigned long lastEdgeUs = 0;
const unsigned long    MIN_PULSE_US = 300;  // 200~500 µs 사이에서 조정

unsigned long lastPulseCnt = 0;
unsigned long lastSpdCalc  = 0;

/* ── 속도 필터(EMA) & LCD 갱신 ──────────── */
float kmhFilt     = 0.0f;
const float ALPHA = 0.25f;   // 0.15~0.35 권장 (작을수록 안정, 클수록 즉각)
unsigned long lastLcdMs = 0;
float lastShown   = -1.0f;   // 처음엔 무조건 갱신되도록 -1로 둠
const float LCD_THRESH = 0.2f; // 0.2 km/h 이상 변화 시만 표시

/* ── 버튼·디바운스 & 길게누르기 ──────────── */
int  lastBtnState   = HIGH;
unsigned long pressStart  = 0;
bool          longHandled = false;
const unsigned long debounceMs = 30;    // (현재 논리에서 실제 사용 X)
const unsigned long LONG_MS    = 3000;  // 3초 길게 누르면 리셋

/* ── 백라이트 깜빡임 스케줄 ─────────────── */
bool          blinkActive   = false;
unsigned long blinkEndMs    = 0;
unsigned long nextBlinkMs   = 0;
bool          backOn        = true;
const unsigned long BLINK_DUR = 600;
const unsigned long BLINK_INT = 150;

/* ── 정지 감지 파라미터 ─────────────────── */
// 최근 lastEdgeUs 이후 이 시간(µs) 동안 펄스가 없으면 “정지”로 판정
const unsigned long STOP_US = 800000UL;   // ★ 추가: 0.8 s, 필요시 600~1200 ms로 조정

/* ── 유틸 ───────────────────────────────── */
void hallISR() {
  unsigned long t = micros();
  // 너무 촘촘한(노이즈성) 에지는 무시
  if (t - lastEdgeUs >= MIN_PULSE_US) {
    pulseCnt++;
    lastEdgeUs = t;
  }
}

void mmss(unsigned long ms, char *buf) {
  unsigned int s = ms / 1000;
  sprintf(buf, "%02u:%02u", s/60, s%60);
}

/* ── SETUP ─────────────────────────────── */
void setup() {
  Serial.begin(9600);

  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, RISING);

  pinMode(BTN_PIN, INPUT_PULLUP);
  exhaust.attach(SERVO_PIN, 1000, 2000);      // 안전 클램프

  // LCD init
  delay(200);          // 전원 안정화
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("SPEED: --.- km/h   ");
  lcd.setCursor(0,1); lcd.print("TIME : --:--       ");
  lcd.setCursor(0,2); lcd.print("BEST : --:--(--)   ");
  lcd.setCursor(0,3); lcd.print("LAP  : 0           ");

  lastSpdCalc = millis();
  lastLcdMs   = millis();
}

/* ── LOOP ──────────────────────────────── */
void loop() {
  unsigned long now = millis();

  /* -------- 버튼 처리 -------------------- */
  int reading = digitalRead(BTN_PIN);

  // Falling Edge (누르기 시작)
  if (reading == LOW && lastBtnState == HIGH) {
    pressStart  = now;
    longHandled = false;
  }

  // 길게 누름(3 s) → 리셋
  if (reading == LOW && !longHandled && (now - pressStart >= LONG_MS)) {
    running = false;
    startMs = lastLapMs = bestLapMs = 0;
    lapCount = 0;

    lcd.setCursor(7,1); lcd.print("--:--     ");
    lcd.setCursor(7,2); lcd.print("--:--(--) ");
    lcd.setCursor(7,3); lcd.print("0   ");

    longHandled = true;
  }

  // Rising Edge → 스타트/랩
  if (reading == HIGH && lastBtnState == LOW) {
    if ((now - pressStart) < LONG_MS && !longHandled) {
      /* 백라이트 깜빡임 */
      blinkActive = true;
      blinkEndMs  = now + BLINK_DUR;
      nextBlinkMs = now; backOn = false;
      lcd.noBacklight();

      if (!running) {                 /* 타이머 시작 */
        running   = true;
        startMs   = now;
        lapCount  = 1;
        bestLapMs = 0;
        lcd.setCursor(7,1); lcd.print("00:00     ");
        lcd.setCursor(7,3); lcd.print("1   ");
      } else {                        /* 랩 기록 */
        lastLapMs = now - startMs;
        startMs   = now;
        lapCount++;

        bool newBest = (bestLapMs==0 || lastLapMs < bestLapMs);
        if (newBest) bestLapMs = lastLapMs;

        char bestBuf[6]; mmss(bestLapMs, bestBuf);
        lcd.setCursor(7,2); lcd.print(bestBuf);

        char diffBuf[8] = "(LAST)";
        if (!newBest) {
          long diff = (long)lastLapMs - (long)bestLapMs;
          long sec  = labs(diff)/1000;
          sprintf(diffBuf,"(%c%02ld)", (diff>0?'+':'-'), sec);
        }
        lcd.print(diffBuf); lcd.print("  ");
        lcd.setCursor(7,3); lcd.print(lapCount); lcd.print("   ");
      }
    }
  }
  lastBtnState = reading;

  /* -------- 백라이트 토글 ----------------- */
  if (blinkActive) {
    if (now >= blinkEndMs) { blinkActive=false; backOn=true; lcd.backlight(); }
    else if (now >= nextBlinkMs) {
      nextBlinkMs += BLINK_INT;
      backOn = !backOn;
      backOn ? lcd.backlight() : lcd.noBacklight();
    }
  }

  /* -------- 속도 & PWM 계산 -------------- */
  if (now - lastSpdCalc >= SPD_INTERVAL) {
    // 표본창 동안의 펄스 증가량 측정
    noInterrupts();
    unsigned long delta = pulseCnt - lastPulseCnt;
    lastPulseCnt = pulseCnt;
    unsigned long lastEdgeCopy = lastEdgeUs;   // ★ 추가: 원자적 읽기용 복사
    interrupts();

    float dt_ms    = (now - lastSpdCalc);            // ms
    float motorRpm = (delta * 60.0f * 1000.0f) / (PPR * dt_ms);
    float wheelRpm = motorRpm / TRANS_RATIO;
    float kmh      = (wheelRpm * 3.1416f * TIRE_DIAM_CM * 60.0f) / 100000.0f;

    // 지수이동평균(EMA) 필터
    kmhFilt += ALPHA * (kmh - kmhFilt);

    // ★ 추가: 정지 감지(최근 Hall 에지 없음) → 0으로 강제 클램프
    if ((micros() - lastEdgeCopy) > STOP_US) {
      kmhFilt = 0.0f;
    }

    // PWM 매핑 (필터된 값 사용)
    float kmhClamp = constrain(kmhFilt, 0.0f, KMH_MAX);
    int thUs = SERVO_IDLE_US +
               (int)(kmhClamp * (SERVO_MAX_US - SERVO_IDLE_US) / KMH_MAX);
    exhaust.writeMicroseconds(thUs);

    lastSpdCalc = now;
  }

  /* -------- LCD 속도 표시 (10 Hz & 임계변화) ---- */
  if (now - lastLcdMs >= 100) {  // 10 Hz
    lastLcdMs = now;
    if (fabs(kmhFilt - lastShown) >= LCD_THRESH || lastShown < 0.0f) {
      lcd.setCursor(7,0);
      if (kmhFilt < 10.0f) lcd.print(' ');
      lcd.print(kmhFilt, 1); lcd.print(" km/h   ");
      lastShown = kmhFilt;
    }
  }

  /* -------- 스톱워치 갱신 ----------------- */
  if (running) {
    char buf[6]; mmss(now - startMs, buf);
    lcd.setCursor(7,1); lcd.print(buf); lcd.print("     ");
  }
}