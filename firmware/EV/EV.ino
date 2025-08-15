#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

/* ── LCD & 사운드(PWM) ─────────────────── */
// 필요 시 0x27로 변경
#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);
Servo exhaust;

/* ── 핀 설정 ────────────────────────────── */
const byte HALL_PIN  = 2;     // Hall 센서
const byte SERVO_PIN = 9;     // ESS-DUAL+ PWM
const byte BTN_PIN   = 3;     // 랩 버튼

/* ── 속도 파라미터 ──────────────────────── */
const int   PPR           = 6;
const float TIRE_DIAM_CM  = 47.0;
const float TRANS_RATIO   = 4.60;
const unsigned long SPD_INTERVAL = 500;   // ms

/* ── PWM 매핑 (아이들~풀) ───────────────── */
const int   SERVO_IDLE_US = 1600;         // 아이들 (0 km/h → 1600 µs)
const int   SERVO_MAX_US  = 2000;         // 풀 스로틀
const float KMH_MAX       = 100.0;        // 최고속(필요하면 변경)

/* ── 랩·스톱워치 변수 ───────────────────── */
bool          running      = false;
unsigned long startMs      = 0;
unsigned long lastLapMs    = 0;
unsigned long bestLapMs    = 0;
unsigned long lapCount     = 0;

/* ── 인터럽트 & 내부 변수 ───────────────── */
volatile unsigned long pulseCnt = 0;
unsigned long lastPulseCnt = 0, lastSpdCalc = 0;

/* ── 버튼·디바운스 & 길게누르기 ──────────── */
int  lastBtnState   = HIGH;
unsigned long pressStart  = 0;
bool          longHandled = false;
const unsigned long debounceMs = 30;
const unsigned long LONG_MS    = 3000;

/* ── 백라이트 깜빡임 스케줄 ─────────────── */
bool          blinkActive   = false;
unsigned long blinkEndMs    = 0;
unsigned long nextBlinkMs   = 0;
bool          backOn        = true;
const unsigned long BLINK_DUR = 600;
const unsigned long BLINK_INT = 150;

/* ── 유틸 ───────────────────────────────── */
void hallISR() { pulseCnt++; }

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

  // ★ LiquidCrystal_I2C: init/backlight 방식
  delay(200);          // 전원 안정화
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("SPEED: --.- km/h   ");
  lcd.setCursor(0,1); lcd.print("TIME : --:--       ");
  lcd.setCursor(0,2); lcd.print("BEST : --:--(--)   ");
  lcd.setCursor(0,3); lcd.print("LAP  : 0           ");
}

/* ── LOOP ──────────────────────────────── */
void loop() {
  unsigned long now = millis();

  /* -------- 버튼 처리 -------------------- */
  int reading = digitalRead(BTN_PIN);

  /* Falling Edge */
  if (reading == LOW && lastBtnState == HIGH) {
    pressStart  = now;
    longHandled = false;
  }

  /* 길게 누름(3 s) → 리셋 */
  if (reading == LOW && !longHandled && (now - pressStart >= LONG_MS)) {
    running = false;
    startMs = lastLapMs = bestLapMs = 0;
    lapCount = 0;

    lcd.setCursor(7,1); lcd.print("--:--     ");
    lcd.setCursor(7,2); lcd.print("--:--(--) ");
    lcd.setCursor(7,3); lcd.print("0   ");

    longHandled = true;
  }

  /* Rising Edge → 스타트/랩 */
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
    noInterrupts();
    unsigned long delta = pulseCnt - lastPulseCnt;
    lastPulseCnt = pulseCnt;
    interrupts();

    float motorRpm = (delta * 60.0 * 1000.0) / (PPR * (now - lastSpdCalc));
    float wheelRpm = motorRpm / TRANS_RATIO;
    float kmh = (wheelRpm * 3.1416 * TIRE_DIAM_CM * 60.0) / 100000.0;

    /* 0 km/h → 1600 µs,  KMH_MAX km/h → 2000 µs */
    float kmhClamp = constrain(kmh, 0.0, KMH_MAX);
    int thUs = SERVO_IDLE_US +
               (int)(kmhClamp * (SERVO_MAX_US - SERVO_IDLE_US) / KMH_MAX);
    exhaust.writeMicroseconds(thUs);

    lcd.setCursor(7,0);
    if (kmh < 10) lcd.print(' ');
    lcd.print(kmh, 1); lcd.print(" km/h   ");

    lastSpdCalc = now;
  }

  /* -------- 스톱워치 갱신 ----------------- */
  if (running) {
    char buf[6]; mmss(now - startMs, buf);
    lcd.setCursor(7,1); lcd.print(buf); lcd.print("     ");
  }
}
