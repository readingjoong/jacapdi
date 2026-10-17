//  자캡디 조향 PWM 시험 스케치  steer_pwm_test  (Arduino Mega 2560)  - 2026-10-17
//
//  조향 모터에 내가 정한 PWM 을 정한 시간만큼 넣고, 바퀴(가변저항 ADC)가 얼마나 움직였는지 잰다.
//  주행 펌웨어가 아님! 시험 끝나면 car_flash.sh 로 주행 펌웨어를 다시 올릴 것.
//
//  ■ 설정: Arduino IDE 시리얼 모니터 115200 baud, 줄 끝 = "새 줄(Newline)"
//  ■ 명령
//     l<PWM> [ms]  : 왼쪽으로 PWM 만큼 ms 동안 (예: l150 300 → 왼쪽 PWM 150 으로 0.3초). ms 생략하면 300
//     r<PWM> [ms]  : 오른쪽으로
//     c            : 가운데(462)로 돌아가기 (PWM 150, ±3 칸 안이면 멈춤)
//     a            : 지금 ADC 한 번 보기
//  ■ 출력
//     펄스 동안 0.01초마다  t=<ms> adc=<값>
//     끝나고 요약: 시작 ADC, 끝 ADC, 움직인 칸, 대략 각도, 최고 속도(칸/초), 모터 끈 뒤 관성으로 더 간 칸
//  ■ 안전: 뒷바퀴는 항상 꺼짐. 왼쪽 527 / 오른쪽 403 에 닿거나 값이 360~570 밖이면 즉시 모터 끔. 펄스 최대 2초.

const uint8_t STEERING_1 = 2;   // 이 핀에 PWM → 오른쪽 (ADC 감소)
const uint8_t STEERING_2 = 3;   // 이 핀에 PWM → 왼쪽 (ADC 증가)
const uint8_t RIGHT_1 = 4, RIGHT_2 = 5, LEFT_1 = 6, LEFT_2 = 7;
const uint8_t POT = A2;

const int POT_LEFT = 530, POT_CENTER = 462, POT_RIGHT = 400;   // driving_team_cal_1016r 과 같은 값
const int LEFT_LIMIT = POT_LEFT - 3, RIGHT_LIMIT = POT_RIGHT + 3;
const int ADC_MIN = POT_RIGHT - 40, ADC_MAX = POT_LEFT + 40;
const float DEG_PER_COUNT_LEFT = 0.2846, DEG_PER_COUNT_RIGHT = 0.3040;  // vehicle.yaml (10/16 회전 원). 참고용 대략값
const uint32_t MAX_PULSE_MS = 2000;
const uint32_t COAST_MS = 300;  // 모터 끈 뒤 관성으로 더 가는 걸 이 시간 동안 봄

char line[24]; uint8_t len = 0;

int readAdc() { int s = 0; for (uint8_t i = 0; i < 4; ++i) s += analogRead(POT); return (s + 2) / 4; }
void steerOff() { analogWrite(STEERING_1, 0); analogWrite(STEERING_2, 0); }
void drive(int dir, int pwm) {   // dir +1 왼쪽, -1 오른쪽
  steerOff();
  if (dir > 0) analogWrite(STEERING_2, pwm);
  if (dir < 0) analogWrite(STEERING_1, pwm);
}
bool unsafe(int adc, int dir) {
  if (adc < ADC_MIN || adc > ADC_MAX) { Serial.print(F("# 값이 범위 밖 (배선?) ADC=")); Serial.println(adc); return true; }
  if (dir > 0 && adc >= LEFT_LIMIT) { Serial.println(F("# 왼쪽 끝(527) 도착 → 모터 끔")); return true; }
  if (dir < 0 && adc <= RIGHT_LIMIT) { Serial.println(F("# 오른쪽 끝(403) 도착 → 모터 끔")); return true; }
  return false;
}

void pulse(int dir, int pwm, uint32_t ms) {
  pwm = constrain(pwm, 0, 255);
  ms = min(ms, MAX_PULSE_MS);
  int start = readAdc();
  Serial.print(F("# 시작: ")); Serial.print(dir > 0 ? F("왼쪽") : F("오른쪽"));
  Serial.print(F(" PWM ")); Serial.print(pwm); Serial.print(F(", ")); Serial.print(ms); Serial.print(F(" ms, ADC ")); Serial.println(start);
  if (unsafe(start, dir)) return;

  uint32_t t0 = millis(), lastLog = 0; int prev = start, maxSpeed = 0, adc = start, firstMoveMs = -1;
  drive(dir, pwm);
  while (millis() - t0 < ms) {
    uint32_t t = millis() - t0;
    if (t - lastLog >= 10) {
      adc = readAdc();
      int sp = (adc - prev) * dir * 100;             // 칸/초 (0.01초 간격)
      if (sp > maxSpeed) maxSpeed = sp;
      if (firstMoveMs < 0 && (adc - start) * dir >= 2) firstMoveMs = t;
      prev = adc; lastLog = t;
      Serial.print(F("t=")); Serial.print(t); Serial.print(F(" adc=")); Serial.println(adc);
      if (unsafe(adc, dir)) break;
    }
  }
  steerOff();
  int atOff = readAdc();
  uint32_t tOff = millis();
  while (millis() - tOff < COAST_MS) { }             // 관성으로 더 가는지 기다림
  int end = readAdc();

  int moved = (end - start) * dir;
  float deg = moved * (dir > 0 ? DEG_PER_COUNT_LEFT : DEG_PER_COUNT_RIGHT);
  Serial.print(F("# 결과: ADC ")); Serial.print(start); Serial.print(F(" -> ")); Serial.print(end);
  Serial.print(F("  움직임 ")); Serial.print(moved); Serial.print(F(" 칸 (약 ")); Serial.print(deg, 1); Serial.print(F("도)"));
  Serial.print(F("  최고속도 ")); Serial.print(maxSpeed); Serial.print(F(" 칸/초"));
  Serial.print(F("  움직이기 시작 ")); if (firstMoveMs < 0) Serial.print(F("안 움직임")); else { Serial.print(firstMoveMs); Serial.print(F(" ms")); }
  Serial.print(F("  관성 ")); Serial.print((end - atOff) * dir); Serial.println(F(" 칸"));
}

void center() {
  Serial.println(F("# 가운데로"));
  uint32_t t0 = millis();
  while (millis() - t0 < MAX_PULSE_MS) {
    int adc = readAdc();
    if (adc < ADC_MIN || adc > ADC_MAX) { Serial.println(F("# 값이 범위 밖 → 멈춤")); break; }
    int err = POT_CENTER - adc;
    if (abs(err) <= 3) break;
    drive(err > 0 ? 1 : -1, 150);
    delay(10);
  }
  steerOff(); delay(200);
  Serial.print(F("# 가운데 ADC ")); Serial.println(readAdc());
}

void command(char* s) {
  char c = s[0] | 0x20;                               // 소문자로
  if (c == 'a') { Serial.print(F("# ADC ")); Serial.println(readAdc()); return; }
  if (c == 'c') { center(); return; }
  if (c == 'l' || c == 'r') {
    char* p = s + 1; long pwm = strtol(p, &p, 10); long ms = strtol(p, &p, 10);
    if (pwm <= 0 || pwm > 255) { Serial.println(F("# PWM 은 1~255 (예: l150 300)")); return; }
    if (ms <= 0) ms = 300;
    pulse(c == 'l' ? 1 : -1, static_cast<int>(pwm), static_cast<uint32_t>(ms));
    return;
  }
  Serial.println(F("# 명령: l<PWM> [ms], r<PWM> [ms], c (가운데), a (ADC 보기)"));
}

void setup() {
  const uint8_t pins[] = {STEERING_1, STEERING_2, RIGHT_1, RIGHT_2, LEFT_1, LEFT_2};
  for (uint8_t i = 0; i < 6; ++i) { pinMode(pins[i], OUTPUT); analogWrite(pins[i], 0); }
  pinMode(POT, INPUT);
  Serial.begin(115200);
  Serial.println(F("# STEER_PWM_TEST L=530 C=462 R=400"));
  Serial.println(F("# 명령: l<PWM> [ms], r<PWM> [ms], c (가운데), a (ADC 보기)   예) l150 300"));
  Serial.print(F("# 지금 ADC ")); Serial.println(readAdc());
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (len) { line[len] = 0; command(line); len = 0; } }
    else if (len < sizeof(line) - 1) line[len++] = c;
  }
}
