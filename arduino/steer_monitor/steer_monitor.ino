//  자캡디 조향 시험 스케치  steer_monitor  (Arduino Mega 2560)  - 2026-10-17
//
//  Arduino IDE 시리얼 모니터(또는 시리얼 플로터)로 조향 가변저항 값을 보면서 조향 명령을 손으로 넣는 벤치 시험용.
//  주행 펌웨어가 아님! 시험 끝나면 car_flash.sh 로 주행 펌웨어(1016R 등)를 다시 올릴 것.
//
//  ■ 설정: 115200 baud, 줄 끝 = "새 줄(Newline)"
//  ■ 명령 (시리얼 모니터 입력칸에 치고 Enter)
//     -100 ~ 100 : 조향 목표 (%) — 주행 펌웨어와 같은 변환. -100 = 왼쪽 끝(527), 0 = 가운데(462), 100 = 오른쪽 끝(403)
//                  보낸 목표를 계속 유지함 (주행 펌웨어처럼 0.3초 뒤 멈추지 않음)
//     f          : 조향 모터 끄고 값만 보기 (손으로 바퀴 돌리기·선 흔들기 시험)
//  ■ 출력 (0.02초마다):  raw:<한 번 읽은 값> avg:<4번 평균> target:<목표> pwm:<조향 모터 힘>
//     시리얼 플로터에서 raw 가 1023/0 으로 뾰족하게 튀면 배선 접촉 불량
//  ■ 안전: 뒷바퀴는 항상 꺼짐. 값이 360~570 밖이면 그 순간 조향 모터 끔. 2초 동안 못 움직이면 "STALL" 하고 f 모드로.

const uint8_t STEERING_1 = 2;   // 이 핀에 PWM → 오른쪽 (ADC 감소)
const uint8_t STEERING_2 = 3;   // 이 핀에 PWM → 왼쪽 (ADC 증가)
const uint8_t RIGHT_1 = 4, RIGHT_2 = 5, LEFT_1 = 6, LEFT_2 = 7;
const uint8_t POT = A2;

const int POT_LEFT = 530, POT_CENTER = 462, POT_RIGHT = 400;   // driving_team_cal_1016r 과 같은 값
const int LEFT_LIMIT = POT_LEFT - 3, RIGHT_LIMIT = POT_RIGHT + 3;
const int ADC_MIN = POT_RIGHT - 40, ADC_MAX = POT_LEFT + 40;
const int DEADBAND = 3;
const int PWM_MIN = 90;          // 1016q 와 같은 PD
const float KP = 8.0, KD = 12.0;

bool hold = false;               // true = 목표 유지, false = 모터 끔(f 모드)
int target = POT_CENTER;
int prevAdc = -1;
int progressAdc = 0;
uint32_t progressSince = 0, lastControl = 0;
char line[16]; uint8_t len = 0;

int targetAdc(int step) {        // 주행 펌웨어 targetAdc() 와 같은 식
  if (step < 0) return POT_CENTER + ((LEFT_LIMIT - POT_CENTER) * (-step) + 50) / 100;
  return POT_CENTER - ((POT_CENTER - RIGHT_LIMIT) * step + 50) / 100;
}

void steerOff() { analogWrite(STEERING_1, 0); analogWrite(STEERING_2, 0); }

void command(const char* s) {
  if (s[0] == 'f' || s[0] == 'F') { hold = false; steerOff(); Serial.println(F("# f: 조향 모터 끔, 값만 봄")); return; }
  char* end; long v = strtol(s, &end, 10);
  if (end == s || v < -100 || v > 100) { Serial.println(F("# 명령은 -100~100 또는 f")); return; }
  target = targetAdc(static_cast<int>(v)); hold = true;
  progressSince = millis(); progressAdc = analogRead(POT);
  Serial.print(F("# 목표 ")); Serial.print(v); Serial.print(F("% -> ADC ")); Serial.println(target);
}

void setup() {
  const uint8_t pins[] = {STEERING_1, STEERING_2, RIGHT_1, RIGHT_2, LEFT_1, LEFT_2};
  for (uint8_t i = 0; i < 6; ++i) { pinMode(pins[i], OUTPUT); analogWrite(pins[i], 0); }
  pinMode(POT, INPUT);
  Serial.begin(115200);
  Serial.println(F("# STEER_MONITOR L=530 C=462 R=400. 명령: -100~100 (조향 %), f (모터 끔)"));
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (len) { line[len] = 0; command(line); len = 0; } }
    else if (len < sizeof(line) - 1) line[len++] = c;
  }
  uint32_t now = millis();
  if (now - lastControl < 20) return;
  lastControl = now;

  int raw = analogRead(POT);
  int avg = raw; for (uint8_t i = 0; i < 3; ++i) avg += analogRead(POT); avg = (avg + 2) / 4;

  int pwm = 0;
  if (hold && avg >= ADC_MIN && avg <= ADC_MAX) {
    int error = target - avg;
    int dir = 0;
    if (error > DEADBAND && avg < LEFT_LIMIT) dir = 1;
    if (error < -DEADBAND && avg > RIGHT_LIMIT) dir = -1;
    int vel = (prevAdc < 0) ? 0 : avg - prevAdc;
    if (dir != 0) {
      float u = KP * abs(error) - KD * (vel * dir); if (u < 0) u = 0;
      pwm = constrain(PWM_MIN + static_cast<int>(u), PWM_MIN, 255);
      if ((avg - progressAdc) * dir >= 2) { progressAdc = avg; progressSince = now; }
      else if (now - progressSince > 2000) { hold = false; pwm = 0; Serial.println(F("# STALL: 2초 동안 못 움직임 -> f 모드")); }
    } else { progressAdc = avg; progressSince = now; }
    steerOff();
    if (pwm > 0 && dir > 0) analogWrite(STEERING_2, pwm);
    if (pwm > 0 && dir < 0) analogWrite(STEERING_1, pwm);
  } else {
    steerOff();                   // f 모드이거나 값이 범위 밖 → 조향 모터 끔
  }
  prevAdc = avg;

  Serial.print(F("raw:")); Serial.print(raw);
  Serial.print(F(" avg:")); Serial.print(avg);
  Serial.print(F(" target:")); Serial.print(hold ? target : 0);
  Serial.print(F(" pwm:")); Serial.println(pwm);
}
