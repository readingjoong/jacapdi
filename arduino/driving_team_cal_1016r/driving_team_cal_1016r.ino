// =====================================================================================
//  자캡디 주행 펌웨어  driving_team_cal_1016r  (Arduino Mega 2560)
//  "노트북이 USB로 보낸 글자 명령 → 조향 모터 + 뒷바퀴 모터 2개를 움직인다"
// =====================================================================================
//
//  ■ 명령 형식 (노트북 → 아두이노, 115200 baud, 한 줄씩)
//      s<조향 단계>l<왼쪽 뒷바퀴 PWM>r<오른쪽 뒷바퀴 PWM>\n
//      예) s30l240r240   : 오른쪽으로 30% , 뒷바퀴 둘 다 240
//          s-100l200r200 : 왼쪽 끝까지(100%), 뒷바퀴 200
//          s0l0r0        : 모든 모터 끄기 (정지)
//      조향: -100(왼쪽 끝) ~ 0(가운데) ~ +100(오른쪽 끝)   [1010f: 예전 ±7 단계 → ±100 (1% 단위)]
//      PWM: -255 ~ 255 (모터 힘. 0 = 정지, 255 = 최대, 음수 = 뒤로)
//
//  ■ 조향은 어떻게 하나? (피드백 제어)
//      앞바퀴 축에 가변저항(포텐셔미터)이 붙어 있어서, 바퀴가 돌면 A2 핀 전압이 변한다.
//      아두이노는 그 전압을 0~1023 숫자(ADC 값)로 읽는다.
//        왼쪽 끝 ≈ 543,  가운데 ≈ 462 (1009b 실측),  오른쪽 끝 ≈ 391   (2026-10-02 새 가변저항, 손으로 잰 값)
//  ■ 1016p: 1010p 와 같고 가변저항 기준만 다시 잼 (2026-10-16 손으로): 왼쪽 끝 530 (←543), 가운데 462 (그대로, 4번 잰 평균), 오른쪽 끝 400 (←391). 양 끝은 3번 잰 값(525/535/530, 392/401/400) 중 마지막. 가운데는 474/450/457/467 로 24칸 흔들려 평균
//  ■ 1016q: 1016p 와 같고, D 항이 힘을 0 으로 꺼버리지 않게 함. 목표에서 데드밴드(±3) 밖이면 최소 STEER_PWM_MIN 은 항상 냄
//           (1016p 로그: 돌려야 할 때의 약 30% 에서 D 항 때문에 힘 0 → 목표 앞에서 미끄러지듯 멈춰 "확 안 꺾임")
//  ■ 1016r: 1016q 와 같고, 가변저항 값이 0/1023 처럼 범위 밖으로 '튀면' 바로 고장 잠금하지 않음.
//           튄 동안은 조향 모터만 쉬고(뒷바퀴는 그대로) 'ADC_GLITCH' 를 보고. ADC_BAD_FAULT_MS(1초) 넘게 계속 범위 밖일 때만 고장.
//      조향 단계를 목표 ADC 값으로 바꾸고(targetAdc), 지금 ADC 가 목표보다 작으면 왼쪽으로,
//      크면 오른쪽으로 조향 모터를 돌린다. 목표에 가까워지면(±ADC_DEADBAND) 멈춘다.
//
//  ■ 안전 장치 (이상하면 모든 모터를 끈다)
//      1) 0.3 초 동안 명령이 안 오면 → 정지 (노트북 프로그램이 죽거나 USB 가 빠졌을 때)
//      2) 형식이 틀린 명령, s0l0r0 → 정지
//      3) FAULT(고장 잠금) → 모든 모터 끄고, 보드를 리셋할 때까지 명령을 무시
//           - adc_out_of_range : 가변저항 값이 말이 안 되는 범위 (선 빠짐/합선)
//           - wrong_direction  : 조향을 돌렸는데 반대로 움직임 (배선 반대/센서 이상)
//           - stall            : 조향을 돌리는데 2초 동안 목표 쪽으로 안 감 (걸림/힘 부족)
//
//  ■ 1009b: 가운데 POT_CENTER 467 → 462 (실측). 나머지는 1009 와 같음
//  ■ 1010: 조향 더 빠르게. STEERING_PWM 180 → 220, STEERING_PWM_NEAR 95 → 130, NEAR_COUNTS 30 → 15
//  ■ 1010f: 조향 세밀하게. 명령 ±7 단계 → ±100 (1% 단위, ADC 1칸 ≈ 0.8%). 예전 s7 = 지금 s100
//           ADC_DEADBAND 6 → 3, 목표 6칸 안에서는 더 약하게(STEERING_PWM_FINE 100) 다가감
//           "도착" 해제 기준을 단계 번호가 아니라 목표 ADC 가 DEADBAND 넘게 바뀌었을 때로 (1% 씩 바뀔 때 떨림 방지)
//           SETTLE_COUNTS 12 → 8
//  ■ 1010p: 조향 모터 힘을 3단계(220/130/100) 대신 PD 제어로
//           힘 = STEER_PWM_MIN + KP × |목표까지 남은 칸| − KD × (목표 쪽으로 다가가는 속도, 칸/0.02초)   (255 에서 자름)
//           멀면 255 로 세게, 가까워질수록 비례해서 약하게, 빨리 다가오면 미리 힘을 빼서 지나치지 않게.
//           힘이 0 이하로 계산되면 그 순간은 모터를 끄고 미끄러지게 둠 (반대로 돌려 브레이크 걸지 않음 → 고장 판정과 충돌 없음)
//           방향/데드밴드(±3)/도착 판정/고장 판정은 1010f 와 같다. 0.2초 보고에 P=<그때 모터 힘> 추가
//
//  ■ 1002b 에서 바뀐 것 (1009)
//      끝 여유 END_MARGIN 왼쪽 8→3, 오른쪽 6→3  → 조향을 끝까지 조금 더 꺾는다 (커브 대응)
//      나머지 동작은 1002b 와 같다. 주석만 아주 자세히 달았다.
//
//  ■ 핀 연결
//      조향 모터 드라이버: 2번, 3번 핀      가변저항 신호: A2 (5V/GND 는 모듈 표시대로)
//      오른쪽 뒷바퀴: 4번, 5번 핀           왼쪽 뒷바퀴: 6번, 7번 핀
// =====================================================================================

#include <Arduino.h>
#include <stdlib.h>          // strtol (글자 → 숫자 변환)

// ---------- 핀 번호 ----------
const uint8_t STEERING_1 = 2;   // 조향 모터: 이 핀에 PWM → 오른쪽으로 (ADC 값이 줄어드는 쪽)
const uint8_t STEERING_2 = 3;   // 조향 모터: 이 핀에 PWM → 왼쪽으로 (ADC 값이 커지는 쪽)
const uint8_t RIGHT_1 = 4;      // 오른쪽 뒷바퀴: 앞으로
const uint8_t RIGHT_2 = 5;      // 오른쪽 뒷바퀴: 뒤로
const uint8_t LEFT_1 = 6;       // 왼쪽 뒷바퀴: 앞으로
const uint8_t LEFT_2 = 7;       // 왼쪽 뒷바퀴: 뒤로
const uint8_t POT = A2;         // 조향 가변저항 신호

// ---------- 조향 가변저항 보정값 (바퀴 위치 ↔ ADC 숫자) ----------
const int POT_LEFT = 530;       // [1016] 왼쪽 끝까지 돌렸을 때 ADC (2026-10-16 손으로 3번 잼 525/535/530 중 마지막, 전엔 543)
const int POT_CENTER = 462;     // [1016] 똑바로일 때 ADC. 손으로 4번 잼 474/450/457/467 → 평균 462 (예전 값과 같음)
const int POT_RIGHT = 400;      // [1016] 오른쪽 끝까지 돌렸을 때 ADC (3번 잼 392/401/400 중 마지막, 전엔 391)
const int END_MARGIN = 3;       // [1009] 왼쪽 끝에서 이만큼 덜 가서 멈춤 (기계 끝에 쾅 부딪히지 않게)
const int END_MARGIN_RIGHT = 3; // [1009] 오른쪽 끝 여유
const int LEFT_LIMIT = POT_LEFT - END_MARGIN;          // 왼쪽 최대 목표 = 527
const int RIGHT_LIMIT = POT_RIGHT + END_MARGIN_RIGHT;  // 오른쪽 최대 목표 = 403
const uint32_t ADC_BAD_FAULT_MS = 1000; // [1016r] 범위 밖이 이 시간 넘게 계속되면 그때 고장 (선이 정말 빠진 경우)
const int ADC_VALID_MARGIN = 40;  // 360~570 밖이면 센서 고장으로 봄 (FAULT adc_out_of_range)

// ---------- 조향 모터 제어 ----------
const int ADC_DEADBAND = 3;       // [1010f] 목표 ± 3 안이면 "도착" → 조향 모터 멈춤 (6 → 3, 약 ±1°)
const int MAX_STEERING_STEP = 100; // [1010f] 조향 명령 최대 (s-100 ~ s100, 1% 단위)
const int MAX_PWM = 255;          // 뒷바퀴 PWM 최대
const int NEAR_COUNTS = 15;       // [1010] "가깝다" 의 기준 (ADC 15칸) - 1010p 에서는 도착 해제 판정에만 씀
// [1010p] PD 제어 (조향 모터 힘)
const int STEER_PWM_MIN = 90;     // 모터가 움직이기 시작하는 최소 힘 (1009b 에서 95 로 50 칸/초 움직임 확인)
const float STEER_KP = 8.0;       // 남은 1 칸마다 힘 +8  (21 칸 넘게 남으면 255)
const float STEER_KD = 12.0;      // 목표 쪽으로 0.02초에 1 칸 다가갈 때마다 힘 −12 (지나침 방지)

// ---------- 고장 판정 ----------
const uint32_t REVERSE_GRACE_MS = 400;  // 방향을 바꾼 직후 0.4 초는 관성으로 밀리는 것 봐줌
const int REVERSE_COUNTS = 15;          // 명령 반대 방향으로 15칸 이상 가면 wrong_direction
const int SETTLE_COUNTS = 8;            // [1010f] 목표 8칸 안에서 멈춰 있으면 "도착" 인정 (12 → 8)
const uint32_t STEERING_STALL_MS = 2000;// 목표에서 멀리 2초 동안 안 움직이면 stall

// ---------- 시간 ----------
const uint32_t RX_TIMEOUT_MS = 300;     // 이 시간 동안 새 명령이 없으면 정지
const uint32_t CONTROL_PERIOD_MS = 20;  // 조향 제어를 0.02 초(50 Hz)마다

// ---------- 지금 상태를 기억하는 변수들 ----------
int steering = 0;              // 마지막으로 받은 조향 단계
int leftPwm = 0;               // 마지막으로 받은 왼쪽 뒷바퀴 PWM
int rightPwm = 0;              // 마지막으로 받은 오른쪽 뒷바퀴 PWM
bool active = false;           // 유효한 주행 명령을 받는 중인가
bool faulted = false;          // 고장 잠금 상태인가 (리셋 전까지 유지)
uint32_t lastRx = 0;           // 마지막 명령을 받은 시각 (ms)
uint32_t lastControl = 0;      // 마지막 조향 제어 시각
int watchedDirection = 0;      // 지금 조향 모터를 돌리는 방향 (+1 왼쪽, -1 오른쪽, 0 멈춤)
int progressAdc = POT_CENTER;  // 고장 판정용: 마지막으로 "앞으로 나아갔다" 고 본 ADC
uint32_t progressSince = 0;    // 그 시각
uint32_t lastReport = 0;       // 노트북에 상태를 마지막으로 보낸 시각
uint32_t directionSince = 0;   // 지금 방향으로 돌리기 시작한 시각
bool settled = false;          // 목표 근처에서 "도착" 인정된 상태인가
int settledTarget = 0;         // [1010f] 그때의 목표 ADC
int prevAdc = -1;              // [1010p] 직전 제어 주기의 ADC (속도 계산용, -1 = 아직 없음)
bool adcBad = false;           // [1016r] 지금 가변저항 값이 범위 밖인가
uint32_t adcBadSince = 0;      // [1016r] 범위 밖이 시작된 시각
int lastSteerPwm = 0;          // [1010p] 마지막으로 낸 조향 모터 힘 (보고용)
char inputLine[32];            // USB 로 들어오는 글자를 한 줄 모으는 칸
uint8_t inputLength = 0;
bool overflowed = false;       // 한 줄이 너무 길었나 (잘못된 명령)

// 모터 하나 끄기 (두 핀 모두 0)
void motorOff(uint8_t pin1, uint8_t pin2) {
  analogWrite(pin1, 0);
  analogWrite(pin2, 0);
}

// 모든 모터 끄고 상태 초기화
void stopAll() {
  motorOff(STEERING_1, STEERING_2);
  motorOff(LEFT_1, LEFT_2);
  motorOff(RIGHT_1, RIGHT_2);
  active = false;
  steering = 0;
  leftPwm = 0;
  rightPwm = 0;
  watchedDirection = 0;
}

// 고장 잠금: 모든 모터 끄고, 이유와 그때 ADC 를 노트북에 알림
void latchFault(const __FlashStringHelper *why, int adc) {
  faulted = true;
  stopAll();
  Serial.print(F("FAULT: outputs OFF; reason=")); Serial.print(why);
  Serial.print(F(" ADC=")); Serial.println(adc);
}

// 조향 명령(-100~+100) → 목표 ADC 값
//   왼쪽(음수): 가운데에서 왼쪽 최대까지를 100등분 (반올림)
//   오른쪽(양수): 가운데에서 오른쪽 최대까지를 100등분
//   예) -100 → 527,  0 → 462,  +100 → 403,  +50 → 432
int targetAdc(int step) {
  if (step < 0) {
    return POT_CENTER + ((LEFT_LIMIT - POT_CENTER) * (-step) + MAX_STEERING_STEP / 2) / MAX_STEERING_STEP;
  }
  return POT_CENTER - ((POT_CENTER - RIGHT_LIMIT) * step + MAX_STEERING_STEP / 2) / MAX_STEERING_STEP;
}

// 글자에서 정수 하나 읽기 (범위 밖이면 실패)
bool boundedInt(const char *&p, int minimum, int maximum, int &result) {
  const char *digits = p;
  if (*digits == '-' || *digits == '+') ++digits;
  if (*digits < '0' || *digits > '9') return false;
  char *end;
  long value = strtol(p, &end, 10);
  if (value < minimum || value > maximum) return false;
  p = end;
  result = static_cast<int>(value);
  return true;
}

// 완성된 한 줄 명령 해석: "s30l240r240" → steering=30, leftPwm=240, rightPwm=240
void processData(const char *data) {
  const char *p = data;
  int nextSteering, nextLeft, nextRight;
  if (*p++ != 's' || !boundedInt(p, -MAX_STEERING_STEP, MAX_STEERING_STEP, nextSteering) ||
      *p++ != 'l' || !boundedInt(p, -MAX_PWM, MAX_PWM, nextLeft) ||
      *p++ != 'r' || !boundedInt(p, -MAX_PWM, MAX_PWM, nextRight) || *p != '\0') {
    stopAll();                 // 형식이 틀리면 안전하게 정지
    return;
  }
  if (faulted || (nextSteering == 0 && nextLeft == 0 && nextRight == 0)) {
    stopAll();                 // 고장 잠금 중이거나 s0l0r0 이면 정지
    return;
  }
  steering = nextSteering;
  leftPwm = nextLeft;
  rightPwm = nextRight;
  lastRx = millis();           // 명령 받은 시각 기록 (0.3 초 타임아웃용)
  active = true;
}

// USB 로 들어온 글자 하나씩 모으기. 줄바꿈('\n')이 오면 한 줄 완성 → processData
void receiveByte(char value) {
  if (value == '\r') return;
  if (value == '\n') {
    if (!overflowed) {
      inputLine[inputLength] = '\0';
      processData(inputLine);
    }
    inputLength = 0;
    overflowed = false;
  } else if (!overflowed) {
    if (inputLength < sizeof(inputLine) - 1) {
      inputLine[inputLength++] = value;
    } else {
      overflowed = true;       // 너무 긴 줄 = 잘못된 명령 → 정지
      stopAll();
    }
  }
}

// 뒷바퀴 모터 하나 돌리기 (양수 = 앞으로 pin1, 음수 = 뒤로 pin2)
void writeRear(uint8_t pin1, uint8_t pin2, int demand) {
  motorOff(pin1, pin2);
  if (demand > 0) analogWrite(pin1, demand);
  if (demand < 0) analogWrite(pin2, -demand);
}

// 전원이 켜지거나 리셋될 때 한 번
void setup() {
  const uint8_t pins[] = {STEERING_1, STEERING_2, LEFT_1, LEFT_2, RIGHT_1, RIGHT_2};
  for (uint8_t i = 0; i < sizeof(pins); ++i) {
    digitalWrite(pins[i], LOW);    // 시작할 때 모든 모터 핀 0 (갑자기 움직이지 않게)
    pinMode(pins[i], OUTPUT);
  }
  pinMode(POT, INPUT);
  stopAll();
  Serial.begin(115200);
  Serial.println(F("DRIVING_TEAM_CAL_1016R IDLE L=530 C=462 R=400"));   // 노트북이 이 글자로 펌웨어 확인
}

// 계속 반복
void loop() {
  // (1) USB 로 들어온 글자 처리
  while (Serial.available() > 0) receiveByte(static_cast<char>(Serial.read()));
  uint32_t now = millis();

  // (2) 명령이 없거나, 고장이거나, 0.3 초 넘게 명령이 끊기면 → 정지. 0.5 초마다 ADC 만 알려줌
  if (!active || faulted || static_cast<uint32_t>(now - lastRx) > RX_TIMEOUT_MS) {
    stopAll();
    prevAdc = -1;                                // [1010p] 다시 시작할 때 속도를 0 부터
    if (static_cast<uint32_t>(now - lastReport) >= 500) {
      lastReport = now;
      Serial.print(faulted ? F("FAULTED ADC=") : F("IDLE ADC="));
      Serial.println(analogRead(POT));
    }
    return;
  }

  // (3) 조향 제어는 0.02 초마다
  if (static_cast<uint32_t>(now - lastControl) < CONTROL_PERIOD_MS) return;
  lastControl = now;

  // (4) 가변저항 4번 읽어 평균 (잡음 줄이기)
  int adc = 0;
  for (uint8_t i = 0; i < 4; ++i) adc += analogRead(POT);
  adc = (adc + 2) / 4;
  if (adc < POT_RIGHT - ADC_VALID_MARGIN || adc > POT_LEFT + ADC_VALID_MARGIN) {
    // [1016r] 한 번 튄 걸로는 멈추지 않음: 이번 주기는 조향 모터만 쉬고 뒷바퀴는 그대로
    if (!adcBad) {
      adcBad = true;
      adcBadSince = now;
      Serial.print(F("ADC_GLITCH ADC=")); Serial.println(adc);
    }
    if (static_cast<uint32_t>(now - adcBadSince) > ADC_BAD_FAULT_MS) {
      latchFault(F("adc_out_of_range"), adc);  // 1초 넘게 계속 범위 밖 → 선이 빠진 것 → 고장
      return;
    }
    motorOff(STEERING_1, STEERING_2);
    lastSteerPwm = 0;
    writeRear(LEFT_1, LEFT_2, leftPwm);
    writeRear(RIGHT_1, RIGHT_2, rightPwm);
    prevAdc = -1;                                // 튄 값으로 속도(D) 계산하지 않게
    watchedDirection = 0;                        // 고장 판정 기준점도 다시 잡게
    return;
  }
  if (adcBad) {
    adcBad = false;
    Serial.print(F("ADC_GLITCH_END ms=")); Serial.println(now - adcBadSince);
  }

  // (5) 목표와의 차이 → 조향 모터를 어느 쪽으로 돌릴지
  int target = targetAdc(steering);
  int error = target - adc;                    // + 면 왼쪽으로 더 가야 함
  int direction = 0;                           // +1: 왼쪽(ADC 증가), -1: 오른쪽(ADC 감소), 0: 멈춤
  if (error > ADC_DEADBAND && adc < LEFT_LIMIT) direction = 1;
  if (error < -ADC_DEADBAND && adc > RIGHT_LIMIT) direction = -1;
  if (settled && (abs(target - settledTarget) > ADC_DEADBAND || abs(error) > NEAR_COUNTS)) settled = false;  // [1010f] 목표가 DEADBAND 넘게 바뀌면 도착 해제
  if (settled) direction = 0;                  // 이미 도착 인정이면 그대로 둠

  // (6) 0.2 초마다 노트북에 상태 보고 (board_monitor.py 가 기록)
  if (static_cast<uint32_t>(now - lastReport) >= 200) {
    lastReport = now;
    Serial.print(F("ADC=")); Serial.print(adc);
    Serial.print(F(" T=")); Serial.print(target);
    Serial.print(F(" S=")); Serial.print(steering);
    Serial.print(F(" D=")); Serial.print(direction);
    Serial.print(F(" P=")); Serial.println(lastSteerPwm);
  }

  // (7) 고장 판정: 돌리는 방향으로 실제 움직이고 있나?
  if (direction == 0) {
    watchedDirection = 0;
  } else if (direction != watchedDirection) {  // 방향을 새로 시작/바꿈 → 기준점 다시 잡기
    watchedDirection = direction;
    progressAdc = adc;
    progressSince = now;
    directionSince = now;
  } else {
    int progress = (adc - progressAdc) * direction;   // 원하는 방향으로 몇 칸 갔나 (+ 좋음)
    bool inGrace = static_cast<uint32_t>(now - directionSince) < REVERSE_GRACE_MS;
    if (progress >= 2) {                         // 잘 가고 있음
      progressAdc = adc;
      progressSince = now;
    } else if (inGrace && progress < 0) {        // 방향 바꾼 직후 관성 → 봐줌
      progressAdc = adc;
      progressSince = now;
    } else if (progress <= -REVERSE_COUNTS) {    // 반대로 크게 감 → 고장
      latchFault(F("wrong_direction"), adc);
      return;
    } else if (abs(error) <= SETTLE_COUNTS && static_cast<uint32_t>(now - progressSince) > 300) {
      settled = true;                            // 목표 근처에서 0.3 초 멈춤 → 도착 인정 (끝이 뻑뻑할 때)
      settledTarget = target;
      watchedDirection = 0;
      direction = 0;
      Serial.print(F("SETTLED ADC=")); Serial.print(adc); Serial.print(F(" T=")); Serial.println(target);
    } else if (static_cast<uint32_t>(now - progressSince) > STEERING_STALL_MS) {
      latchFault(F("stall"), adc);               // 멀리서 2초 동안 못 움직임 → 고장
      return;
    }
  }

  // (8) [1010p] PD 로 조향 모터 힘 계산 후 돌리기
  int vel = (prevAdc < 0) ? 0 : adc - prevAdc;            // 0.02 초 동안 움직인 칸 (+ = 왼쪽)
  prevAdc = adc;
  int steerPwm = 0;
  if (direction != 0) {
    float u = STEER_KP * abs(error) - STEER_KD * (vel * direction);   // 다가가는 중이면 vel*direction > 0 → 힘 줄임
    if (u < 0) u = 0;                                      // [1016q] D 항이 힘을 0 으로 끄지 않게: 데드밴드 밖이면 최소 STEER_PWM_MIN
    steerPwm = constrain(STEER_PWM_MIN + static_cast<int>(u), STEER_PWM_MIN, 255);
  }
  lastSteerPwm = steerPwm;
  motorOff(STEERING_1, STEERING_2);
  if (direction > 0) analogWrite(STEERING_2, steerPwm);   // 왼쪽으로
  if (direction < 0) analogWrite(STEERING_1, steerPwm);   // 오른쪽으로
  writeRear(LEFT_1, LEFT_2, leftPwm);                      // 뒷바퀴는 받은 PWM 그대로
  writeRear(RIGHT_1, RIGHT_2, rightPwm);
}
