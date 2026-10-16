# 자캡디 실차 전체 코드 - 한 파일 모음 (2026-10-09)

읽기용입니다. 고칠 때는 폴더의 개별 파일을 고치세요. 순서: 아두이노 → 경로 → 제어 → 슬라이더 → 도구 → 스크립트 → 설정 → 팀 원본

## 목차
- `arduino/driving_team_cal_1009b/driving_team_cal_1009b.ino`
- `arduino/driving_team_cal_1009c/driving_team_cal_1009c.ino`
- `python/1_path/B_dashed_fallback.py`
- `python/1_path/E_curve_cross.py`
- `python/1_path/F_outlier_hold.py`
- `python/1_path/lane_from_lines.py`
- `python/2_control/team_lane_keeping_controller.py`
- `python/2_control/tunable_controller.py`
- `python/3_slider_gui/tune_gui.py`
- `python/4_tools/board_monitor.py`
- `python/4_tools/calib_pitch.py`
- `python/4_tools/frame_logger.py`
- `python/4_tools/motor_fixed_test.py`
- `python/4_tools/path_viewer.py`
- `python/4_tools/rate_probe.py`
- `python/4_tools/run_monitor.py`
- `python/4_tools/steer_sequence_test.py`
- `python/4_tools/webcam_pub.py`
- `scripts/car_all.sh`
- `scripts/car_common.sh`
- `scripts/car_flash.sh`
- `scripts/car_go.sh`
- `scripts/car_lap.sh`
- `scripts/car_prep.sh`
- `scripts/car_stop.sh`
- `scripts/run.sh`
- `scripts/start_ab.sh`
- `scripts/timed_drive.sh`
- `config/drive_controller_tunable.yaml`
- `config/ipm_c920.yaml`
- `config/tune_current.json`
- `config/tune_default.json`
- `config/presets/preset_1009_best.json`
- `config/presets/preset_1009_best2.json`
- `config/presets/preset_1009_best3.json`
- `config/presets/preset_1009_v3.json`
- `config/presets/preset_best_20261009_1455.json`
- `config/presets/preset_curve_aggressive.json`
- `config/presets/preset_curve_mid.json`
- `config/presets/preset_curve_mid_v1.json`
- `python/team_original/serial_sender_node.py`
- `python/team_original/yolov8_node.py`


---

## `arduino/driving_team_cal_1009b/driving_team_cal_1009b.ino`

```cpp
// =====================================================================================
//  자캡디 주행 펌웨어  driving_team_cal_1009b  (Arduino Mega 2560)
//  "노트북이 USB로 보낸 글자 명령 → 조향 모터 + 뒷바퀴 모터 2개를 움직인다"
// =====================================================================================
//
//  ■ 명령 형식 (노트북 → 아두이노, 115200 baud, 한 줄씩)
//      s<조향 단계>l<왼쪽 뒷바퀴 PWM>r<오른쪽 뒷바퀴 PWM>\n
//      예) s3l240r240   : 오른쪽으로 3단계, 뒷바퀴 둘 다 240
//          s-7l200r200  : 왼쪽 끝까지(7단계), 뒷바퀴 200
//          s0l0r0       : 모든 모터 끄기 (정지)
//      조향 단계: -7(왼쪽 끝) ~ 0(가운데) ~ +7(오른쪽 끝)
//      PWM: -255 ~ 255 (모터 힘. 0 = 정지, 255 = 최대, 음수 = 뒤로)
//
//  ■ 조향은 어떻게 하나? (피드백 제어)
//      앞바퀴 축에 가변저항(포텐셔미터)이 붙어 있어서, 바퀴가 돌면 A2 핀 전압이 변한다.
//      아두이노는 그 전압을 0~1023 숫자(ADC 값)로 읽는다.
//        왼쪽 끝 ≈ 543,  가운데 ≈ 462 (1009b 실측),  오른쪽 끝 ≈ 391   (2026-10-02 새 가변저항, 손으로 잰 값)
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
const int POT_LEFT = 543;       // 왼쪽 끝까지 돌렸을 때 ADC
const int POT_CENTER = 462;     // [1009b] 똑바로일 때 ADC. 2026-10-09 바퀴를 손으로 똑바로 맞추고 잰 값 (전엔 양 끝 평균 467)
const int POT_RIGHT = 391;      // 오른쪽 끝까지 돌렸을 때 ADC
const int END_MARGIN = 3;       // [1009] 왼쪽 끝에서 이만큼 덜 가서 멈춤 (기계 끝에 쾅 부딪히지 않게)
const int END_MARGIN_RIGHT = 3; // [1009] 오른쪽 끝 여유
const int LEFT_LIMIT = POT_LEFT - END_MARGIN;          // 왼쪽 최대 목표 = 540
const int RIGHT_LIMIT = POT_RIGHT + END_MARGIN_RIGHT;  // 오른쪽 최대 목표 = 394
const int ADC_VALID_MARGIN = 40;  // 351~583 밖이면 센서 고장으로 봄 (FAULT adc_out_of_range)

// ---------- 조향 모터 제어 ----------
const int ADC_DEADBAND = 6;       // 목표 ± 6 안이면 "도착" → 조향 모터 멈춤 (떨림 방지)
const int MAX_STEERING_STEP = 7;  // 조향 단계 최대 (명령 s-7 ~ s7)
const int MAX_PWM = 255;          // 뒷바퀴 PWM 최대
const int STEERING_PWM = 180;     // 목표가 멀 때 조향 모터 힘
const int STEERING_PWM_NEAR = 95; // 목표가 가까울 때(NEAR_COUNTS 안) 조향 모터 힘 → 지나치지 않게 살살
const int NEAR_COUNTS = 30;       // "가깝다" 의 기준 (ADC 30칸)

// ---------- 고장 판정 ----------
const uint32_t REVERSE_GRACE_MS = 400;  // 방향을 바꾼 직후 0.4 초는 관성으로 밀리는 것 봐줌
const int REVERSE_COUNTS = 15;          // 명령 반대 방향으로 15칸 이상 가면 wrong_direction
const int SETTLE_COUNTS = 12;           // 목표 12칸 안에서 멈춰 있으면 "도착" 인정 (끝이 뻑뻑해도 OK)
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
int settledStep = 0;           // 그때의 조향 단계
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

// 조향 단계(-7~+7) → 목표 ADC 값
//   왼쪽(음수 단계): 가운데에서 왼쪽 최대까지를 7등분
//   오른쪽(양수 단계): 가운데에서 오른쪽 최대까지를 7등분
//   예) -7 → 540,  0 → 462,  +7 → 394
int targetAdc(int step) {
  if (step < 0) {
    return POT_CENTER + ((LEFT_LIMIT - POT_CENTER) * (-step) + 3) / MAX_STEERING_STEP;
  }
  return POT_CENTER - ((POT_CENTER - RIGHT_LIMIT) * step + 3) / MAX_STEERING_STEP;
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

// 완성된 한 줄 명령 해석: "s3l240r240" → steering=3, leftPwm=240, rightPwm=240
void processData(const char *data) {
  const char *p = data;
  int nextSteering, nextLeft, nextRight;
  if (*p++ != 's' || !boundedInt(p, -7, 7, nextSteering) ||
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
  Serial.println(F("DRIVING_TEAM_CAL_1009B IDLE L=543 C=462 R=391"));   // 노트북이 이 글자로 펌웨어 확인
}

// 계속 반복
void loop() {
  // (1) USB 로 들어온 글자 처리
  while (Serial.available() > 0) receiveByte(static_cast<char>(Serial.read()));
  uint32_t now = millis();

  // (2) 명령이 없거나, 고장이거나, 0.3 초 넘게 명령이 끊기면 → 정지. 0.5 초마다 ADC 만 알려줌
  if (!active || faulted || static_cast<uint32_t>(now - lastRx) > RX_TIMEOUT_MS) {
    stopAll();
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
    latchFault(F("adc_out_of_range"), adc);    // 센서 값이 말이 안 됨 → 고장
    return;
  }

  // (5) 목표와의 차이 → 조향 모터를 어느 쪽으로 돌릴지
  int error = targetAdc(steering) - adc;       // + 면 왼쪽으로 더 가야 함
  int direction = 0;                           // +1: 왼쪽(ADC 증가), -1: 오른쪽(ADC 감소), 0: 멈춤
  if (error > ADC_DEADBAND && adc < LEFT_LIMIT) direction = 1;
  if (error < -ADC_DEADBAND && adc > RIGHT_LIMIT) direction = -1;
  if (settled && (steering != settledStep || abs(error) > NEAR_COUNTS)) settled = false;  // 새 목표면 도착 해제
  if (settled) direction = 0;                  // 이미 도착 인정이면 그대로 둠

  // (6) 0.2 초마다 노트북에 상태 보고 (board_monitor.py 가 기록)
  if (static_cast<uint32_t>(now - lastReport) >= 200) {
    lastReport = now;
    Serial.print(F("ADC=")); Serial.print(adc);
    Serial.print(F(" T=")); Serial.print(targetAdc(steering));
    Serial.print(F(" S=")); Serial.print(steering);
    Serial.print(F(" D=")); Serial.println(direction);
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
      settledStep = steering;
      watchedDirection = 0;
      direction = 0;
      Serial.print(F("SETTLED ADC=")); Serial.print(adc); Serial.print(F(" T=")); Serial.println(targetAdc(steering));
    } else if (static_cast<uint32_t>(now - progressSince) > STEERING_STALL_MS) {
      latchFault(F("stall"), adc);               // 멀리서 2초 동안 못 움직임 → 고장
      return;
    }
  }

  // (8) 실제로 모터 돌리기
  motorOff(STEERING_1, STEERING_2);
  int steerPwm = (abs(error) <= NEAR_COUNTS) ? STEERING_PWM_NEAR : STEERING_PWM;   // 가까우면 살살
  if (direction > 0) analogWrite(STEERING_2, steerPwm);   // 왼쪽으로
  if (direction < 0) analogWrite(STEERING_1, steerPwm);   // 오른쪽으로
  writeRear(LEFT_1, LEFT_2, leftPwm);                      // 뒷바퀴는 받은 PWM 그대로
  writeRear(RIGHT_1, RIGHT_2, rightPwm);
}
```


---

## `arduino/driving_team_cal_1009c/driving_team_cal_1009c.ino`

```cpp
// =====================================================================================
//  자캡디 주행 펌웨어  driving_team_cal_1009c  (Arduino Mega 2560)
//  "노트북이 USB로 보낸 글자 명령 → 조향 모터 + 뒷바퀴 모터 2개를 움직인다"
// =====================================================================================
//
//  ■ 명령 형식 (노트북 → 아두이노, 115200 baud, 한 줄씩)
//      s<조향 단계>l<왼쪽 뒷바퀴 PWM>r<오른쪽 뒷바퀴 PWM>\n
//      예) s3l240r240   : 오른쪽으로 3단계, 뒷바퀴 둘 다 240
//          s-7l200r200  : 왼쪽 끝까지(7단계), 뒷바퀴 200
//          s0l0r0       : 모든 모터 끄기 (정지)
//      조향 단계: -7(왼쪽 끝) ~ 0(가운데) ~ +7(오른쪽 끝)
//      PWM: -255 ~ 255 (모터 힘. 0 = 정지, 255 = 최대, 음수 = 뒤로)
//
//  ■ 조향은 어떻게 하나? (피드백 제어)
//      앞바퀴 축에 가변저항(포텐셔미터)이 붙어 있어서, 바퀴가 돌면 A2 핀 전압이 변한다.
//      아두이노는 그 전압을 0~1023 숫자(ADC 값)로 읽는다.
//        왼쪽 끝 ≈ 543,  가운데 ≈ 462 (1009b 실측),  오른쪽 끝 ≈ 391   (2026-10-02 새 가변저항, 손으로 잰 값)
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
//  ■ 1009c: 조향 모터 힘 STEERING_PWM 180 → 255 (더 빨리 꺾음). 가까울 때 힘(95)은 그대로
//  ■ 1009b: 가운데 POT_CENTER 467 → 462 (실측). 나머지는 1009 와 같음
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
const int POT_LEFT = 543;       // 왼쪽 끝까지 돌렸을 때 ADC
const int POT_CENTER = 462;     // [1009b] 똑바로일 때 ADC. 2026-10-09 바퀴를 손으로 똑바로 맞추고 잰 값 (전엔 양 끝 평균 467)
const int POT_RIGHT = 391;      // 오른쪽 끝까지 돌렸을 때 ADC
const int END_MARGIN = 3;       // [1009] 왼쪽 끝에서 이만큼 덜 가서 멈춤 (기계 끝에 쾅 부딪히지 않게)
const int END_MARGIN_RIGHT = 3; // [1009] 오른쪽 끝 여유
const int LEFT_LIMIT = POT_LEFT - END_MARGIN;          // 왼쪽 최대 목표 = 540
const int RIGHT_LIMIT = POT_RIGHT + END_MARGIN_RIGHT;  // 오른쪽 최대 목표 = 394
const int ADC_VALID_MARGIN = 40;  // 351~583 밖이면 센서 고장으로 봄 (FAULT adc_out_of_range)

// ---------- 조향 모터 제어 ----------
const int ADC_DEADBAND = 6;       // 목표 ± 6 안이면 "도착" → 조향 모터 멈춤 (떨림 방지)
const int MAX_STEERING_STEP = 7;  // 조향 단계 최대 (명령 s-7 ~ s7)
const int MAX_PWM = 255;          // 뒷바퀴 PWM 최대
const int STEERING_PWM = 255;     // [1009c] 목표가 멀 때 조향 모터 힘 (180 → 255 최대: 더 빨리 꺾음)
const int STEERING_PWM_NEAR = 95; // 목표가 가까울 때(NEAR_COUNTS 안) 조향 모터 힘 → 지나치지 않게 살살
const int NEAR_COUNTS = 30;       // "가깝다" 의 기준 (ADC 30칸)

// ---------- 고장 판정 ----------
const uint32_t REVERSE_GRACE_MS = 400;  // 방향을 바꾼 직후 0.4 초는 관성으로 밀리는 것 봐줌
const int REVERSE_COUNTS = 15;          // 명령 반대 방향으로 15칸 이상 가면 wrong_direction
const int SETTLE_COUNTS = 12;           // 목표 12칸 안에서 멈춰 있으면 "도착" 인정 (끝이 뻑뻑해도 OK)
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
int settledStep = 0;           // 그때의 조향 단계
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

// 조향 단계(-7~+7) → 목표 ADC 값
//   왼쪽(음수 단계): 가운데에서 왼쪽 최대까지를 7등분
//   오른쪽(양수 단계): 가운데에서 오른쪽 최대까지를 7등분
//   예) -7 → 540,  0 → 462,  +7 → 394
int targetAdc(int step) {
  if (step < 0) {
    return POT_CENTER + ((LEFT_LIMIT - POT_CENTER) * (-step) + 3) / MAX_STEERING_STEP;
  }
  return POT_CENTER - ((POT_CENTER - RIGHT_LIMIT) * step + 3) / MAX_STEERING_STEP;
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

// 완성된 한 줄 명령 해석: "s3l240r240" → steering=3, leftPwm=240, rightPwm=240
void processData(const char *data) {
  const char *p = data;
  int nextSteering, nextLeft, nextRight;
  if (*p++ != 's' || !boundedInt(p, -7, 7, nextSteering) ||
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
  Serial.println(F("DRIVING_TEAM_CAL_1009C IDLE L=543 C=462 R=391"));   // 노트북이 이 글자로 펌웨어 확인
}

// 계속 반복
void loop() {
  // (1) USB 로 들어온 글자 처리
  while (Serial.available() > 0) receiveByte(static_cast<char>(Serial.read()));
  uint32_t now = millis();

  // (2) 명령이 없거나, 고장이거나, 0.3 초 넘게 명령이 끊기면 → 정지. 0.5 초마다 ADC 만 알려줌
  if (!active || faulted || static_cast<uint32_t>(now - lastRx) > RX_TIMEOUT_MS) {
    stopAll();
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
    latchFault(F("adc_out_of_range"), adc);    // 센서 값이 말이 안 됨 → 고장
    return;
  }

  // (5) 목표와의 차이 → 조향 모터를 어느 쪽으로 돌릴지
  int error = targetAdc(steering) - adc;       // + 면 왼쪽으로 더 가야 함
  int direction = 0;                           // +1: 왼쪽(ADC 증가), -1: 오른쪽(ADC 감소), 0: 멈춤
  if (error > ADC_DEADBAND && adc < LEFT_LIMIT) direction = 1;
  if (error < -ADC_DEADBAND && adc > RIGHT_LIMIT) direction = -1;
  if (settled && (steering != settledStep || abs(error) > NEAR_COUNTS)) settled = false;  // 새 목표면 도착 해제
  if (settled) direction = 0;                  // 이미 도착 인정이면 그대로 둠

  // (6) 0.2 초마다 노트북에 상태 보고 (board_monitor.py 가 기록)
  if (static_cast<uint32_t>(now - lastReport) >= 200) {
    lastReport = now;
    Serial.print(F("ADC=")); Serial.print(adc);
    Serial.print(F(" T=")); Serial.print(targetAdc(steering));
    Serial.print(F(" S=")); Serial.print(steering);
    Serial.print(F(" D=")); Serial.println(direction);
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
      settledStep = steering;
      watchedDirection = 0;
      direction = 0;
      Serial.print(F("SETTLED ADC=")); Serial.print(adc); Serial.print(F(" T=")); Serial.println(targetAdc(steering));
    } else if (static_cast<uint32_t>(now - progressSince) > STEERING_STALL_MS) {
      latchFault(F("stall"), adc);               // 멀리서 2초 동안 못 움직임 → 고장
      return;
    }
  }

  // (8) 실제로 모터 돌리기
  motorOff(STEERING_1, STEERING_2);
  int steerPwm = (abs(error) <= NEAR_COUNTS) ? STEERING_PWM_NEAR : STEERING_PWM;   // 가까우면 살살
  if (direction > 0) analogWrite(STEERING_2, steerPwm);   // 왼쪽으로
  if (direction < 0) analogWrite(STEERING_1, steerPwm);   // 오른쪽으로
  writeRear(LEFT_1, LEFT_2, leftPwm);                      // 뒷바퀴는 받은 PWM 그대로
  writeRear(RIGHT_1, RIGHT_2, rightPwm);
}
```


---

## `python/1_path/B_dashed_fallback.py`

```python
# -*- coding: utf-8 -*-
"""[변형 B 점선 대체, 2026-09-27 sim] 오른쪽 실선이 없어도 점선이 보이면 탐색하지 않고 점선 기준 중심으로 계속.

선 라벨(solid_line/dashed_line) YOLO 결과 -> lane 폴리곤 -> /lane_detections (실시간 ROS 연결부).

START_HERE_KO.md 2B 계약을 따른다.
  - /detections 의 mask.data 는 픽셀 좌표이므로 polygon_to_mask(정규화용)를 쓰지 않고 직접 fillPoly 한다.
  - 경계 추출은 팀 labeling-tools 원본 함수를 그대로 호출한다.
      width_model 파라미터가 있으면: lane_path.find_path (한쪽 선 보완 포함)
      없으면: lane_from_lines.scan_lane / interpolate_edges / lane_polygon (양쪽 선이 관측된 구간만)
  - 결과 lane 폴리곤을 class_name='lane' 픽셀 Detection 으로 /lane_detections 에 발행한다.
    미터 변환(IPM)과 LaneTrajectory 발행은 팀 lane_info_extractor_node 가
    sub_detection_topic:=/lane_detections 로 재사용한다.
  - 실패/영상 지연(max_image_age_sec 초과)/원본 영상 없음은 빈 DetectionArray 를 발행해
    lane_info_extractor 가 valid=false 를 내게 한다. 마지막 정상 경로를 재발행하지 않는다.
  - header 는 원본 프레임 header 를 유지한다.

관대 모드 (사용자 요청, 2026-09-24):
  - online_width:=true  양쪽 선이 보인 행에서 차로 폭 표본을 모아 lane_path.fit_width 로 폭 모델을 온라인 학습.
                         양쪽 스캔이 실패하면 그 모델로 lane_path.find_path (한쪽 선 보완)를 시도.
  - hold_last_sec:=T    둘 다 실패하면 마지막 정상 lane 폴리곤을 최대 T초 동안 새 header 로 재발행.
                         T초가 지나면 빈 결과(= valid=false, Stanley 정지).
  - 엄격 모드 원본: line_path_node.strict.py

solid_follow 모드 (mode:=solid_follow, 2026-09-24 예선용):
  - 2차선 고정: 오른쪽 solid_line 안쪽 가장자리 = 차로 오른쪽 경계. dashed_line 은 solid 왼쪽에 있을 때만 왼쪽 경계.
  - 왼쪽 경계가 없거나 폭이 맞지 않으면 IPM(ipm_yaml)으로 solid 에서 lane_width_m(0.85m) 왼쪽 점을 계산해 경계로 쓴다.
  - solid 가 안 보이면 dashed 오른쪽 lane_width_m 을 차로로 본다(여전히 2차선).
  - 차 바로 앞(이미지 아래쪽 near_top_row 아래) 행만 사용해 먼 곳 커브에 미리 조향하지 않게 한다.
  - 관대 모드 원본: line_path_node.lenient.py
  v2 (우회전 대응): 경로를 이미지 폴리곤이 아니라 지면 좌표(m)로 직접 계산해 /lane_trajectory 발행.
    solid 안쪽 끝 + half_width, dashed 안쪽 끝 - half_width 로 중심점을 만들고 (둘 다 있으면 평균),
    solid 가 시야에서 사라지면 dashed 만으로 계속 따라간다. 화면 밖 차로도 잘리지 않는다.
    이 모드에서는 lane_info_extractor 를 함께 띄우지 않는다(중복 발행 방지). v1: line_path_node.solid_v1.py
"""
import collections
import time
import json
import os
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from interfaces_pkg.msg import DetectionArray, Detection, Point2D, LaneTrajectory

PACK = os.environ.get("JACAPDI_LIVE_PACK", os.path.expanduser("~/jacapdi-live-car-2026-09-24"))
sys.path.insert(0, os.path.join(PACK, "labeling-tools"))
from lane_from_lines import scan_lane, interpolate_edges, lane_polygon, runs_in_row  # noqa: E402
from lane_path import find_path, fit_width  # noqa: E402


class LinePathNode(Node):
    def __init__(self):
        super().__init__("line_path_node")
        p = self.declare_parameter
        self.det_topic = p("sub_detection_topic", "/detections").value
        self.img_topic = p("image_topic", "/image_raw").value
        self.pub_topic = p("pub_topic", "/lane_detections").value
        self.solid = p("solid_class", "solid_line").value
        self.dashed = p("dashed_class", "dashed_line").value
        self.width_model_path = p("width_model", "").value
        self.lane_side = p("lane", "right").value          # lane_path: 점선 기준 우리 차로 방향
        self.min_width = p("min_width", 0.10).value        # lane_from_lines 기본값
        self.max_width = p("max_width", 0.90).value
        self.max_gap = p("max_gap", 0.25).value
        self.min_rows = p("min_rows", 0.15).value
        self.max_image_age = p("max_image_age_sec", 0.3).value
        self.online_width = p("online_width", False).value
        self.hold_last_sec = p("hold_last_sec", 0.0).value
        self.mode = p("mode", "lines").value
        self.lane_width_m = p("lane_width_m", 0.85).value
        self.near_top_row = p("near_top_row", -1).value      # 이 행보다 아래만 사용 (-1: IPM 사다리꼴 윗변)
        self.ipm_yaml = p("ipm_yaml", os.path.expanduser("~/jacapdi-docker/ipm_c920.yaml")).value
        self.M = self.Minv = None
        if self.mode == "solid_follow":
            import yaml
            ip = yaml.safe_load(open(self.ipm_yaml))["lane_info_extractor_node"]["ros__parameters"]
            self.ipm = ip
            self.get_logger().info("solid_follow mode (lane 2, dashed left / solid right), ipm=%s" % self.ipm_yaml)
        self.width_samples = collections.deque(maxlen=4000)
        self.online_model, self.last_fit = None, 0.0
        self.last_poly, self.last_poly_time = None, 0.0

        self.width_model = None
        if self.width_model_path:
            models = json.load(open(self.width_model_path, encoding="utf-8"))
            self.width_model = models.get("_global") if "_global" in models else models
            self.get_logger().info("find_path mode, width model: %s" % self.width_model_path)
        else:
            self.get_logger().info("both-sides mode (no width model): lane_from_lines scan_lane")

        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self.bridge = CvBridge()
        self.images = {}  # stamp_ns -> bgr (최근 몇 장)
        self.create_subscription(Image, self.img_topic, self.on_image, 10)
        self.create_subscription(DetectionArray, self.det_topic, self.on_detections, qos)
        self.pub = self.create_publisher(DetectionArray, self.pub_topic, qos)
        self.near_range_m = p("near_range_m", -1.0).value    # 기본: IPM 사다리꼴 범위 (x_forward 기준)
        self.ext_range_m = p("ext_range_m", 1.4).value       # 점 부족 시 확장 범위
        self.traj_pub = self.create_publisher(LaneTrajectory, p("pub_trajectory_topic", "/lane_trajectory").value, qos) \
            if self.mode == "solid_follow" else None
        self.last_traj = None
        self.search_offset_m = p("search_offset_m", 0.15).value   # Stanley 기본 게인에서 약 +4단계
        self.search_max_sec = p("search_max_sec", 2.0).value
        self.search_start = None
        self.stats = {"frames": 0, "ok": 0, "both": 0, "one_side": 0, "held": 0, "search": 0}
        self.last_reason = ""
        self.create_timer(2.0, self.log_stats)

    def now_s(self):
        """[sim 수정 2026-09-27] 노드 시계. use_sim_time 이면 시뮬 시간이라 배속 시뮬에서도 search/hold 시간이 같다.
        실차(use_sim_time false)에서는 원래 time.time() 과 같다. Node 초기화 없이 쓰면(probe) 벽시계."""
        try:
            return self.get_clock().now().nanoseconds * 1e-9
        except Exception:
            return time.time()

    @staticmethod
    def stamp_ns(header):
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    def on_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.images[self.stamp_ns(msg.header)] = img
        if len(self.images) > 10:
            for k in sorted(self.images)[:-10]:
                del self.images[k]

    def publish(self, header, poly):
        out = DetectionArray()
        out.header = header
        if poly is not None:
            d = Detection()
            d.class_name = "lane"
            d.score = 1.0
            d.mask.height, d.mask.width = self.shape
            for x, y in poly:
                pt = Point2D(); pt.x = float(x); pt.y = float(y)
                d.mask.data.append(pt)
            out.detections.append(d)
        self.pub.publish(out)

    def on_detections(self, msg):
        self.stats["frames"] += 1
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        img = self.images.get(self.stamp_ns(msg.header))
        shape = None
        for d in msg.detections:
            if d.mask.height > 0 and d.mask.width > 0:
                shape = (d.mask.height, d.mask.width); break
        if shape is None and img is not None:
            shape = img.shape[:2]
        if age > self.max_image_age or shape is None:
            self.last_reason = "stale(%.2fs)" % age if age > self.max_image_age else "no_mask"
            self.fail(msg.header); return
        self.shape = shape
        h, w = shape
        md = np.zeros((h, w), np.uint8); ms = np.zeros((h, w), np.uint8)
        for d in msg.detections:
            if not d.mask.data or d.class_name not in (self.solid, self.dashed):
                continue
            pts = np.array([[int(round(p.x)), int(round(p.y))] for p in d.mask.data], np.int32)
            pts[:, 0] = np.clip(pts[:, 0], 0, w - 1); pts[:, 1] = np.clip(pts[:, 1], 0, h - 1)
            cv2.fillPoly(md if d.class_name == self.dashed else ms, [pts], 255)
        if not (md.any() or ms.any()):
            self.last_reason = "no_line"
            self.fail(msg.header); return

        poly = None
        if self.mode == "solid_follow":
            traj, poly, reason = self.metric_follow(md, ms, h, w, msg.header)
            self.last_reason = reason
            if traj is None:
                self.fail(msg.header); return
            if not reason.startswith("search"):
                self.stats["both" if reason.startswith("solid+dashed") else "one_side"] += 1
            self.stats["ok"] += 1
            self.last_traj, self.last_poly, self.last_poly_time = traj, poly, self.now_s()
            self.traj_pub.publish(traj); self.publish(msg.header, poly)
            return
        elif self.width_model is not None:
            if img is None:
                self.last_reason = "no_image_for_frame"
                self.publish(msg.header, None); return
            args = SimpleNamespace(lane=self.lane_side, max_gap=self.max_gap, min_rows=self.min_rows)
            res = find_path(img, md, ms, self.width_model, args)
            poly = res["lane_poly"]
            self.last_reason = res["status"] + ("," + ",".join(res["flags"]) if res["flags"] else "")
        else:
            mask = cv2.bitwise_or(md, ms)
            L, R, stat = scan_lane(mask, int(self.min_width * w), int(self.max_width * w), int(self.max_gap * h))
            if self.online_width:
                both = np.flatnonzero(~np.isnan(L) & ~np.isnan(R))
                self.width_samples.extend((float(y), float(R[y] - L[y])) for y in both[::2])
                if self.now_s() - self.last_fit > 1.0 and len(self.width_samples) >= 50:
                    self.last_fit = self.now_s()
                    m = fit_width(list(self.width_samples), h)
                    if m is not None: self.online_model = m
            L, R = interpolate_edges(L, R, int(self.max_gap * h))
            res = lane_polygon(L, R, min_rows=int(self.min_rows * h))
            if res is not None:
                poly = res[0]; self.last_reason = "ok"; self.stats["both"] += 1
            elif self.online_model is not None and img is not None:
                args = SimpleNamespace(lane=self.lane_side, max_gap=self.max_gap, min_rows=self.min_rows,
                                       min_width=self.min_width, max_width=self.max_width)
                r2 = find_path(img, md, ms, self.online_model, args)
                if r2["lane_poly"] and r2["status"] != "too_short":
                    poly = r2["lane_poly"]; self.last_reason = "find_path:" + r2["status"]; self.stats["one_side"] += 1
                else:
                    self.last_reason = "find_path_failed:" + r2["status"]
            else:
                self.last_reason = "one_side_only" if (stat["rows_left_only"] + stat["rows_right_only"]) > stat["rows_both"] else "too_short"
        if poly is None:
            self.fail(msg.header); return
        self.stats["ok"] += 1
        self.last_poly, self.last_poly_time = poly, self.now_s()
        self.publish(msg.header, poly)

    def _homography(self, h, w):
        if self.M is None or self.M_shape != (h, w):
            ip = self.ipm
            src = np.float32(ip["ipm_src_points"]).reshape(4, 2)
            src[:, 0] *= w / float(ip["ipm_src_base_width"]); src[:, 1] *= h / float(ip["ipm_src_base_height"])
            bev_h = int(round(h * float(ip["bev_height_scale"])))
            dst = np.float32([[w * ip["ipm_dst_left_ratio"], 0], [w * ip["ipm_dst_right_ratio"], 0],
                              [w * ip["ipm_dst_right_ratio"], bev_h - 1], [w * ip["ipm_dst_left_ratio"], bev_h - 1]])
            self.M = cv2.getPerspectiveTransform(src, dst); self.Minv = np.linalg.inv(self.M)
            self.M_shape = (h, w); self.top_row = int(src[:, 1].min())
            self.lat_px = self.lane_width_m / float(ip["lateral_meter_per_pixel"])  # 차로폭 BEV 픽셀
        return self.M

    def _shift_in_image(self, x, y, bev_dx):
        """이미지 점 (x, y) 를 BEV 에서 가로로 bev_dx 픽셀 옮긴 점의 이미지 x (같은 행 근사)."""
        b = cv2.perspectiveTransform(np.float32([[[x, y]]]), self.M)[0, 0]
        q = cv2.perspectiveTransform(np.float32([[[b[0] + bev_dx, b[1]]]]), self.Minv)[0, 0]
        return float(q[0])

    def solid_follow(self, md, ms, h, w):
        self._homography(h, w)
        top = self.near_top_row if self.near_top_row >= 0 else self.top_row
        L = np.full(h, np.nan); R = np.full(h, np.nan); kind = np.zeros(h, np.int8)
        prev_r = None; prev_d = None
        for y in range(h - 1, max(top, 0) - 1, -1):
            sr = runs_in_row(ms[y]); dr = runs_in_row(md[y])
            # 오른쪽 solid: 처음엔 화면 가운데보다 오른쪽 조각 중 가장 왼쪽, 이후엔 직전 행과 연속된 조각
            r = None
            if sr:
                if prev_r is None:
                    c = [s_ for s_ in sr if s_[0] > w * 0.4]
                    r = min(c, key=lambda s_: s_[0])[0] if c else None
                else:
                    s_ = min(sr, key=lambda s_: abs(s_[0] - prev_r))
                    r = s_[0] if abs(s_[0] - prev_r) < 40 else None
            # 왼쪽 dashed: solid 보다 왼쪽(2차선 = dashed 왼쪽)
            d = None
            if dr:
                ref = r if r is not None else (prev_r if prev_r is not None else w)
                c = [d_ for d_ in dr if d_[1] < ref - 10]
                if c:
                    d = max(c, key=lambda d_: d_[1])[1]
                    if prev_d is not None and abs(d - prev_d) > 60: d = None
            if r is not None:
                exp_l = self._shift_in_image(r, y, -self.lat_px)
                if d is not None and abs((r - d) - (r - exp_l)) < 0.35 * (r - exp_l):
                    L[y], R[y], kind[y] = d, r, 1
                else:
                    L[y], R[y], kind[y] = exp_l, r, 2
                prev_r = r
                if d is not None: prev_d = d
            elif d is not None:
                L[y], R[y], kind[y] = d, self._shift_in_image(d, y, self.lat_px), 3
                prev_d = d
        have = np.flatnonzero(kind > 0)
        if have.size < max(10, int(self.min_rows * h)):
            return None, "too_few_rows(%d)" % have.size
        # 아래에서부터 끊김(max_gap) 없는 구간만
        seg = [int(have[-1])]
        for y in have[::-1][1:]:
            if seg[-1] - y <= int(self.max_gap * h): seg.append(int(y))
            else: break
        idx = np.array(sorted(seg)); ys = np.arange(idx[0], idx[-1] + 1)
        Lf = np.interp(ys, idx, L[idx]); Rf = np.interp(ys, idx, R[idx])
        pts = np.array([(x, y) for x, y in zip(Lf[::-1], ys[::-1])] + [(x, y) for x, y in zip(Rf, ys)], np.float32)
        approx = cv2.approxPolyDP(pts.reshape(-1, 1, 2), 2.0, True).reshape(-1, 2)
        k = collections.Counter(int(v) for v in kind[idx])
        reason = "solid+dashed" if k.get(1, 0) >= len(idx) * 0.5 else ("solid_only" if k.get(2, 0) >= k.get(3, 0) else "dashed_only")
        return [(float(x), float(y)) for x, y in approx], reason

    def metric_follow(self, md, ms, h, w, header):
        """행마다 solid/dashed 안쪽 끝을 지면(m)으로 투영해 차로 중심점을 만들고 2차 곡선으로 적합."""
        M = self._homography(h, w)
        ip = self.ipm; lat, lon = float(ip["lateral_meter_per_pixel"]), float(ip["longitudinal_meter_per_pixel"])
        bev_h = int(round(h * float(ip["bev_height_scale"]))); ego = (w - 1) * 0.5
        half = self.lane_width_m / 2.0
        near = self.near_range_m if self.near_range_m > 0 else (bev_h - 1) * lon
        def to_m(xs, y):
            pts = cv2.perspectiveTransform(np.float32([[x, y] for x in xs]).reshape(-1, 1, 2), M).reshape(-1, 2)
            return (bev_h - 1 - pts[:, 1]) * lon, (ego - pts[:, 0]) * lat
        pts, n_s, n_d = [], 0, 0
        prev_r = prev_d = None; last_xf = -1e9
        for y in range(h - 1, -1, -2):
            xf0, _ = to_m([ego], y)
            xf0 = float(xf0[0])
            if xf0 < last_xf - 1e-6 or xf0 > self.ext_range_m: break   # 지평선 근처/범위 밖
            last_xf = xf0
            if xf0 < -0.3: continue
            sr = runs_in_row(ms[y]); dr = runs_in_row(md[y])
            yr = yd = None
            if sr:
                _, ys = to_m([r[0] for r in sr], y)        # solid 안쪽(왼쪽) 끝
                if prev_r is None: c = [v for v in ys if -1.2 < v < 0.2]; yr = min(c, key=lambda v: abs(v + half)) if c else None
                else:
                    v = min(ys, key=lambda v: abs(v - prev_r)); yr = v if abs(v - prev_r) < 0.15 else None
            if dr:
                _, ys = to_m([d[1] for d in dr], y)        # dashed 안쪽(오른쪽) 끝
                if yr is not None and any(v < yr - 0.2 for v in ys):
                    yr = None                              # solid 가 점선보다 왼쪽 = 1차선 바깥 실선 -> 버림
                if yr is not None: ys = [v for v in ys if v > yr + 0.4]  # 2차선: dashed 는 solid 왼쪽
                if prev_d is None: c = [v for v in ys if -0.6 < v < 1.2]; yd = min(c, key=lambda v: abs(v - half)) if c else None
                elif len(ys):
                    v = min(ys, key=lambda v: abs(v - prev_d)); yd = v if abs(v - prev_d) < 0.2 else None
            if yr is not None and yd is not None and abs((yd - yr) - self.lane_width_m) < 0.3:
                yc = 0.5 * ((yr + half) + (yd - half)); n_s += 1; n_d += 1
            elif yr is not None:
                yc = yr + half; n_s += 1
            elif yd is not None:
                yc = yd - half; n_d += 1
            else:
                yc = None
            if yr is not None: prev_r = yr
            if yd is not None: prev_d = yd
            if yc is not None: pts.append((xf0, yc))
        if n_s == 0 and n_d == 0:
            # [B] 실선도 점선도 안 보일 때만 오른쪽 탐색. 실선만 없으면 점선 - half 로 계속 (원본은 여기서 탐색 -> 2 s 뒤 정지)
            now = self.now_s()
            if self.search_start is None: self.search_start = now
            if now - self.search_start > self.search_max_sec:
                return None, None, "search_timeout(%.1fs)" % (now - self.search_start)
            self.stats["search"] += 1
            t = LaneTrajectory(); t.header = header; t.valid = True
            t.a = t.b = 0.0; t.c = -float(self.search_offset_m)
            t.x_min_m, t.x_max_m, t.fit_error_m, t.num_points = 0.0, 0.5, 0.0, 0
            return t, None, "search_right(%.1fs)" % (now - self.search_start)
        self.search_start = None
        if len(pts) < 8:
            return None, None, "too_few_points(%d)" % len(pts)
        P = np.array(pts)
        sel = P[P[:, 0] <= near]
        used = "near"
        if len(sel) < 15 or np.ptp(sel[:, 0]) < 0.15:
            sel = P; used = "ext"
        if len(sel) < 8 or np.ptp(sel[:, 0]) < 0.08:
            return None, None, "short_span"
        deg = 2 if np.ptp(sel[:, 0]) >= 0.3 else 1
        c = np.polyfit(sel[:, 0], sel[:, 1], deg)
        if deg == 1: c = np.array([0.0, c[0], c[1]])
        err = float(np.sqrt(np.mean((np.polyval(c, sel[:, 0]) - sel[:, 1]) ** 2)))
        if not np.all(np.isfinite(c)) or err > 0.15:
            return None, None, "bad_fit(%.2f)" % err
        t = LaneTrajectory(); t.header = header; t.valid = True
        t.a, t.b, t.c = float(c[0]), float(c[1]), float(c[2])
        t.x_min_m, t.x_max_m = float(sel[:, 0].min()), float(sel[:, 0].max())
        t.fit_error_m, t.num_points = err, int(len(sel))
        # 뷰어용: 중심 +- half 띠를 이미지로 역투영한 폴리곤
        xs = np.linspace(t.x_min_m, t.x_max_m, 12); yc = np.polyval(c, xs)
        band = [(ego - (y + half) / lat, bev_h - 1 - x / lon) for x, y in zip(xs, yc)] + \
               [(ego - (y - half) / lat, bev_h - 1 - x / lon) for x, y in zip(xs[::-1], yc[::-1])]
        poly = cv2.perspectiveTransform(np.float32(band).reshape(-1, 1, 2), self.Minv).reshape(-1, 2)
        kind = "solid+dashed" if n_s and n_d else ("solid_only" if n_s else "dashed_only")
        return t, [(float(x), float(y)) for x, y in poly], "%s,%s" % (kind, used)

    def fail(self, header):
        """실패 시: hold_last_sec 이내면 마지막 결과 재발행, 아니면 빈 결과/invalid(정지)."""
        held = self.last_poly is not None and self.now_s() - self.last_poly_time <= self.hold_last_sec
        if held: self.stats["held"] += 1
        self.publish(header, self.last_poly if held else None)
        if self.traj_pub is not None:
            if held and self.last_traj is not None:
                t = self.last_traj; t.header = header
            else:
                t = LaneTrajectory(); t.header = header; t.valid = False
            self.traj_pub.publish(t)

    def log_stats(self):
        s = self.stats
        wm = "none" if self.online_model is None else "a=%.3f b=%.1f n=%d" % (self.online_model["a"], self.online_model["b"], self.online_model["n"])
        self.get_logger().info("frames=%d ok=%d (both %d, one_side %d, search %d) held=%d last=%s width_model=%s"
                               % (s["frames"], s["ok"], s["both"], s["one_side"], s.get("search", 0), s["held"], self.last_reason, wm))


def main():
    rclpy.init()
    n = LinePathNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
```


---

## `python/1_path/E_curve_cross.py`

```python
# -*- coding: utf-8 -*-
"""
=====================================================================================
 경로 알고리즘 E  (2026-10-09)  -  "YOLO 가 찾은 선 → 차가 따라갈 경로(곡선)"
=====================================================================================

이 프로그램은 ROS 2 노드 하나입니다. 하는 일은 딱 하나:
   카메라 사진에서 YOLO 가 찾아 준 "실선(solid_line)" 과 "점선(dashed_line)" 을 보고,
   우리 차로(2차선)의 한가운데를 지나는 곡선  y = a·x² + b·x + c  를 계산해서 보낸다.

   ┌──────────────┐   /detections    ┌───────────────┐  /lane_trajectory  ┌────────────┐
   │ yolov8_node  │ ───────────────▶ │  이 노드 (E)   │ ─────────────────▶ │  제어기     │ → 조향/속도
   └──────────────┘ (선 모양 + 확신도) └───────────────┘   (경로 곡선 a,b,c) └────────────┘

좌표 약속 (중요!)
   x = 차 앞쪽 거리 (미터).  0 = 카메라 보정 구간의 가장 가까운 줄(차 앞 약 0.84 m)
   y = 차 왼쪽 거리 (미터).  + 는 왼쪽, - 는 오른쪽
   그래서 c > 0 이면 "차로 한가운데가 차보다 왼쪽에 있다" = 왼쪽으로 꺾어야 함.

트랙 규칙 (예선): 항상 2차선을 달린다.
   왼쪽 경계 = 점선(dashed_line),  오른쪽 경계 = 실선(solid_line),  차로 폭 0.85 m
   → 차로 한가운데 = 실선에서 0.425 m 왼쪽 = 점선에서 0.425 m 오른쪽

B 와 달라진 점 (나머지는 B 와 같음)
   [E1] 가로로 누운 실선은 버린다.
        횡단보도/정지선 앞의 굵은 가로선이 solid_line 으로 잡히면, B 는 그것을 "오른쪽 실선" 으로 착각해서
        경로를 순간적으로 오른쪽으로 꺾었다 (횡단보도에서 멈칫 + 오른쪽으로 가던 현상).
        E 는 실선 모양을 바닥 좌표로 바꿔 방향을 재고, 차 진행 방향과 60° 넘게 기울어 있으면 무시한다.
   [E2] 선 종류마다 다른 확신도 기준.
        점선은 흐려서 YOLO 확신도가 낮게 나오는 경우가 많다 → 0.30 이상이면 인정 (실선은 0.50 그대로).
        (YOLO 자체 threshold 는 0.3 으로 낮춰서 실행: CONF=0.3)
   [E3] 경로를 더 멀리까지 본다.
        B 는 차 앞 0.84~1.40 m 만 써서 커브를 늦게 알아챘다. E 는 기본 1.0 m 범위(약 0.84~1.84 m)까지 쓴다.
        → 커브가 시작되는 것을 더 일찍 보고 더 일찍 꺾는다.

실행 (start_ab.sh 가 대신 해 줌):  ALGO=E CONF=0.3 ~/jacapdi-docker/car_prep.sh
"""
import math
import os
import sys
import time

import cv2                      # 사진/도형 계산 라이브러리 (OpenCV)
import numpy as np              # 숫자 배열 계산 라이브러리
import rclpy                    # ROS 2 파이썬 라이브러리
import yaml                     # 설정 파일(.yaml) 읽기
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.time import Time
from interfaces_pkg.msg import DetectionArray, Detection, Point2D, LaneTrajectory   # 팀이 정의한 메시지 양식

# 팀 labeling-tools 의 함수 하나를 빌려 쓴다: 사진 한 줄(row)에서 선이 칠해진 구간들을 찾는 함수
sys.path.insert(0, os.path.join(os.path.expanduser("~/jacapdi-live-car-2026-09-24"), "labeling-tools"))
from lane_from_lines import runs_in_row   # noqa: E402   (예: [0,0,1,1,1,0,1,1] → [(2,4), (6,7)])


class LinePathNodeE(Node):
    # ---------------------------------------------------------------------------------
    # 1) 시작할 때 한 번: 설정값(파라미터) 읽기, 통로(토픽) 연결
    # ---------------------------------------------------------------------------------
    def __init__(self):
        super().__init__("line_path_node")          # 노드 이름 (다른 프로그램이 이 이름으로 우리를 확인함)
        p = self.declare_parameter                  # "실행할 때 바꿀 수 있는 값" 을 선언하는 함수 (줄여 쓰기)

        # --- 통로(토픽) 이름 ---
        self.det_topic = p("sub_detection_topic", "/detections").value       # YOLO 결과가 들어오는 통로
        self.traj_topic = p("pub_trajectory_topic", "/lane_trajectory").value # 우리가 경로를 내보내는 통로
        self.band_topic = p("pub_topic", "/lane_detections").value           # 화면(path_viewer)용 차로 띠

        # --- 트랙/선 관련 ---
        self.lane_width_m = p("lane_width_m", 0.85).value      # 트랙 차로 폭 (규격)
        self.solid_min_score = p("solid_min_score", 0.50).value   # [E2] 실선 최소 확신도
        self.dashed_min_score = p("dashed_min_score", 0.30).value # [E2] 점선 최소 확신도
        self.transverse_deg = p("transverse_deg", 60.0).value     # [E1] 이 각도보다 누운 실선은 버림

        # --- 얼마나 멀리 볼지 ---
        self.near_range_m = p("near_range_m", 1.0).value        # [E3] 평소 쓰는 거리 (x=0 부터)
        self.ext_range_m = p("ext_range_m", 1.4).value          # 점이 부족할 때 넓혀 볼 최대 거리

        # --- 안전 관련 ---
        self.max_image_age = p("max_image_age_sec", 0.3).value  # 사진이 이보다 오래됐으면 쓰지 않음
        self.hold_last_sec = p("hold_last_sec", 1.0).value      # 선을 잃으면 직전 경로를 이 시간까지만 유지
        self.search_offset_m = p("search_offset_m", 0.15).value # 선이 전혀 없으면 오른쪽으로 찾는 정도
        self.search_max_sec = p("search_max_sec", 2.0).value    # 그렇게 찾는 최대 시간 (넘으면 정지)

        # --- 카메라 보정값 (사진 픽셀 → 바닥 미터 변환에 필요) ---
        ipm_path = p("ipm_yaml", os.path.expanduser("~/jacapdi-docker/ipm_c920.yaml")).value
        self.ipm = yaml.safe_load(open(ipm_path))["lane_info_extractor_node"]["ros__parameters"]
        self.M = None                                            # 변환 행렬 (첫 사진 크기를 보고 만듦)

        # --- 통로 연결 ---
        # RELIABLE = 메시지를 잃어버리지 않도록 확인하며 보냄, depth=1 = 가장 최신 것 하나만 보관
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self.create_subscription(DetectionArray, self.det_topic, self.on_detections, qos)  # 받기
        self.traj_pub = self.create_publisher(LaneTrajectory, self.traj_topic, qos)       # 보내기 (경로)
        self.band_pub = self.create_publisher(DetectionArray, self.band_topic, qos)       # 보내기 (화면용)

        # --- 상태 기억용 변수 ---
        self.last_traj = None; self.last_band = None; self.last_ok_time = 0.0   # 직전 정상 경로
        self.search_start = None                                                 # 오른쪽 탐색 시작 시각
        self.stats = {"frames": 0, "ok": 0, "both": 0, "one_side": 0, "held": 0, "search": 0, "transverse_dropped": 0}
        self.last_reason = ""
        self.create_timer(2.0, self.log_stats)      # 2초마다 통계를 로그에 남김 (line_path.log)
        self.get_logger().info("algo E: transverse<%.0fdeg drop, solid>=%.2f dashed>=%.2f, near %.2fm"
                               % (self.transverse_deg, self.solid_min_score, self.dashed_min_score, self.near_range_m))

    # ---------------------------------------------------------------------------------
    # 2) 사진 픽셀 → 바닥 미터 변환 준비 (카메라 보정 = IPM)
    # ---------------------------------------------------------------------------------
    def homography(self, h, w):
        """사진 속 사다리꼴(바닥의 직사각형) 4점 ↔ 조감도 직사각형 4점 을 잇는 변환 행렬 M 을 만든다.
        M 으로 사진 점을 '하늘에서 내려다본 그림(BEV)' 의 점으로 옮기고, 거기서 픽셀 × (미터/픽셀) 로 미터를 얻는다."""
        if self.M is None or self.M_shape != (h, w):
            ip = self.ipm
            src = np.float32(ip["ipm_src_points"]).reshape(4, 2)            # 사진 속 4점 (640x480 기준)
            src[:, 0] *= w / float(ip["ipm_src_base_width"]); src[:, 1] *= h / float(ip["ipm_src_base_height"])
            self.bev_h = int(round(h * float(ip["bev_height_scale"])))      # 조감도 세로 픽셀 수
            dst = np.float32([[w * ip["ipm_dst_left_ratio"], 0], [w * ip["ipm_dst_right_ratio"], 0],
                              [w * ip["ipm_dst_right_ratio"], self.bev_h - 1], [w * ip["ipm_dst_left_ratio"], self.bev_h - 1]])
            self.M = cv2.getPerspectiveTransform(src, dst); self.Minv = np.linalg.inv(self.M)
            self.M_shape = (h, w)
            self.lat = float(ip["lateral_meter_per_pixel"])       # 조감도 가로 1픽셀 = 몇 미터
            self.lon = float(ip["longitudinal_meter_per_pixel"])  # 조감도 세로 1픽셀 = 몇 미터
            self.ego = (w - 1) * 0.5                              # 차 중심선 (조감도 가운데)
        return self.M

    def to_m(self, xs, ys):
        """사진 픽셀 좌표들 (xs[i], ys[i]) → 바닥 좌표 (x 앞 m, y 왼쪽 m) 배열 두 개."""
        pts = cv2.perspectiveTransform(np.float32(np.c_[xs, ys]).reshape(-1, 1, 2), self.M).reshape(-1, 2)
        return (self.bev_h - 1 - pts[:, 1]) * self.lon, (self.ego - pts[:, 0]) * self.lat

    # ---------------------------------------------------------------------------------
    # 3) [E1] 이 실선이 '가로로 누운 선' 인가?  (횡단보도/정지선 앞 굵은 선)
    # ---------------------------------------------------------------------------------
    def is_transverse(self, poly):
        """실선 테두리 점들을 바닥 좌표로 바꾼 뒤, 점들이 퍼진 주된 방향(주성분)을 구한다.
        차로 경계선이라면 대체로 차 앞쪽(x 방향)으로 길게 뻗어 있고,
        정지선/횡단보도 앞 선은 옆(y 방향)으로 길게 누워 있다.
        주된 방향이 x 축과 transverse_deg(기본 60°) 넘게 벌어지면 '가로선' 으로 판단."""
        xf, yl = self.to_m(poly[:, 0], poly[:, 1])
        ok = (xf > -0.5) & (xf < 4.0)                    # 지평선 근처(변환이 이상해지는 점)는 제외
        if ok.sum() < 4:
            return False                                 # 점이 너무 적으면 판단 보류 (버리지 않음)
        P = np.c_[xf[ok], yl[ok]]
        P = P - P.mean(axis=0)                           # 가운데로 옮기고
        _, _, vt = np.linalg.svd(P, full_matrices=False) # 가장 길게 퍼진 방향 = vt[0]
        dx, dy = vt[0]
        angle = math.degrees(math.atan2(abs(dy), abs(dx)))   # 0° = 앞쪽 방향, 90° = 옆 방향
        return angle > self.transverse_deg

    # ---------------------------------------------------------------------------------
    # 4) YOLO 결과가 올 때마다 (1초에 약 30번) 실행되는 함수
    # ---------------------------------------------------------------------------------
    def on_detections(self, msg):
        self.stats["frames"] += 1
        # (a) 사진이 너무 오래됐으면 쓰지 않는다 (지연된 정보로 운전하면 위험)
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        shape = next(((d.mask.height, d.mask.width) for d in msg.detections if d.mask.height > 0), None)
        if age > self.max_image_age or shape is None:
            self.last_reason = "stale(%.2fs)" % age if age > self.max_image_age else "no_mask"
            return self.fail(msg.header)
        h, w = shape
        self.homography(h, w)

        # (b) 실선 / 점선을 각각 흑백 그림(마스크)으로 칠한다. 칠할지 말지는 [E1][E2] 규칙으로 거른다.
        md = np.zeros((h, w), np.uint8)    # 점선 마스크 (255 = 점선이 있는 픽셀)
        ms = np.zeros((h, w), np.uint8)    # 실선 마스크
        for d in msg.detections:
            if not d.mask.data:
                continue
            is_solid = d.class_name == "solid_line"
            is_dashed = d.class_name == "dashed_line"
            if not (is_solid or is_dashed):
                continue                                                   # 다른 클래스(정지선 등)는 무시
            if is_solid and d.score < self.solid_min_score:  continue      # [E2] 실선 확신도 기준
            if is_dashed and d.score < self.dashed_min_score: continue     # [E2] 점선 확신도 기준 (더 관대)
            poly = np.array([[p.x, p.y] for p in d.mask.data], np.float32)
            if is_solid and self.is_transverse(poly):                      # [E1] 가로로 누운 실선 버림
                self.stats["transverse_dropped"] += 1
                continue
            pts = np.clip(np.round(poly), [0, 0], [w - 1, h - 1]).astype(np.int32)
            cv2.fillPoly(md if is_dashed else ms, [pts], 255)
        if not (md.any() or ms.any()):
            self.last_reason = "no_line"
            return self.fail(msg.header)

        # (c) 마스크로 경로 계산
        traj, band, reason = self.find_path(md, ms, h, w, msg.header)
        self.last_reason = reason
        if traj is None:
            return self.fail(msg.header)
        if not reason.startswith("search"):
            self.stats["both" if reason.startswith("solid+dashed") else "one_side"] += 1
        self.stats["ok"] += 1
        self.last_traj, self.last_band, self.last_ok_time = traj, band, time.time()
        self.traj_pub.publish(traj)
        self.publish_band(msg.header, band, shape)

    # ---------------------------------------------------------------------------------
    # 5) 경로 계산의 핵심: 아래(차 바로 앞)부터 위로 한 줄씩 훑으며 차로 한가운데 점을 모은다
    # ---------------------------------------------------------------------------------
    def find_path(self, md, ms, h, w, header):
        self.homography(h, w)                   # 픽셀→미터 변환 준비 (이미 했으면 그대로 씀)
        half = self.lane_width_m / 2.0          # 0.425 m
        pts = []                                # 모은 (x, y) 중심점들
        n_s = n_d = 0                           # 실선 / 점선을 본 줄 수
        prev_r = prev_d = None                  # 바로 아랫줄에서 찾은 실선 / 점선 위치 (연결성 확인용)
        last_xf = -1e9
        for row in range(h - 1, -1, -2):        # 사진 맨 아래(가까운 곳) → 위(먼 곳), 2줄씩
            xf0 = float(self.to_m([self.ego], [row])[0][0])      # 이 줄이 차 앞 몇 m 인가
            if xf0 < last_xf - 1e-6 or xf0 > self.ext_range_m:  # 지평선에 가까워지거나 너무 멀면 멈춤
                break
            last_xf = xf0
            if xf0 < -0.3:
                continue
            solid_runs = runs_in_row(ms[row])                   # 이 줄에서 실선이 칠해진 구간들
            dash_runs = runs_in_row(md[row])                    # 이 줄에서 점선이 칠해진 구간들
            yr = yd = None                                      # 이번 줄의 오른쪽 경계 / 왼쪽 경계 (m)

            # --- 오른쪽 경계 = 실선의 '안쪽(왼쪽) 끝' ---
            if solid_runs:
                _, ys = self.to_m([r[0] for r in solid_runs], [row] * len(solid_runs))
                if prev_r is None:   # 처음 찾을 때: 차 오른쪽(-1.2 ~ +0.2 m)에 있는 것 중 차로 폭 절반에 가장 가까운 것
                    cand = [v for v in ys if -1.2 < v < 0.2]
                    yr = min(cand, key=lambda v: abs(v + half)) if cand else None
                else:                # 이어서 찾을 때: 아랫줄 위치와 15 cm 안쪽으로 이어지는 것만 (튀는 것 방지)
                    v = min(ys, key=lambda v: abs(v - prev_r)); yr = v if abs(v - prev_r) < 0.15 else None

            # --- 왼쪽 경계 = 점선의 '안쪽(오른쪽) 끝' ---
            if dash_runs:
                _, ys = self.to_m([r[1] for r in dash_runs], [row] * len(dash_runs))
                if yr is not None and any(v < yr - 0.2 for v in ys):
                    yr = None        # 점선보다 왼쪽에 있는 실선 = 1차선 바깥 실선 → 우리 경계 아님
                if yr is not None:
                    ys = [v for v in ys if v > yr + 0.4]   # 2차선 규칙: 점선은 실선보다 0.4 m 이상 왼쪽
                if prev_d is None:
                    cand = [v for v in ys if -0.6 < v < 1.2]
                    yd = min(cand, key=lambda v: abs(v - half)) if cand else None
                elif len(ys):
                    v = min(ys, key=lambda v: abs(v - prev_d)); yd = v if abs(v - prev_d) < 0.2 else None

            # --- 이 줄의 차로 한가운데 ---
            if yr is not None and yd is not None and abs((yd - yr) - self.lane_width_m) < 0.3:
                yc = 0.5 * ((yr + half) + (yd - half)); n_s += 1; n_d += 1   # 둘 다 보이고 폭이 맞으면 평균
            elif yr is not None:
                yc = yr + half; n_s += 1                                      # 실선만: 실선 + 0.425 m
            elif yd is not None:
                yc = yd - half; n_d += 1                                      # 점선만: 점선 - 0.425 m
            else:
                yc = None
            if yr is not None: prev_r = yr
            if yd is not None: prev_d = yd
            if yc is not None:
                pts.append((xf0, yc))

        # --- 선이 하나도 없으면: 잠깐 오른쪽으로 찾아보고, 2초 넘으면 정지 ---
        if n_s == 0 and n_d == 0:
            now = time.time()
            if self.search_start is None: self.search_start = now
            if now - self.search_start > self.search_max_sec:
                return None, None, "search_timeout(%.1fs)" % (now - self.search_start)
            self.stats["search"] += 1
            t = LaneTrajectory(); t.header = header; t.valid = True
            t.a = t.b = 0.0; t.c = -float(self.search_offset_m)               # 중심이 오른쪽 15 cm 에 있다고 가정
            t.x_min_m, t.x_max_m, t.fit_error_m, t.num_points = 0.0, 0.5, 0.0, 0
            return t, None, "search_right(%.1fs)" % (now - self.search_start)
        self.search_start = None
        if len(pts) < 8:
            return None, None, "too_few_points(%d)" % len(pts)

        # --- 모은 점들에 곡선 y = a·x² + b·x + c 맞추기 ---
        P = np.array(pts)
        sel = P[P[:, 0] <= self.near_range_m]                 # [E3] 평소 범위 (기본 1.0 m)
        used = "near"
        if len(sel) < 15 or np.ptp(sel[:, 0]) < 0.15:         # 점이 부족하면 더 멀리까지
            sel = P; used = "ext"
        if len(sel) < 8 or np.ptp(sel[:, 0]) < 0.08:
            return None, None, "short_span"
        deg = 2 if np.ptp(sel[:, 0]) >= 0.3 else 1            # 점이 퍼진 길이가 짧으면 직선(1차)만
        c = np.polyfit(sel[:, 0], sel[:, 1], deg)
        if deg == 1: c = np.array([0.0, c[0], c[1]])
        err = float(np.sqrt(np.mean((np.polyval(c, sel[:, 0]) - sel[:, 1]) ** 2)))   # 곡선이 점과 평균 몇 m 어긋나나
        if not np.all(np.isfinite(c)) or err > 0.15:
            return None, None, "bad_fit(%.2f)" % err          # 15 cm 넘게 어긋나면 믿지 않음

        t = LaneTrajectory(); t.header = header; t.valid = True
        t.a, t.b, t.c = float(c[0]), float(c[1]), float(c[2])
        t.x_min_m, t.x_max_m = float(sel[:, 0].min()), float(sel[:, 0].max())
        t.fit_error_m, t.num_points = err, int(len(sel))
        # 화면용: 경로 ± 0.425 m 띠를 다시 사진 좌표로
        xs = np.linspace(t.x_min_m, t.x_max_m, 12); yc = np.polyval(c, xs)
        band = [(self.ego - (y + half) / self.lat, self.bev_h - 1 - x / self.lon) for x, y in zip(xs, yc)] + \
               [(self.ego - (y - half) / self.lat, self.bev_h - 1 - x / self.lon) for x, y in zip(xs[::-1], yc[::-1])]
        band = cv2.perspectiveTransform(np.float32(band).reshape(-1, 1, 2), self.Minv).reshape(-1, 2)
        kind = "solid+dashed" if n_s and n_d else ("solid_only" if n_s else "dashed_only")
        return t, [(float(x), float(y)) for x, y in band], "%s,%s" % (kind, used)

    # ---------------------------------------------------------------------------------
    # 6) 실패했을 때: 1초까지는 직전 경로를 계속 쓰고, 그 뒤엔 valid=false (제어기가 차를 세움)
    # ---------------------------------------------------------------------------------
    def fail(self, header):
        held = self.last_traj is not None and time.time() - self.last_ok_time <= self.hold_last_sec
        if held:
            self.stats["held"] += 1
            t = self.last_traj; t.header = header
        else:
            t = LaneTrajectory(); t.header = header; t.valid = False
        self.traj_pub.publish(t)
        self.publish_band(header, self.last_band if held else None, None)

    def publish_band(self, header, band, shape):
        out = DetectionArray(); out.header = header
        if band is not None:
            d = Detection(); d.class_name = "lane"; d.score = 1.0
            if shape: d.mask.height, d.mask.width = shape
            for x, y in band:
                pt = Point2D(); pt.x = float(x); pt.y = float(y); d.mask.data.append(pt)
            out.detections.append(d)
        self.band_pub.publish(out)

    def log_stats(self):
        s = self.stats
        self.get_logger().info("frames=%d ok=%d (both %d, one_side %d, search %d) held=%d transverse_dropped=%d last=%s"
                               % (s["frames"], s["ok"], s["both"], s["one_side"], s["search"], s["held"],
                                  s["transverse_dropped"], self.last_reason))


def main():
    rclpy.init()
    n = LinePathNodeE()
    try:
        rclpy.spin(n)          # 메시지가 올 때마다 on_detections 를 불러 주며 계속 실행
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
```


---

## `python/1_path/F_outlier_hold.py`

```python
# -*- coding: utf-8 -*-
"""
=====================================================================================
 경로 알고리즘 F  (2026-10-09)  =  E  +  "경로가 갑자기 튀면 무시하고 직전 경로 유지"
=====================================================================================

왜 필요한가?
   10/09 주행 횡단보도에서: 한 프레임 만에 경로가 c=+0.16 (왼쪽) → c=-0.15 (오른쪽) 로 바뀌었고
   조향 명령이 s-4 → s+4 로 뒤집혔다 (멈칫 + 오른쪽으로 가는 현상).
   횡단보도/정지선/주차구역 등 선이 갑자기 많이 보이거나 끊기는 곳에서 경로 계산이 잠깐 엉뚱해진다.

어떻게 하나? (이상치 = outlier)
   1) 새로 계산한 경로와 마지막으로 "믿은" 경로를 비교한다.
      차 앞 x = 0.0 / 0.4 / 0.8 m 세 지점에서 두 경로의 옆 위치(y) 차이 중 가장 큰 값을 본다.
   2) 차이가 jump_m (기본 0.10 m = 10 cm) 보다 크면 "이상치" → 새 경로는 버리고 믿은 경로를 그대로 보낸다.
      (정상 주행에서 1/30 초 사이 경로 변화는 보통 1 cm 미만. 10/09 E 주행 기록 10 Hz 기준 90% 가 4 cm 이하)
   3) 그런데 이상치가 outlier_hold_sec (기본 0.5 초) 넘게 계속되면 → "진짜로 길이 바뀐 것" 으로 보고 새 경로를 받아들인다.
      (영원히 옛 경로에 갇히지 않도록)
   4) 선을 아예 잃었을 때 E 는 "오른쪽으로 찾기(search_right)" 경로를 냈는데, 이것도 같은 규칙을 거친다.
      → 횡단보도에서 선이 잠깐 끊겨도 바로 오른쪽으로 꺾지 않고 0.5 초는 직전 경로로 간다.

조정값 (start 할 때 -p 로 바꿀 수 있음)
   jump_m            0.10   이보다 크게 튀면 이상치.  크게 하면 덜 막음, 작게 하면 커브 진입도 막을 수 있음
   outlier_hold_sec  0.5    이상치를 최대 몇 초 무시할지. 길게 하면 진짜 커브에 늦게 반응
   ref_max_age_sec   1.0    믿은 경로가 이보다 오래되면(선을 오래 잃었으면) 비교하지 않고 새 경로를 바로 믿음
   far_only_xmin_m   0.4    [10/09 추가] 경로를 만든 점들이 이 거리(m)보다 먼 곳에만 있으면(차 앞이 안 보임, 예: 횡단보도)
                            그 경로를 차 앞까지 늘린 값은 믿을 수 없다 → 이상치로 보고 믿은 경로를 계속 쓴다 (시간 제한 없이, ref_max_age 까지)

실행:  ALGO=F CONF=0.3 ~/jacapdi-docker/car_prep.sh
"""
import copy
import os
import sys
import time

import numpy as np
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from E_curve_cross import LinePathNodeE   # noqa: E402   E 의 모든 기능을 그대로 물려받는다


class LinePathNodeF(LinePathNodeE):
    CHECK_X = np.array([0.0, 0.4, 0.8])      # 경로를 비교할 차 앞 지점들 (m, x=0 은 차 앞 약 0.84 m)

    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        self.jump_m = p("jump_m", 0.10).value
        self.outlier_hold_sec = p("outlier_hold_sec", 0.5).value
        self.ref_max_age_sec = p("ref_max_age_sec", 1.0).value
        self.far_only_xmin_m = p("far_only_xmin_m", 0.4).value
        self.stats["far_only_held"] = 0
        self.ref = None; self.ref_band = None; self.ref_time = 0.0   # 마지막으로 "믿은" 경로
        self.outlier_since = None                                    # 이상치가 시작된 시각
        self.stats["outlier_held"] = 0
        self.stats["outlier_accepted"] = 0
        self.get_logger().info("algo F: outlier jump>%.2fm hold<=%.2fs" % (self.jump_m, self.outlier_hold_sec))

    # E 의 find_path 결과를 한 번 더 검사한다
    def find_path(self, md, ms, h, w, header):
        traj, band, reason = super().find_path(md, ms, h, w, header)
        if traj is None:
            return traj, band, reason                    # 실패는 E 와 같이 처리 (직전 경로 1초 유지 후 정지)
        now = time.time()
        ref_ok = self.ref is not None and now - self.ref_time <= self.ref_max_age_sec
        # [10/09] 먼 곳 점만으로 만든 경로 (차 앞 x_min 이 멀다) → 믿은 경로 유지
        if ref_ok and traj.x_min_m > self.far_only_xmin_m:
            self.stats["far_only_held"] += 1
            t = copy.deepcopy(self.ref); t.header = header
            return t, self.ref_band, "far_only_hold(xmin=%.2f,%s)" % (traj.x_min_m, reason)
        if ref_ok:
            y_new = np.polyval([traj.a, traj.b, traj.c], self.CHECK_X)
            y_ref = np.polyval([self.ref.a, self.ref.b, self.ref.c], self.CHECK_X)
            jump = float(np.abs(y_new - y_ref).max())    # 두 경로가 옆으로 가장 많이 벌어진 거리 (m)
            if jump > self.jump_m:
                if self.outlier_since is None:
                    self.outlier_since = now
                if now - self.outlier_since < self.outlier_hold_sec:
                    # 이상치 → 새 경로 버리고, 믿은 경로를 복사해서 보낸다 (시각만 지금으로)
                    self.stats["outlier_held"] += 1
                    t = copy.deepcopy(self.ref); t.header = header
                    self.ref_time = now                  # 붙잡고 있는 동안 믿은 경로가 낡았다고 보지 않음
                    return t, self.ref_band, "outlier_hold(%.2fm,%s)" % (jump, reason)
                self.stats["outlier_accepted"] += 1      # 0.5 초 넘게 계속 → 진짜 변화로 인정
                reason = "outlier_accepted(%.2fm),%s" % (jump, reason)
        # 정상 경로 → 믿은 경로로 저장
        self.outlier_since = None
        self.ref, self.ref_band, self.ref_time = copy.deepcopy(traj), band, now
        return traj, band, reason

    def log_stats(self):
        super().log_stats()
        self.get_logger().info("outlier_held=%d outlier_accepted=%d far_only_held=%d"
                               % (self.stats["outlier_held"], self.stats["outlier_accepted"], self.stats["far_only_held"]))


def main():
    rclpy.init()
    n = LinePathNodeF()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
```


---

## `python/1_path/lane_from_lines.py`

```python
# -*- coding: utf-8 -*-
"""흰 선 라벨(solid_line, dashed_line)에서 주행 영역 `lane` 폴리곤을 자동 생성해 YOLOv8 세그 데이터셋에 붙인다.

입력: Roboflow YOLOv8 세그 내보내기 폴더 (data.yaml + train/valid/test/{images,labels})
출력: 같은 구조의 새 폴더. 라벨 파일 끝에 `lane` 클래스 폴리곤 한 줄 추가, data.yaml names에 lane 추가,
      qa/ 폴더에 확인용 오버레이 PNG, report.json에 통계.

원리: 선 폴리곤을 마스크로 그리고, 이미지 아래(차 앞)에서 위로 한 줄씩 훑으며 현재 차로 중심의
      왼쪽에서 가장 가까운 선 조각과 오른쪽에서 가장 가까운 선 조각을 찾는다. 점선 사이 빈 줄은
      위아래 값으로 보간한다. 두 경계 사이를 채운 영역이 lane이다. 한쪽 선만 보이는 이미지는
      lane을 만들지 않고 report에 남긴다(그 이미지는 lane 학습에서 빼는 편이 안전).

사용:
  python lane_from_lines.py <내보내기 폴더> <출력 폴더> [--boundary solid_line dashed_line] [--qa 40]
  python lane_from_lines.py --selftest <출력 폴더>      # 합성 데이터로 동작 확인

정적 검사와 합성 시험만 했다. 실제 라벨 분포에서는 --min-width, --max-gap 조정이 필요할 수 있다.
"""
import argparse
import json
import os
import random
import shutil
import sys

import cv2
import numpy as np

try:
    import yaml
except ImportError:  # data.yaml을 간단 파서로 읽는다
    yaml = None

LANE_NAME = "lane"


# --------------------------------------------------------------------- data.yaml
def read_names(data_yaml):
    text = open(data_yaml, encoding="utf-8").read()
    if yaml is not None:
        d = yaml.safe_load(text)
        names = d.get("names")
        if isinstance(names, dict):
            names = [names[k] for k in sorted(names, key=int)]
        return d, list(names)
    names = []
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("names:"):
            inner = line[len("names:"):].strip().strip("[]")
            names = [n.strip().strip("'\"") for n in inner.split(",") if n.strip()]
    return {"names": names}, names


def write_names(src_yaml, dst_yaml, d, names):
    d = dict(d)
    d["names"] = names
    d["nc"] = len(names)
    if yaml is not None:
        open(dst_yaml, "w", encoding="utf-8").write(yaml.safe_dump(d, allow_unicode=True, sort_keys=False))
    else:
        lines = [l for l in open(src_yaml, encoding="utf-8").read().splitlines()
                 if not l.strip().startswith(("names:", "nc:"))]
        lines.append("nc: %d" % len(names))
        lines.append("names: [%s]" % ", ".join("'%s'" % n for n in names))
        open(dst_yaml, "w", encoding="utf-8").write("\n".join(lines) + "\n")


# --------------------------------------------------------------------- 라벨 파일
def read_label(path):
    """YOLO 세그 라벨: 줄마다 `cls x1 y1 x2 y2 ...` (정규화). (cls, [(x,y),...]) 목록."""
    out = []
    if not os.path.isfile(path):
        return out
    for line in open(path, encoding="utf-8"):
        t = line.split()
        if len(t) < 7:
            continue
        cls = int(float(t[0]))
        vals = list(map(float, t[1:]))
        pts = [(vals[i], vals[i + 1]) for i in range(0, len(vals) - 1, 2)]
        out.append((cls, pts))
    return out


def polygon_to_mask(polys, w, h):
    mask = np.zeros((h, w), np.uint8)
    for pts in polys:
        arr = np.array([[int(round(x * w)), int(round(y * h))] for x, y in pts], np.int32)
        if len(arr) >= 3:
            cv2.fillPoly(mask, [arr], 255)
    return mask


# --------------------------------------------------------------------- 핵심
def runs_in_row(row, gap=3):
    """한 행에서 선 픽셀의 연속 구간 [(xl, xr), ...]. gap 픽셀 이하 끊김은 같은 구간."""
    xs = np.flatnonzero(row)
    if xs.size == 0:
        return []
    runs = []
    start = prev = int(xs[0])
    for x in xs[1:]:
        x = int(x)
        if x - prev > gap + 1:
            runs.append((start, prev))
            start = x
        prev = x
    runs.append((start, prev))
    return runs


def scan_lane(mask, min_width, max_width, max_gap_rows, start_x=None):
    """아래에서 위로 훑어 행별 (left_edge, right_edge). 없는 행은 None.
    반환: L[h], R[h] (np.float 배열, nan = 없음), 통계 dict."""
    h, w = mask.shape
    L = np.full(h, np.nan)
    R = np.full(h, np.nan)
    mid = (w - 1) / 2.0 if start_x is None else float(start_x)
    last_both = None
    rows_since_any = 0
    stat = {"rows_both": 0, "rows_left_only": 0, "rows_right_only": 0, "rows_none": 0}
    for y in range(h - 1, -1, -1):
        runs = runs_in_row(mask[y])
        left = [r for r in runs if r[1] < mid]
        right = [r for r in runs if r[0] > mid]
        lx = max(left, key=lambda r: r[1])[1] if left else None
        rx = min(right, key=lambda r: r[0])[0] if right else None
        if lx is not None and rx is not None:
            width = rx - lx
            if width < min_width or width > max_width:
                stat["rows_none"] += 1
                rows_since_any += 1
            else:
                L[y], R[y] = lx, rx
                mid = (lx + rx) / 2.0
                last_both = y
                rows_since_any = 0
                stat["rows_both"] += 1
                continue
        elif lx is not None:
            L[y] = lx
            stat["rows_left_only"] += 1
            rows_since_any = 0
        elif rx is not None:
            R[y] = rx
            stat["rows_right_only"] += 1
            rows_since_any = 0
        else:
            stat["rows_none"] += 1
            rows_since_any += 1
        if rows_since_any > max_gap_rows and last_both is not None:
            break  # 선이 끝났다
    return L, R, stat


def interpolate_edges(L, R, max_gap_rows):
    """점선 사이처럼 한쪽만 빈 행을 위아래 값으로 선형 보간. 양 끝 밖은 채우지 않는다."""
    def fill(arr):
        idx = np.flatnonzero(~np.isnan(arr))
        if idx.size < 2:
            return arr
        out = arr.copy()
        for a, b in zip(idx[:-1], idx[1:]):
            if b - a > 1 and (b - a) <= max_gap_rows:
                out[a + 1:b] = np.interp(np.arange(a + 1, b), [a, b], [arr[a], arr[b]])
        return out
    return fill(L), fill(R)


def lane_polygon(L, R, min_rows, epsilon=2.0):
    """L, R가 모두 있는 연속 행 구간을 폴리곤으로. 픽셀 좌표 리스트 또는 None."""
    both = ~np.isnan(L) & ~np.isnan(R)
    if both.sum() < min_rows:
        return None
    ys = np.flatnonzero(both)
    # 가장 아래에서 시작하는 연속 구간만 사용
    ys = ys[::-1]  # 아래(큰 y)부터
    seg = [int(ys[0])]
    for y in ys[1:]:
        if seg[-1] - y == 1:
            seg.append(int(y))
        else:
            break
    if len(seg) < min_rows:
        return None
    left = [(float(L[y]), float(y)) for y in seg]            # 아래→위
    right = [(float(R[y]), float(y)) for y in reversed(seg)]  # 위→아래
    pts = np.array(left + right, np.float32).reshape(-1, 1, 2)
    approx = cv2.approxPolyDP(pts, epsilon, True).reshape(-1, 2)
    if len(approx) < 3:
        return None
    return [(float(x), float(y)) for x, y in approx], (seg[-1], seg[0])


def process_image(label_path, img_w, img_h, boundary_ids, args):
    items = read_label(label_path)
    polys = [pts for cls, pts in items if cls in boundary_ids]
    if not polys:
        return None, {"reason": "no_boundary_label"}
    mask = polygon_to_mask(polys, img_w, img_h)
    min_width = int(args.min_width * img_w)
    max_width = int(args.max_width * img_w)
    max_gap = int(args.max_gap * img_h)
    L, R, stat = scan_lane(mask, min_width, max_width, max_gap)
    L, R = interpolate_edges(L, R, max_gap)
    res = lane_polygon(L, R, min_rows=int(args.min_rows * img_h))
    if res is None:
        reason = "one_side_only" if (stat["rows_left_only"] + stat["rows_right_only"]) > stat["rows_both"] else "too_short"
        return None, {"reason": reason, **stat}
    poly, (top, bottom) = res
    norm = [(x / img_w, y / img_h) for x, y in poly]
    return norm, {"reason": "ok", "top_y": top, "bottom_y": bottom, **stat}


def overlay(img, mask, poly_px):
    vis = img.copy()
    blue = np.zeros_like(vis)
    blue[:, :, 0] = mask
    vis = cv2.addWeighted(vis, 1.0, blue, 0.6, 0)
    if poly_px is not None:
        fill = np.zeros_like(vis)
        cv2.fillPoly(fill, [np.array(poly_px, np.int32)], (0, 200, 0))
        vis = cv2.addWeighted(vis, 1.0, fill, 0.35, 0)
        cv2.polylines(vis, [np.array(poly_px, np.int32)], True, (0, 255, 0), 1)
    return vis


# --------------------------------------------------------------------- 실행
def convert(src, dst, args):
    data_yaml = os.path.join(src, "data.yaml")
    if not os.path.isfile(data_yaml):
        sys.exit("data.yaml 없음: %s" % data_yaml)
    d, names = read_names(data_yaml)
    boundary_ids = {names.index(n) for n in args.boundary if n in names}
    missing = [n for n in args.boundary if n not in names]
    if missing:
        sys.exit("경계 클래스가 data.yaml에 없음: %s (있는 이름: %s)" % (missing, names))
    if LANE_NAME in names:
        lane_id = names.index(LANE_NAME)
        new_names = names
    else:
        lane_id = len(names)
        new_names = names + [LANE_NAME]
    os.makedirs(dst, exist_ok=True)
    qa_dir = os.path.join(dst, "qa")
    os.makedirs(qa_dir, exist_ok=True)
    report = {"src": src, "dst": dst, "lane_class_id": lane_id, "boundary": sorted(boundary_ids), "splits": {}}
    qa_pool = []
    for split in ("train", "valid", "test"):
        img_dir = os.path.join(src, split, "images")
        lbl_dir = os.path.join(src, split, "labels")
        if not os.path.isdir(img_dir):
            continue
        out_img = os.path.join(dst, split, "images")
        out_lbl = os.path.join(dst, split, "labels")
        os.makedirs(out_img, exist_ok=True)
        os.makedirs(out_lbl, exist_ok=True)
        cnt = {"images": 0, "ok": 0, "one_side_only": 0, "too_short": 0, "no_boundary_label": 0, "unreadable": 0}
        skipped = []
        for fn in sorted(os.listdir(img_dir)):
            stem, ext = os.path.splitext(fn)
            if ext.lower() not in (".jpg", ".jpeg", ".png", ".bmp"):
                continue
            cnt["images"] += 1
            img = cv2.imread(os.path.join(img_dir, fn))
            if img is None:
                cnt["unreadable"] += 1
                continue
            h, w = img.shape[:2]
            lbl = os.path.join(lbl_dir, stem + ".txt")
            poly, info = process_image(lbl, w, h, boundary_ids, args)
            src_lines = open(lbl, encoding="utf-8").read().rstrip("\n") if os.path.isfile(lbl) else ""
            src_lines = "\n".join(l for l in src_lines.splitlines() if not l.startswith("%d " % lane_id)) if LANE_NAME in names else src_lines
            if poly is not None:
                cnt["ok"] += 1
                line = "%d " % lane_id + " ".join("%.6f %.6f" % (x, y) for x, y in poly)
                out_text = (src_lines + "\n" if src_lines else "") + line + "\n"
            else:
                cnt[info["reason"]] += 1
                skipped.append({"file": fn, **info})
                out_text = src_lines + ("\n" if src_lines else "")
            if args.skip_without_lane and poly is None:
                continue
            shutil.copy2(os.path.join(img_dir, fn), os.path.join(out_img, fn))
            open(os.path.join(out_lbl, stem + ".txt"), "w", encoding="utf-8").write(out_text)
            qa_pool.append((split, fn, os.path.join(img_dir, fn), lbl, poly))
        cnt["skipped_list"] = skipped[:200]
        report["splits"][split] = cnt
        print("%s: images %d, lane ok %d, one_side_only %d, too_short %d, no_label %d"
              % (split, cnt["images"], cnt["ok"], cnt["one_side_only"], cnt["too_short"], cnt["no_boundary_label"]))
    write_names(data_yaml, os.path.join(dst, "data.yaml"), d, new_names)
    for extra in ("README.roboflow.txt", "README.dataset.txt"):
        if os.path.isfile(os.path.join(src, extra)):
            shutil.copy2(os.path.join(src, extra), os.path.join(dst, extra))
    # QA 오버레이: 성공/실패 섞어서
    random.seed(0)
    random.shuffle(qa_pool)
    n_qa = 0
    for split, fn, img_path, lbl, poly in qa_pool:
        if n_qa >= args.qa:
            break
        img = cv2.imread(img_path)
        h, w = img.shape[:2]
        items = read_label(lbl)
        mask = polygon_to_mask([pts for cls, pts in items if cls in boundary_ids], w, h)
        poly_px = [(x * w, y * h) for x, y in poly] if poly else None
        cv2.imwrite(os.path.join(qa_dir, "%s_%s.png" % (split, os.path.splitext(fn)[0])), overlay(img, mask, poly_px))
        n_qa += 1
    json.dump(report, open(os.path.join(dst, "report.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("data.yaml names:", new_names)
    print("QA 오버레이 %d장: %s" % (n_qa, qa_dir))
    print("report: %s" % os.path.join(dst, "report.json"))
    return report


# --------------------------------------------------------------------- 합성 시험
def make_synthetic(root, n=6, w=320, h=240):
    """오른쪽 실선 + 왼쪽 점선(조각 여러 개) 곡선 트랙과, 한쪽 선만 있는 이미지 하나."""
    names = ["solid_line", "dashed_line", "traffic_light_red"]
    for split in ("train", "valid"):
        os.makedirs(os.path.join(root, split, "images"), exist_ok=True)
        os.makedirs(os.path.join(root, split, "labels"), exist_ok=True)
    rng = random.Random(1)
    for i in range(n):
        split = "train" if i < n - 2 else "valid"
        img = np.full((h, w, 3), 40, np.uint8)
        curve = rng.uniform(-0.0015, 0.0015)
        lines = []
        def xc(y, base):  # 곡선: 위로 갈수록 휨
            return base + curve * (h - y) ** 2
        # 오른쪽 실선 폴리곤 (폭 6px), 아래→위
        rpts = [(xc(y, 240) , y) for y in range(h, 60, -10)]
        solid = [(x - 3, y) for x, y in rpts] + [(x + 3, y) for x, y in reversed(rpts)]
        lines.append((0, solid))
        # 왼쪽 점선: 길이 24px, 간격 24px
        y = h
        while y > 70:
            y0, y1 = y, max(y - 24, 70)
            dash = [(xc(y0, 80) - 3, y0), (xc(y0, 80) + 3, y0), (xc(y1, 80) + 3, y1), (xc(y1, 80) - 3, y1)]
            lines.append((1, dash))
            y -= 48
        if i == n - 1:  # 한쪽 선만
            lines = [l for l in lines if l[0] == 0]
        for cls, pts in lines:
            cv2.fillPoly(img, [np.array(pts, np.int32)], (255, 255, 255))
        fn = "syn_%02d" % i
        cv2.imwrite(os.path.join(root, split, "images", fn + ".jpg"), img)
        with open(os.path.join(root, split, "labels", fn + ".txt"), "w", encoding="utf-8") as f:
            for cls, pts in lines:
                f.write("%d " % cls + " ".join("%.6f %.6f" % (x / w, y / h) for x, y in pts) + "\n")
    open(os.path.join(root, "data.yaml"), "w", encoding="utf-8").write(
        "train: ../train/images\nval: ../valid/images\nnc: 3\nnames: [%s]\n" % ", ".join("'%s'" % n for n in names))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src", nargs="?", help="Roboflow YOLOv8 세그 내보내기 폴더")
    ap.add_argument("dst", help="출력 폴더")
    ap.add_argument("--boundary", nargs="+", default=["solid_line", "dashed_line"], help="차로 경계로 쓸 클래스 이름")
    ap.add_argument("--min-width", type=float, default=0.10, help="차로 최소 폭, 이미지 폭 비율")
    ap.add_argument("--max-width", type=float, default=0.90, help="차로 최대 폭, 이미지 폭 비율")
    ap.add_argument("--max-gap", type=float, default=0.25, help="점선 사이 보간 최대 간격, 이미지 높이 비율")
    ap.add_argument("--min-rows", type=float, default=0.15, help="lane으로 인정할 최소 행 수, 이미지 높이 비율")
    ap.add_argument("--qa", type=int, default=40, help="확인용 오버레이 장수")
    ap.add_argument("--skip-without-lane", action="store_true", help="lane을 못 만든 이미지는 출력에서 뺀다")
    ap.add_argument("--selftest", action="store_true", help="합성 데이터로 동작 확인")
    args = ap.parse_args()
    if args.selftest:
        src = os.path.join(args.dst, "_synthetic_src")
        if os.path.isdir(src):
            shutil.rmtree(src)
        make_synthetic(src)
        rep = convert(src, os.path.join(args.dst, "_synthetic_out"), args)
        tr = rep["splits"]["train"]
        va = rep["splits"]["valid"]
        ok = tr["ok"] == tr["images"] and va["ok"] == va["images"] - 1 and va["one_side_only"] == 1
        print("SELFTEST", "PASS" if ok else "FAIL")
        sys.exit(0 if ok else 1)
    if not args.src:
        ap.error("src 폴더가 필요합니다")
    convert(args.src, args.dst, args)


if __name__ == "__main__":
    main()
```


---

## `python/2_control/team_lane_keeping_controller.py`

```python
from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class StanleyControlDebug:
    theta_e_rad: float
    cte_m: float
    curvature: float
    stanley_rad: float
    feedforward_rad: float
    delta_rad: float


class LaneKeepingController:
    """Stanley controller with curvature feedforward for y = ax^2 + bx + c."""

    def __init__(
        self,
        stanley_gain: float,
        curvature_gain: float,
        softening_speed_mps: float,
        front_axle_offset_m: float,
        heading_gain: float,
    ):
        self.stanley_gain = float(stanley_gain)
        self.curvature_gain = float(curvature_gain)
        self.softening_speed_mps = float(softening_speed_mps)
        self.front_axle_offset_m = float(front_axle_offset_m)
        self.heading_gain = float(heading_gain)

    def compute_delta(self, coeffs: Tuple[float, float, float], speed_mps: float):
        a, b, c = map(float, coeffs)
        x = self.front_axle_offset_m

        dy_dx = 2.0 * a * x + b
        d2y_dx2 = 2.0 * a

        theta_e = np.arctan(dy_dx)
        cte = a * x * x + b * x + c
        curvature = d2y_dx2 / np.power(1.0 + dy_dx * dy_dx, 1.5)

        speed_denom = abs(float(speed_mps)) + self.softening_speed_mps
        stanley = np.arctan((self.stanley_gain * cte) / speed_denom)
        feedforward = self.curvature_gain * curvature
        delta = self.heading_gain * theta_e + stanley + feedforward

        debug = StanleyControlDebug(
            theta_e_rad=float(theta_e),
            cte_m=float(cte),
            curvature=float(curvature),
            stanley_rad=float(stanley),
            feedforward_rad=float(feedforward),
            delta_rad=float(delta),
        )
        return float(delta), debug
```


---

## `python/2_control/tunable_controller.py`

```python
# -*- coding: utf-8 -*-
"""
=====================================================================================
 조정형 Stanley 제어기  (tunable_controller.py, 2026-10-09)
 "경로 곡선 → 핸들을 몇 단계 꺾을지 + 뒷바퀴를 얼마나 세게 돌릴지"
=====================================================================================

   /lane_trajectory (경로 a,b,c)  ──▶  이 노드  ──▶  /topic_control_signal (조향 단계, 왼쪽 PWM, 오른쪽 PWM)
                                         ▲
                     tune/tune.json (슬라이더 창 tune_gui.py 가 저장) 을 0.05 초마다 확인해서 바뀌면 바로 반영

계산식은 팀 코드(decision_making_pkg 의 LaneKeepingController = Stanley 제어)를 그대로 가져다 쓴다.
기본값이면 팀 stanley_controller_node 와 완전히 같은 명령이 나온다 (무작위 경로 1만 개로 확인).

Stanley 제어 한 줄 요약
   조향각 δ = heading_gain × (경로 방향 오차)  +  atan( stanley_gain × (옆으로 벗어난 거리) / (속도 + 저속완화) )
            + curvature_gain × (경로 곡률)
   - "옆으로 벗어난 거리" = 평가 지점(front_axle_offset_m)에서 경로가 차보다 얼마나 옆에 있나 (y)
   - "경로 방향 오차"     = 그 지점에서 경로가 차 앞 방향과 이루는 각도
   - "곡률"               = 경로가 얼마나 휘었나 (커브 미리 대응)
   그 다음 δ 를 -7 ~ +7 정수 단계로 바꾼다 (펌웨어 규칙: + 오른쪽, - 왼쪽).

이 노드에서 추가한 것
   - path_offset_m  : 경로 전체를 옆으로 옮기기 (+ 왼쪽).  오른쪽 실선을 자꾸 밟을 때
   - inside_bias_m  : [인코스] 커브에서만 경로를 커브 안쪽으로 당기기. 많이 휠수록 많이(최대 이 값).
                      바깥으로 크게 돌지 않고 안쪽 라인을 타서 랩타임 단축. 너무 크면 점선/실선을 밟는다.
   - steer_trim     : 직진이 한쪽으로 흐르면 조향 단계를 일정하게 보정
   - pwm_by_step    : 조향 단계(0~7)마다 뒷바퀴 PWM 따로 (커브에서 감속)
   - steer_rate     : [급발진 방지] 0.05 초마다 조향 단계를 최대 몇 단계까지 바꿀지 (0 = 제한 없음)
                      1 이면 0 → 7 까지 0.35 초. 조향이 확 튀지 않고 부드럽게 따라간다.
   - motor_balance  : [좌우 모터 보정] 직선에서 한쪽으로 흐를 때. + 면 왼쪽 뒷바퀴 PWM 을 그만큼 줄임(오른쪽 쏠림 보정),
                      - 면 오른쪽 뒷바퀴를 줄임(왼쪽 쏠림 보정). 0 = 둘 다 같은 PWM
   - lane_guard_left_m: [선 넘기 방지 - 왼쪽(점선) 쪽 기준] 따로 정하고 싶을 때 (0 = lane_guard_m 과 같게).
                      인코스로 커브 안쪽(왼쪽)에 붙을 때 너무 일찍 막지 않도록 오른쪽보다 크게 둔다.
   - inside_bias_right_m: [인코스 - 오른쪽 커브만] 0 이 아니면 오른쪽 커브에서는 inside_bias_m 대신 이 값.
                      - 면 오른쪽 커브를 바깥(왼쪽)으로 크게 돈다 = 늦게 꺾음 (왼쪽 커브 직후 오른쪽 커브에서 실선 안쪽을 질러가는 것 방지)
   - lane_guard_push: [선 넘기 방지 - 되돌리기] 선 넘기 방지 기준을 넘으면 반대쪽으로 최소 이 단계만큼은 꺾는다.
                      0 = 막기만 함(예전). 2 = 실선 쪽으로 치우치면 최소 왼쪽 2단계.
   - stall_boost_pwm: [멈춤 탈출] 경로가 1초 동안 거의 안 변하면(= 차가 멈춤, 예: 끝까지 꺾고 낮은 PWM)
                      0.5 초 동안 이 PWM 으로 세게 민다. 0 = 끔. (움직일 땐 경로가 1초에 6 cm 넘게 변함, 멈추면 0.3 cm 미만)
   - lane_guard_m   : [선 넘기 방지] 차가 차로 가운데에서 이 거리(m)보다 왼쪽에 있으면 왼쪽으로 더 못 꺾게,
                      오른쪽에 있으면 오른쪽으로 더 못 꺾게 막는다 (0 = 끔). 점선/실선 넘는 것 방지.

사용 (car_prep.sh 가 대신 실행):  python3 tunable_controller.py [기록폴더]
"""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from interfaces_pkg.msg import LaneTrajectory, MotionCommand
from decision_making_pkg.lib.lane_keeping_controller import LaneKeepingController   # 팀 Stanley 계산식

TUNE = os.path.expanduser("~/jacapdi-docker/tune/tune.json")       # 슬라이더 값 (사람이 바꿈)
STATUS = os.path.expanduser("~/jacapdi-docker/tune/status.json")   # 지금 상태 (슬라이더 창에 표시)
RUN = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else ""   # 기록 폴더
FW_MAX_STEP = 7          # 펌웨어가 받는 조향 단계 최대값 (s-7 ~ s7)
CURV_FULL = 0.6          # 인코스: 곡률이 이 값(1/m, 반경 약 1.7 m) 이상이면 inside_bias_m 를 다 씀

# 새로 생긴 설정이 예전 tune.json 에 없을 때 쓸 기본값 (기본값이면 동작이 예전과 같음)
DEFAULTS = {"inside_bias_m": 0.0, "steer_rate": 0, "lane_guard_m": 0.0, "motor_balance": 0, "stall_boost_pwm": 0, "lane_guard_left_m": 0.0, "lane_guard_push": 0, "inside_bias_right_m": 0.0}


class Tunable(Node):
    def __init__(self):
        super().__init__("tunable_controller")
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                       durability=DurabilityPolicy.VOLATILE, depth=1)
        self.pub = self.create_publisher(MotionCommand, "/topic_control_signal", q)   # 시리얼 송신기가 받는 통로
        self.create_subscription(LaneTrajectory, "/lane_trajectory", self.on_traj, q)
        self.p = None; self.mtime = 0.0; self.ctrl = None
        self.traj = None; self.traj_t = None      # 마지막으로 받은 정상 경로와 받은 시각
        self.n = 0
        self.prev_steer = 0                       # 직전에 보낸 조향 단계 (급발진 방지용)
        self.hist = []                            # 최근 1초 경로 (시각, b, c) - 멈춤 감지용
        self.boost_until = 0.0                    # 이 시각까지 멈춤 탈출 PWM 사용
        self.reload(force=True)
        self.create_timer(0.05, self.tick)        # 1초에 20번 명령 계산 (팀 노드와 같은 주기)

    # --- 슬라이더 값 다시 읽기 (파일이 바뀌었을 때만) ---
    def reload(self, force=False):
        try:
            mt = os.path.getmtime(TUNE)
            if not force and mt == self.mtime:
                return
            p = dict(DEFAULTS); p.update(json.load(open(TUNE)))
        except (OSError, ValueError):
            return                                # 슬라이더 창이 쓰는 도중이면 다음 주기에 다시
        self.mtime = mt; self.p = p
        # Stanley 계산기를 새 게인으로 다시 만든다
        self.ctrl = LaneKeepingController(stanley_gain=p["stanley_gain"], curvature_gain=p["curvature_gain"],
                                          softening_speed_mps=p["softening_speed_mps"],
                                          front_axle_offset_m=p["front_axle_offset_m"], heading_gain=p["heading_gain"])
        self.get_logger().info("tune loaded: %s" % json.dumps(p, ensure_ascii=False))
        if RUN:                                   # 언제 무엇으로 바꿨는지 기록 (나중에 비교용)
            with open(os.path.join(RUN, "tune_changes.log"), "a") as f:
                f.write("%s %s\n" % (time.strftime("%H:%M:%S"), json.dumps(p, ensure_ascii=False)))

    # --- 경로가 올 때마다: 정상 경로면 기억, 아니면 지움 ---
    def on_traj(self, m):
        if m.valid:
            self.traj, self.traj_t = m, time.monotonic()
        else:
            self.traj, self.traj_t = None, None

    # --- 1초에 20번: 명령 계산해서 보내기 ---
    def tick(self):
        self.reload()
        p = self.p
        # (1) 경로가 없거나 너무 오래됐으면 정지 명령 (안전)
        stale = self.traj is None or (time.monotonic() - self.traj_t) > p["lane_timeout_sec"]
        delta = None; shift = 0.0
        if stale:
            steer, l, r = 0, 0, 0
        else:
            t = self.traj
            # (2) 경로를 옆으로 옮기기: 고정 이동(path_offset) + 커브 안쪽으로 당기기(인코스)
            x0 = p["front_axle_offset_m"]
            curv = 2.0 * t.a / (1.0 + (2.0 * t.a * x0 + t.b) ** 2) ** 1.5    # 평가 지점의 곡률 (+ = 왼쪽 커브)
            bias = p["inside_bias_m"]
            if curv < 0 and p["inside_bias_right_m"] != 0:      # 오른쪽 커브(곡률 -)는 따로 정한 값
                bias = p["inside_bias_right_m"]
            shift = p["path_offset_m"] + bias * float(np.clip(curv / CURV_FULL, -1.0, 1.0))
            # (3) Stanley 로 조향각(라디안) 계산
            delta, _ = self.ctrl.compute_delta((t.a, t.b, t.c + shift), p["vehicle_speed_mps"])
            # (4) 조향각 → 조향 단계.  최대 조향각(max_steer_deg)을 최대 단계로 맞춘다.
            mx = int(max(1, min(FW_MAX_STEP, p["max_steering_step"])))
            norm = float(np.clip(delta / max(math.radians(p["max_steer_deg"]), 1e-6), -1.0, 1.0))
            steer = int(round(p["steering_sign"] * norm * mx)) + int(p["steer_trim"])   # sign=-1: 왼쪽 각도 → 음수 단계
            steer = int(np.clip(steer, -mx, mx))
            # (4-1) [선 넘기 방지] 차 앞(x=0, 약 0.84 m 앞)에서 차로 가운데가 차보다 어디 있나 = t.c
            #       t.c < -guard : 가운데가 오른쪽 = 차가 왼쪽(점선 쪽)으로 치우침 → 왼쪽(음수) 조향 금지
            #       t.c > +guard : 가운데가 왼쪽   = 차가 오른쪽(실선 쪽)으로 치우침 → 오른쪽(양수) 조향 금지
            g = p["lane_guard_m"]                                  # 오른쪽(실선) 쪽 기준
            gl = p["lane_guard_left_m"] if p["lane_guard_left_m"] > 0 else g   # 왼쪽(점선) 쪽 기준
            push = int(round(p["lane_guard_push"]))                  # 넘으면 반대쪽으로 최소 이만큼
            if gl > 0 and t.c < -gl: steer = max(steer, push)       # 점선 쪽으로 치우침 → 최소 오른쪽 push 단계
            if g > 0 and t.c > g: steer = min(steer, -push)         # 실선 쪽으로 치우침 → 최소 왼쪽 push 단계
            # (4-2) [급발진 방지] 직전 단계에서 steer_rate 단계 넘게 한 번에 바뀌지 않게
            rate = int(round(p["steer_rate"]))
            if rate > 0:
                steer = int(np.clip(steer, self.prev_steer - rate, self.prev_steer + rate))
            # (5) 조향 단계에 맞는 뒷바퀴 PWM (많이 꺾을수록 느리게 설정 가능)
            l = r = int(np.clip(p["pwm_by_step"][abs(steer)], 0, 255))
            # (5-0) [멈춤 탈출] 최근 1초 동안 경로(b, c)가 1 cm 도 안 변했으면 차가 서 있는 것 → 0.5 초 세게
            now = time.monotonic()
            self.hist = [h for h in self.hist if now - h[0] <= 1.0] + [(now, t.b, t.c)]
            boost = int(p["stall_boost_pwm"])
            if boost > 0 and now >= self.boost_until and now - self.hist[0][0] >= 0.9:
                if max(max(h[1] for h in self.hist) - min(h[1] for h in self.hist),
                       max(h[2] for h in self.hist) - min(h[2] for h in self.hist)) < 0.01:
                    self.boost_until = now + 0.5
                    self.hist = []
                    self.get_logger().info("stall detected -> boost PWM %d for 0.5 s" % boost)
            if now < self.boost_until:
                l = r = max(l, boost)
            # (5-1) [좌우 모터 보정] 두 모터 힘이 달라 한쪽으로 흐르면 센 쪽을 줄인다
            bal = int(round(p["motor_balance"]))
            if bal > 0: l = max(0, l - bal)
            if bal < 0: r = max(0, r + bal)
        self.prev_steer = steer
        msg = MotionCommand(); msg.steering, msg.left_speed, msg.right_speed = steer, l, r
        self.pub.publish(msg)

        # (6) 0.2 초마다 슬라이더 창에 상태 알림
        self.n += 1
        if self.n % 4 == 0:
            st = {"time": time.time(), "steer": steer, "pwm": l, "valid": not stale,
                  "c": None if stale else round(self.traj.c, 3), "b": None if stale else round(self.traj.b, 3),
                  "shift": round(shift, 3), "delta_deg": None if delta is None else round(math.degrees(delta), 1)}
            try:
                with open(STATUS + ".tmp", "w") as f:
                    json.dump(st, f)
                os.replace(STATUS + ".tmp", STATUS)
            except OSError:
                pass


rclpy.init()
n = Tunable()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
finally:
    m = MotionCommand(); m.steering = m.left_speed = m.right_speed = 0    # 끝날 때 정지 명령
    n.pub.publish(m); n.destroy_node(); rclpy.try_shutdown()
```


---

## `python/3_slider_gui/tune_gui.py`

```python
# -*- coding: utf-8 -*-
"""자캡디 주행 튜닝 슬라이더 (노트북에서 실행: python3 ~/jacapdi-docker/tune_gui.py)
슬라이더를 움직이면 ~/jacapdi-docker/tune/tune.json 에 바로 저장되고, 컨테이너의 tunable_controller.py 가 0.05초 안에 반영한다.
주행 중에도 바꿀 수 있다. 위쪽 상태 줄은 제어기가 0.2초마다 남기는 status.json 을 보여준다."""
import json, os, time
import tkinter as tk
from tkinter import ttk, filedialog

DIR = os.path.expanduser("~/jacapdi-docker/tune")
TUNE, DEFAULT, STATUS = DIR + "/tune.json", DIR + "/tune_default.json", DIR + "/status.json"

# (키, 이름, 최소, 최대, 간격, 설명)
CTRL = [
    ("stanley_gain", "Stanley 게인 / Stanley gain", 0.0, 3.0, 0.05, "차로 중심에서 벗어난 거리에 대한 반응. 크면 세게 복귀(흔들림 증가)"),
    ("heading_gain", "방향 게인 / Heading gain", 0.0, 2.0, 0.05, "경로 방향과 차 방향 차이에 대한 반응. 크면 흔들림 감쇠·커브 진입 빠름"),
    ("vehicle_speed_mps", "속도 가정 / Assumed speed (m/s)", 0.2, 3.0, 0.05, "Stanley 식의 속도. 크면 거리 오차 반응이 약해짐"),
    ("curvature_gain", "곡률 보정 / Curvature gain", 0.0, 2.0, 0.05, "경로가 휘는 만큼 미리 조향 (커브 대응)"),
    ("softening_speed_mps", "저속 완화 / Softening speed", 0.05, 1.0, 0.05, "저속에서 조향이 튀는 것 완화"),
    ("front_axle_offset_m", "평가 지점 / Look-ahead offset (m)", -0.8, 0.8, 0.02, "경로를 평가하는 앞뒤 위치 (0 = 카메라 보정 근거리 끝)"),
    ("max_steer_deg", "최대 조향각 / Max steer angle (deg)", 10, 40, 1, "이 각도 = 7단계. 작을수록 더 많이 꺾음, 클수록 덜 꺾음 / Smaller = steers more"),
    ("max_steering_step", "최대 조향 단계 / Max steer step", 1, 7, 1, "펌웨어로 보내는 조향 단계 상한"),
    ("steer_trim", "조향 트림 / Steer trim (step)", -3, 3, 1, "직진이 한쪽으로 흐를 때 보정 (+ 오른쪽)"),
    ("path_offset_m", "경로 좌우 이동 / Path offset (m)", -0.3, 0.3, 0.01, "+ 이면 경로를 왼쪽으로 (오른쪽 실선 침범 시)"),
    ("inside_bias_m", "인코스 / Inside bias (m)", -0.2, 0.2, 0.01, "+ 커브 안쪽으로, - 바깥쪽으로. 커브에서 안쪽 점선 밟으면 - 로 / + inside, - outside"),
    ("inside_bias_right_m", "인코스-오른쪽 커브 / Inside bias right (m)", -0.2, 0.2, 0.01, "0 = 위와 같게. - 면 오른쪽 커브를 늦게·크게 돎 / - = later, wider right turns"),
    ("steer_rate", "급발진 방지 / Steer rate limit (step/0.05s)", 0, 7, 1, "조향을 한 번에 몇 단계까지 바꿀지. 1 = 부드럽게, 0 = 제한 없음"),
    ("motor_balance", "좌우 모터 보정 / Motor balance (PWM)", -30, 30, 1, "직선에서 오른쪽으로 흐르면 + (왼쪽 모터 줄임), 왼쪽으로 흐르면 -"),
    ("stall_boost_pwm", "멈춤 탈출 / Stall boost (PWM)", 0, 255, 5, "1초 동안 차가 안 움직이면 0.5초 이 PWM 으로. 0 = 끔 / 0 = off"),
    ("lane_guard_m", "선 넘기 방지-오른쪽 / Lane guard right (m)", 0.0, 0.3, 0.01, "실선 쪽으로 이만큼 벗어나면 오른쪽으로 더 못 꺾음. 0 = 끔"),
    ("lane_guard_push", "선 넘기 방지-되돌리기 / Lane guard push (step)", 0, 4, 1, "기준 넘으면 반대쪽으로 최소 이 단계. 0 = 막기만 / 0 = block only"),
    ("lane_guard_left_m", "선 넘기 방지-왼쪽 / Lane guard left (m)", 0.0, 0.3, 0.01, "점선 쪽 기준. 0 = 오른쪽과 같게. 인코스 쓰면 오른쪽보다 크게 (0.18)"),
    ("lane_timeout_sec", "경로 끊김 허용 / Path timeout (s)", 0.1, 1.0, 0.05, "이 시간 넘게 경로 없으면 정지"),
]

def load(path):
    with open(path) as f: return json.load(f)

class App:
    def __init__(self, root):
        self.root = root; root.title("자캡디 주행 튜닝 / Jacapdi drive tuning")
        self.vals = load(TUNE) if os.path.exists(TUNE) else load(DEFAULT)
        self.vals.setdefault("inside_bias_m", 0.0)
        self.vals.setdefault("steer_rate", 0)
        self.vals.setdefault("lane_guard_m", 0.0)
        self.vals.setdefault("motor_balance", 0)
        self.vals.setdefault("stall_boost_pwm", 0)
        self.vals.setdefault("lane_guard_left_m", 0.0)
        self.vals.setdefault("lane_guard_push", 0)
        self.vals.setdefault("inside_bias_right_m", 0.0)
        self.vars = {}; self.pwm_vars = []; self.pending = None; self.labels = []
        top = ttk.Frame(root, padding=8); top.pack(fill="both", expand=True)
        self.status = tk.StringVar(value="제어기 상태: 대기")
        ttk.Label(top, textvariable=self.status, font=("TkFixedFont", 11, "bold")).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 6))

        box = ttk.LabelFrame(top, text="조향 / Steering (Stanley)", padding=6); box.grid(row=1, column=0, sticky="nsew", padx=(0, 8))
        for i, (k, name, lo, hi, step, tip) in enumerate(CTRL):
            v = tk.DoubleVar(value=self.vals[k]); self.vars[k] = (v, step)
            ttk.Label(box, text=name, width=34).grid(row=2 * i, column=0, sticky="w")
            val = ttk.Label(box, width=7); val.grid(row=2 * i, column=2)
            s = tk.Scale(box, from_=lo, to=hi, resolution=step, orient="horizontal", length=320, variable=v, showvalue=False,
                         command=lambda _=None, v=v, val=val, st=step: self.changed(v, val, st))
            s.grid(row=2 * i, column=1); self.fmt(v, val, step); self.labels.append((v, val, step))
            ttk.Label(box, text=tip, foreground="#666", font=("TkDefaultFont", 8)).grid(row=2 * i + 1, column=0, columnspan=3, sticky="w")

        pw = ttk.LabelFrame(top, text="조향 단계별 뒷바퀴 PWM / Rear PWM per steer step", padding=6); pw.grid(row=1, column=1, sticky="nsew")
        for i in range(8):
            v = tk.IntVar(value=int(self.vals["pwm_by_step"][i])); self.pwm_vars.append(v)
            ttk.Label(pw, text="단계 Step %d%s" % (i, " (직진 straight)" if i == 0 else (" (최대 max)" if i == 7 else "")), width=24).grid(row=i, column=0, sticky="w")
            val = ttk.Label(pw, width=5); val.grid(row=i, column=2)
            tk.Scale(pw, from_=0, to=255, resolution=5, orient="horizontal", length=260, variable=v, showvalue=False,
                     command=lambda _=None, v=v, val=val: self.changed(v, val, 1)).grid(row=i, column=1)
            self.fmt(v, val, 1); self.labels.append((v, val, 1))
        self.all_pwm = tk.IntVar(value=int(self.vals["pwm_by_step"][0]))
        row = ttk.Frame(pw); row.grid(row=9, column=0, columnspan=3, pady=(8, 0), sticky="w")
        ttk.Label(row, text="전체 / All:").pack(side="left")
        ttk.Spinbox(row, from_=0, to=255, increment=10, textvariable=self.all_pwm, width=5).pack(side="left", padx=4)
        ttk.Button(row, text="모든 단계에 적용 / Apply to all", command=self.set_all).pack(side="left")
        ttk.Button(row, text="커브 감속 예시 / Slow in curves", command=self.curve_preset).pack(side="left", padx=4)

        btn = ttk.Frame(top); btn.grid(row=2, column=0, columnspan=2, pady=(8, 0), sticky="w")
        ttk.Button(btn, text="기본값 / Default", command=self.reset).pack(side="left")
        ttk.Button(btn, text="프리셋 저장 / Save preset", command=self.save_preset).pack(side="left", padx=6)
        ttk.Button(btn, text="프리셋 열기 / Open preset", command=self.open_preset).pack(side="left")
        ttk.Button(btn, text="커브 적극 / Curve aggressive", command=lambda: self.apply(load(DIR + "/preset_curve_aggressive.json"))).pack(side="left", padx=6)
        ttk.Button(btn, text="커브 중간 / Curve mid", command=lambda: self.apply(load(DIR + "/preset_curve_mid.json"))).pack(side="left")
        self.saved = tk.StringVar(value=""); ttk.Label(btn, textvariable=self.saved, foreground="#2e7a4e").pack(side="left", padx=10)
        self.poll()

    def fmt(self, v, lab, step):
        x = v.get(); lab.config(text=("%d" % round(x)) if step >= 1 else ("%.2f" % x))

    def changed(self, v, lab, step):
        self.fmt(v, lab, step)
        if self.pending: self.root.after_cancel(self.pending)
        self.pending = self.root.after(150, self.write)      # 드래그 중엔 0.15초마다 한 번만 저장

    def collect(self):
        d = dict(self.vals)
        for k, (v, step) in self.vars.items():
            d[k] = int(round(v.get())) if step >= 1 and k in ("max_steering_step", "steer_trim", "steer_rate", "motor_balance", "stall_boost_pwm", "lane_guard_push") else round(float(v.get()), 3)
        d["pwm_by_step"] = [int(v.get()) for v in self.pwm_vars]
        return d

    def write(self):
        self.pending = None; d = self.collect(); self.vals = d
        tmp = TUNE + ".tmp"
        with open(tmp, "w") as f: json.dump(d, f, indent=1, ensure_ascii=False)
        os.replace(tmp, TUNE)
        self.my_mtime = os.path.getmtime(TUNE)               # 내가 쓴 파일 시각 (밖에서 바뀐 것과 구분)
        self.saved.set("저장됨 %s" % time.strftime("%H:%M:%S"))

    def apply(self, d):
        for k, (v, _) in self.vars.items(): v.set(d[k])
        for v, x in zip(self.pwm_vars, d["pwm_by_step"]): v.set(int(x))
        self.refresh_labels(); self.write()

    def refresh_labels(self):
        for v, lab, step in self.labels: self.fmt(v, lab, step)

    def set_all(self):
        for v in self.pwm_vars: v.set(int(self.all_pwm.get()))
        self.refresh_labels(); self.write()

    def curve_preset(self):
        base = int(self.all_pwm.get())
        for i, v in enumerate(self.pwm_vars): v.set(int(base if i <= 3 else base * (1 - 0.1 * (i - 3))))
        self.refresh_labels(); self.write()

    def reset(self): self.apply(load(DEFAULT))

    def save_preset(self):
        p = filedialog.asksaveasfilename(initialdir=DIR, initialfile="preset_%s.json" % time.strftime("%m%d_%H%M"), defaultextension=".json")
        if p:
            with open(p, "w") as f: json.dump(self.collect(), f, indent=1, ensure_ascii=False)
            self.saved.set("프리셋 저장: %s" % os.path.basename(p))

    def open_preset(self):
        p = filedialog.askopenfilename(initialdir=DIR, filetypes=[("json", "*.json")])
        if p: self.apply(load(p))

    def sync_from_file(self):
        # 다른 곳(Claude, 다른 창)에서 tune.json 을 바꿨으면 슬라이더도 그 값으로 맞춘다 (덮어쓰기 방지)
        try:
            mt = os.path.getmtime(TUNE)
            if self.pending or mt == getattr(self, "my_mtime", None):
                return
            d = load(TUNE)
        except (OSError, ValueError):
            return
        self.my_mtime = mt; self.vals = d
        for k, (v, _) in self.vars.items():
            if k in d: v.set(d[k])
        for v, x in zip(self.pwm_vars, d["pwm_by_step"]): v.set(int(x))
        self.refresh_labels()
        self.saved.set("파일에서 다시 읽음 %s" % time.strftime("%H:%M:%S"))

    def poll(self):
        self.sync_from_file()
        try:
            s = load(STATUS); age = time.time() - s["time"]
            if age > 2: self.status.set("제어기 상태: 응답 없음 (car_prep.sh 실행했는지 확인)")
            else:
                self.status.set("조향 %+d  PWM %3d  경로 %s  c=%s  b=%s  이동 %s  각도 %s°" % (
                    s["steer"], s["pwm"], "OK " if s["valid"] else "없음", s["c"], s["b"], s.get("shift"), s["delta_deg"]))
        except (OSError, ValueError, KeyError):
            self.status.set("제어기 상태: 대기 (status.json 없음)")
        self.root.after(200, self.poll)

root = tk.Tk(); App(root); root.mainloop()
```


---

## `python/4_tools/board_monitor.py`

```python
"""아두이노 보드 메시지 읽기 전용 기록기 (명령을 보내지 않음). 사용: python3 board_monitor.py <로그파일>
FAULT / 부팅 배너(DRIVING_TEAM_CAL_V1) 를 시각과 함께 남긴다. 포트를 열 때 보드가 한 번 리셋된다(DTR)."""
import sys, time, serial
out = open(sys.argv[1], "a", buffering=1)
s = serial.Serial(); s.port = "/dev/ttyACM0"; s.baudrate = 115200; s.timeout = 0.5; s.open()
out.write("%s monitor start\n" % time.strftime("%H:%M:%S"))
buf = b""
try:
    while True:
        try:
            buf += s.read(256)
        except serial.SerialException as e:
            # 다른 프로그램(정지 명령 전송 등)이 같은 포트를 잠깐 열면 생기는 오류 -> 기록만 하고 계속 읽는다
            # (포트를 다시 열면 보드가 리셋되므로 다시 열지 않음)
            out.write("%s monitor read error (ignored): %s\n" % (time.strftime("%H:%M:%S"), e))
            time.sleep(0.2)
            continue
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            out.write("%s %s\n" % (time.strftime("%H:%M:%S"), line.decode(errors="replace").strip()))
except KeyboardInterrupt:
    pass
```


---

## `python/4_tools/calib_pitch.py`

```python
# -*- coding: utf-8 -*-
"""pitch 만 재보정 (f=560, h=0.607 은 직선구간 보정값 고정). 커브에서도 동작.
양쪽 선(왼쪽 dashed 안쪽 끝, 오른쪽 solid 안쪽 끝)이 같은 행에 보이는 곳의 차로폭이 0.85m 가 되는 pitch 를 찾는다.
IPM 범위: 전방 X_near(보닛 위 행) ~ X_far(기본 1.4m, 차 바로 앞만). 실패하면 yaml 변경 안 함."""
import os, sys, time
import cv2, numpy as np, rclpy, yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from interfaces_pkg.msg import DetectionArray
F, HGT, CX, CY, W, H, LANE, HALF = 560., 0.607, 320., 240., 640, 480, 0.85, 0.8
X_FAR = float(sys.argv[1]) if len(sys.argv) > 1 else 1.4
NEAR_ROW = int(sys.argv[2]) if len(sys.argv) > 2 else 440
OUT = os.path.expanduser("~/jacapdi-docker/ipm_c920.yaml")
rclpy.init(); n = Node("calib_pitch"); dets = []
n.create_subscription(DetectionArray, "/detections", dets.append, QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1))
end = time.time() + 3
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
rows = []
for d in dets[-30:]:
    md = np.zeros((H, W), np.uint8); ms = np.zeros((H, W), np.uint8)
    for x in d.detections:
        if x.mask.data and x.class_name in ("dashed_line", "solid_line"):
            cv2.fillPoly(md if x.class_name == "dashed_line" else ms, [np.array([[p.x, p.y] for p in x.mask.data], np.int32)], 255)
    for y in range(H // 3, H):
        a = np.flatnonzero(md[y]); b = np.flatnonzero(ms[y]); b = b[b > (a.max() if a.size else 0)]
        if a.size and b.size and b.min() > a.max() + 20: rows.append((y, a.max(), b.min()))
if len(rows) < 30:
    print("FAIL: 양쪽 선이 함께 보이는 행 부족 (%d)" % len(rows)); sys.exit(1)
rows = np.array(rows, float)
def ground(u, v, th):
    a = (u - CX) / F; b = (v - CY) / F
    Xd = np.cos(th) - np.sin(th) * b; Zd = -(np.sin(th) + np.cos(th) * b); t = HGT / -Zd
    return t * Xd, -t * a
best = None
for th_deg in np.arange(3, 50, 0.25):
    th = np.deg2rad(th_deg)
    _, YL = ground(rows[:, 1], rows[:, 0], th); _, YR = ground(rows[:, 2], rows[:, 0], th)
    wmed = float(np.median(YL - YR))
    if best is None or abs(wmed - LANE) < abs(best[1] - LANE): best = (th_deg, wmed)
th_deg, wmed = best; th = np.deg2rad(th_deg)
if abs(wmed - LANE) > 0.05:
    print("FAIL: pitch 탐색 실패 (best %.1fdeg width %.2f)" % best); sys.exit(1)
def proj(X, Y):
    xc = -Y; yc = -X * np.sin(th) + HGT * np.cos(th); zc = X * np.cos(th) + HGT * np.sin(th)
    return CX + F * xc / zc, CY + F * yc / zc
Xs = np.linspace(0.05, 10, 20000); _, vv = proj(Xs, 0 * Xs)
X_near = round(float(Xs[np.argmin(np.abs(vv - NEAR_ROW))]) + 0.02, 2)
if X_FAR <= X_near + 0.3:
    print("FAIL: X_far %.2f 가 X_near %.2f 에 너무 가까움" % (X_FAR, X_near)); sys.exit(1)
src = [float(round(c, 1)) for X, Y in [(X_FAR, HALF), (X_FAR, -HALF), (X_near, -HALF), (X_near, HALF)] for c in proj(X, Y)]
p = {"use_sim_time": False, "sub_detection_topic": "/lane_detections", "drivable_class_name": "lane",
     "ipm_src_points": src, "ipm_src_base_width": 640.0, "ipm_src_base_height": 480.0,
     "ipm_dst_left_ratio": 0.3, "ipm_dst_right_ratio": 0.7, "bev_height_scale": 4.0,
     "lateral_meter_per_pixel": round(2 * HALF / 256, 6), "longitudinal_meter_per_pixel": round((X_FAR - X_near) / 1919, 7), "ego_x_px": -1.0}
if os.path.isfile(OUT): os.replace(OUT, OUT + ".prev")
open(OUT, "w").write("# %s calib_pitch.py: f=560 h=0.607 pitch=%.2fdeg (rows %d, lane width %.3fm). 전방 %.2f~%.2fm, 좌우 +-%.1fm.\n"
                     % (time.strftime("%F %T"), th_deg, len(rows), wmed, X_near, X_FAR, HALF)
                     + yaml.safe_dump({"lane_info_extractor_node": {"ros__parameters": p}}, sort_keys=False))
print("OK pitch=%.2fdeg width=%.3f rows=%d X %.2f..%.2f m src=%s" % (th_deg, wmed, len(rows), X_near, X_FAR, src))
```


---

## `python/4_tools/frame_logger.py`

```python
# -*- coding: utf-8 -*-
"""주행 시계열 기록기 (10 Hz). 주행에는 관여하지 않는다(구독만).
사용: python3 frame_logger.py <출력폴더> [hz=10] [raw=0]
  <출력폴더>/frames/NNNNNN_HHMMSS.mmm.png : 카메라 + YOLO 마스크/확신도 + IPM 영역 + 경로 곡선 + 명령/조향값
  <출력폴더>/raw/NNNNNN_....png            : (raw=1 일 때) 덧칠 없는 원본
  <출력폴더>/frames.csv                     : 같은 번호의 시계열 한 줄씩
보드 조향값(ADC/목표)은 같은 기록 폴더 위의 board.log 마지막 줄에서 읽는다."""
import csv, os, sys, time, threading
import cv2, numpy as np, rclpy, yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from interfaces_pkg.msg import DetectionArray, LaneTrajectory, MotionCommand

OUT = sys.argv[1]
HZ = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
RAW = (sys.argv[3] if len(sys.argv) > 3 else "0") == "1"
BOARD = os.path.join(os.path.dirname(OUT.rstrip("/")), "board.log")
IPM = yaml.safe_load(open(os.path.expanduser("~/jacapdi-docker/ipm_c920.yaml")))["lane_info_extractor_node"]["ros__parameters"]
COL = {"solid_line": (255, 80, 0), "dashed_line": (0, 220, 0)}
os.makedirs(OUT + "/frames", exist_ok=True)
if RAW: os.makedirs(OUT + "/raw", exist_ok=True)

def stamp(h): return h.stamp.sec + h.stamp.nanosec * 1e-9

def board_last():
    """board.log 끝부분에서 마지막 조향 보고 (ADC, T, S, FAULT 여부)."""
    try:
        with open(BOARD, "rb") as f:
            f.seek(0, 2); n = f.tell(); f.seek(max(0, n - 600)); lines = f.read().decode(errors="replace").splitlines()
    except OSError:
        return "", "", ""
    for l in reversed(lines):
        if "ADC=" in l:
            kv = dict(x.split("=", 1) for x in l.split()[1:] if "=" in x)
            state = "FAULT" if "FAULT" in l else ("IDLE" if "IDLE" in l else "ACTIVE")
            return kv.get("ADC", ""), kv.get("T", ""), state
    return "", "", ""

class Logger(Node):
    def __init__(self):
        super().__init__("frame_logger")
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.br = CvBridge(); self.lock = threading.Lock()
        self.imgs = {}; self.det = None; self.traj = None; self.cmd = None
        self.create_subscription(Image, "/image_raw", self.on_img, 10)
        self.create_subscription(DetectionArray, "/detections", lambda m: self._set("det", m), q)
        self.create_subscription(LaneTrajectory, "/lane_trajectory", lambda m: self._set("traj", m), q)
        self.create_subscription(MotionCommand, "/topic_control_signal", lambda m: self._set("cmd", m), q)
        self.f = open(OUT + "/frames.csv", "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["idx", "wall_time", "file", "img_stamp", "det_stamp", "det_age_ms", "n_solid", "n_dashed", "n_other",
                         "solid_max_score", "dashed_max_score", "other_classes", "traj_valid", "a", "b", "c", "x_min_m", "x_max_m",
                         "fit_error_m", "traj_age_ms", "steer", "left_pwm", "right_pwm", "board_adc", "board_target", "board_state"])
        self.idx = 0
        self.create_timer(1.0 / HZ, self.tick)

    def _set(self, k, m):
        with self.lock: setattr(self, k, m)

    def on_img(self, m):
        with self.lock:
            self.imgs[round(stamp(m.header), 6)] = m
            if len(self.imgs) > 15:
                for k in sorted(self.imgs)[:-15]: del self.imgs[k]

    def tick(self):
        with self.lock:
            if not self.imgs: return
            det, traj, cmd = self.det, self.traj, self.cmd
            m = self.imgs.get(round(stamp(det.header), 6)) if det is not None else None
            m = m or self.imgs[max(self.imgs)]          # 추론한 바로 그 프레임, 없으면 최신
        img = self.br.imgmsg_to_cv2(m, "bgr8"); now = time.time(); ist = stamp(m.header)
        name = "%06d_%s.%03d.png" % (self.idx, time.strftime("%H%M%S", time.localtime(now)), int(now * 1000) % 1000)
        if RAW: cv2.imwrite(OUT + "/raw/" + name, img, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        vis = img.copy(); over = img.copy(); counts = {}; best = {}
        if det is not None:
            for d in det.detections:
                counts[d.class_name] = counts.get(d.class_name, 0) + 1
                best[d.class_name] = max(best.get(d.class_name, 0.0), d.score)
                if d.mask.data:
                    pts = np.array([[int(p.x), int(p.y)] for p in d.mask.data], np.int32)
                    cv2.fillPoly(over, [pts], COL.get(d.class_name, (0, 0, 255)))
                    cv2.putText(vis, "%s %.2f" % (d.class_name, d.score), tuple(int(v) for v in pts[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        vis = cv2.addWeighted(over, 0.4, vis, 0.6, 0)
        h, w = img.shape[:2]
        src = np.float32(IPM["ipm_src_points"]).reshape(4, 2); src[:, 0] *= w / IPM["ipm_src_base_width"]; src[:, 1] *= h / IPM["ipm_src_base_height"]
        cv2.polylines(vis, [src.astype(np.int32)], True, (0, 255, 255), 1)
        if traj is not None and traj.valid:
            bh = int(round(h * IPM["bev_height_scale"])); lat, lon = IPM["lateral_meter_per_pixel"], IPM["longitudinal_meter_per_pixel"]
            M = cv2.getPerspectiveTransform(src, np.float32([[w * IPM["ipm_dst_left_ratio"], 0], [w * IPM["ipm_dst_right_ratio"], 0],
                                                             [w * IPM["ipm_dst_right_ratio"], bh - 1], [w * IPM["ipm_dst_left_ratio"], bh - 1]]))
            xs = np.linspace(traj.x_min_m, traj.x_max_m, 30); ys = traj.a * xs ** 2 + traj.b * xs + traj.c
            bp = np.stack([(w - 1) * 0.5 - ys / lat, bh - 1 - xs / lon], 1).astype(np.float32)
            ip = cv2.perspectiveTransform(bp.reshape(-1, 1, 2), np.linalg.inv(M)).reshape(-1, 2)
            cv2.polylines(vis, [ip.astype(np.int32)], False, (0, 0, 255), 3)
        adc, tgt, bstate = board_last()
        line1 = "#%d %s  cmd %s  path %s" % (self.idx, time.strftime("%H:%M:%S", time.localtime(now)),
                                             ("s%d l%d r%d" % (cmd.steering, cmd.left_speed, cmd.right_speed)) if cmd else "-",
                                             ("c=%+.2f b=%+.2f" % (traj.c, traj.b)) if (traj and traj.valid) else "INVALID")
        line2 = "solid %d dashed %d  steerADC %s->%s %s" % (counts.get("solid_line", 0), counts.get("dashed_line", 0), adc, tgt, bstate)
        cv2.rectangle(vis, (0, 0), (w, 40), (0, 0, 0), -1)
        cv2.putText(vis, line1, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        cv2.putText(vis, line2, (5, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        cv2.imwrite(OUT + "/frames/" + name, vis, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        dst = stamp(det.header) if det is not None else None
        tst = time.time()
        self.w.writerow([self.idx, "%.3f" % now, name, "%.3f" % ist, "%.3f" % dst if dst else "",
                         "%.0f" % ((now - dst) * 1000) if dst else "", counts.get("solid_line", 0), counts.get("dashed_line", 0),
                         sum(v for k, v in counts.items() if k not in COL), "%.3f" % best.get("solid_line", 0), "%.3f" % best.get("dashed_line", 0),
                         ";".join("%s:%d" % kv for kv in counts.items() if kv[0] not in COL),
                         int(bool(traj and traj.valid)), *(("%.4f" % traj.a, "%.4f" % traj.b, "%.4f" % traj.c, "%.3f" % traj.x_min_m,
                                                          "%.3f" % traj.x_max_m, "%.4f" % traj.fit_error_m) if traj else ("",) * 6),
                         "%.0f" % ((now - stamp(traj.header)) * 1000) if traj else "",
                         cmd.steering if cmd else "", cmd.left_speed if cmd else "", cmd.right_speed if cmd else "", adc, tgt, bstate])
        self.idx += 1
        if self.idx % 10 == 0: self.f.flush()

rclpy.init(); n = Logger()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
finally:
    n.f.close(); print("frames:", n.idx, "->", OUT)
```


---

## `python/4_tools/motor_fixed_test.py`

```python
"""고정 명령 모터 시험 (벤치용). 사용: python3 motor_fixed_test.py <steering> <pwm> <seconds>
s<steering>l<pwm>r<pwm> 을 20Hz 로 <seconds> 동안 보내고, 끝나면 s0l0r0 을 5번 보낸다. Ctrl+C 에도 정지 명령."""
import sys, time, serial
st, pwm, secs = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
s = serial.Serial(); s.port = "/dev/ttyACM0"; s.baudrate = 115200; s.timeout = 0.1; s.open()
cmd = ("s%dl%dr%d\n" % (st, pwm, pwm)).encode()
print(time.strftime("%H:%M:%S"), "send", cmd.strip().decode(), "for", secs, "s", flush=True)
t0 = time.time(); n = 0
try:
    while time.time() - t0 < secs:
        s.write(cmd); n += 1; time.sleep(0.05)
finally:
    for _ in range(5): s.write(b"s0l0r0\n"); time.sleep(0.05)
    print(time.strftime("%H:%M:%S"), "stop sent after %d commands (%.1fs)" % (n, time.time() - t0), flush=True)
    s.close()
```


---

## `python/4_tools/path_viewer.py`

```python
"""/detections(선) + /lane_detections(lane 폴리곤) + /lane_trajectory + /topic_control_signal 시각화. q/Esc 종료.
좌: 원본 + 선 마스크 + lane 영역 + IPM 사다리꼴 + 경로 역투영 / 우: BEV + 경로 곡선.
IPM/미터 변환 값은 lane_info_extractor_node 기본값과 동일하게 사용."""
import os, sys, threading
import cv2, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from interfaces_pkg.msg import DetectionArray, LaneTrajectory, MotionCommand

IMAGE_TOPIC = sys.argv[1] if len(sys.argv) > 1 else "/image_raw"
LANE_TOPIC = sys.argv[2] if len(sys.argv) > 2 else "/lane_detections"
CLASS = "lane"
IPM_SRC = np.float32([238.0, 316.0, 402.0, 313.0, 501.0, 476.0, 155.0, 476.0]).reshape(4, 2)
BASE_W, BASE_H, L_R, R_R, BEV_SCALE = 640.0, 480.0, 0.3, 0.7, 4.0
LAT_MPP, LON_MPP = 0.005, 0.01
# lane_info_extractor 와 같은 IPM 파라미터 파일이 있으면 그 값을 쓴다 (3번째 인자)
IPM_YAML = sys.argv[3] if len(sys.argv) > 3 else os.path.expanduser("~/jacapdi-docker/ipm_c920.yaml")
if os.path.isfile(IPM_YAML):
    import yaml
    _p = yaml.safe_load(open(IPM_YAML))["lane_info_extractor_node"]["ros__parameters"]
    IPM_SRC = np.float32(_p["ipm_src_points"]).reshape(4, 2)
    BASE_W, BASE_H = _p["ipm_src_base_width"], _p["ipm_src_base_height"]
    L_R, R_R, BEV_SCALE = _p["ipm_dst_left_ratio"], _p["ipm_dst_right_ratio"], _p["bev_height_scale"]
    LAT_MPP, LON_MPP = _p["lateral_meter_per_pixel"], _p["longitudinal_meter_per_pixel"]
COLORS = {}

class Viewer(Node):
    def __init__(self):
        super().__init__("path_viewer")
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.bridge, self.lock = CvBridge(), threading.Lock()
        self.img, self.det, self.traj, self.lane, self.cmd = None, None, None, None, None
        self.n_traj, self.n_valid = 0, 0
        self.create_subscription(Image, IMAGE_TOPIC, self.on_img, 10)
        self.create_subscription(DetectionArray, "/detections", self.on_det, q)
        self.create_subscription(LaneTrajectory, "/lane_trajectory", self.on_traj, q)
        self.create_subscription(DetectionArray, LANE_TOPIC, self.on_lane, q)
        self.create_subscription(MotionCommand, "/topic_control_signal", self.on_cmd, q)
    def on_lane(self, m):
        with self.lock: self.lane = m
    def on_cmd(self, m):
        with self.lock: self.cmd = m
    def on_img(self, m):
        with self.lock: self.img = self.bridge.imgmsg_to_cv2(m, "bgr8")
    def on_det(self, m):
        with self.lock: self.det = m
    def on_traj(self, m):
        with self.lock:
            self.traj = m; self.n_traj += 1; self.n_valid += int(m.valid)

def color(name):
    if name not in COLORS:
        rng = np.random.default_rng(abs(hash(name)) % 2**32); COLORS[name] = tuple(int(c) for c in rng.integers(60, 255, 3))
    return COLORS[name]

def render(v):
    with v.lock:
        img = None if v.img is None else v.img.copy(); det, traj, lane, cmd = v.det, v.traj, v.lane, v.cmd; nt, nv = v.n_traj, v.n_valid
    if img is None:
        return None
    h, w = img.shape[:2]
    overlay, sel = img.copy(), np.zeros((h, w), np.uint8)
    counts = {}
    if det is not None:
        for d in det.detections:
            counts[d.class_name] = counts.get(d.class_name, 0) + 1
            if not d.mask.data: continue
            pts = np.array([[int(p.x), int(p.y)] for p in d.mask.data], np.int32)
            cv2.fillPoly(overlay, [pts], color(d.class_name))
            cv2.putText(img, d.class_name, tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color(d.class_name), 1)
    if lane is not None:
        for d in lane.detections:
            if d.class_name == CLASS and d.mask.data:
                pts = np.array([[int(p.x), int(p.y)] for p in d.mask.data], np.int32)
                cv2.fillPoly(sel, [pts], 255)
                lo = overlay.copy(); cv2.fillPoly(lo, [pts], (0, 180, 0)); overlay = cv2.addWeighted(lo, 0.5, overlay, 0.5, 0)
                cv2.polylines(overlay, [pts], True, (0, 255, 0), 2)
    img = cv2.addWeighted(overlay, 0.45, img, 0.55, 0)
    src = IPM_SRC.copy(); src[:, 0] *= w / BASE_W; src[:, 1] *= h / BASE_H
    bev_h = int(round(h * BEV_SCALE))
    dst = np.float32([[round(w * L_R), 0], [round(w * R_R), 0], [round(w * R_R), bev_h - 1], [round(w * L_R), bev_h - 1]])
    M = cv2.getPerspectiveTransform(src, dst); Minv = np.linalg.inv(M)
    cv2.polylines(img, [src.astype(np.int32)], True, (0, 255, 255), 1)
    bev = cv2.cvtColor(cv2.warpPerspective(sel, M, (w, bev_h), flags=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
    status = "no trajectory yet"
    if traj is not None:
        status = "valid=%s pts=%d fit_err=%.3fm x=[%.2f,%.2f]m a=%.3f b=%.3f c=%.3f" % (
            traj.valid, traj.num_points, traj.fit_error_m, traj.x_min_m, traj.x_max_m, traj.a, traj.b, traj.c) if traj.valid else "valid=False"
        if traj.valid:
            xs = np.linspace(traj.x_min_m, traj.x_max_m, 60)
            ys = traj.a * xs**2 + traj.b * xs + traj.c
            bx = (w - 1) * 0.5 - ys / LAT_MPP; by = (bev_h - 1.0) - xs / LON_MPP
            bev_pts = np.stack([bx, by], 1).astype(np.float32)
            cv2.polylines(bev, [bev_pts.astype(np.int32)], False, (0, 0, 255), 6)
            img_pts = cv2.perspectiveTransform(bev_pts.reshape(-1, 1, 2), Minv).reshape(-1, 2)
            cv2.polylines(img, [img_pts.astype(np.int32)], False, (0, 0, 255), 3)
    bev_small = cv2.resize(bev, (int(w * h / bev_h), h))
    canvas = np.hstack([img, bev_small])
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 44), (0, 0, 0), -1)
    cmd_s = "none" if cmd is None else "s%d l%d r%d" % (cmd.steering, cmd.left_speed, cmd.right_speed)
    cv2.putText(canvas, "cmd=%s | %s" % (cmd_s, status), (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(canvas, "valid %d/%d | det: %s" % (nv, nt, ", ".join("%s:%d" % kv for kv in sorted(counts.items())) or "none"),
                (6, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return canvas

def main():
    rclpy.init(); v = Viewer()
    win = "path viewer (q/Esc)"; cv2.namedWindow(win, cv2.WINDOW_NORMAL); cv2.resizeWindow(win, 1400, 720)
    while True:
        for _ in range(5): rclpy.spin_once(v, timeout_sec=0.002)
        c = render(v)
        if c is not None: cv2.imshow(win, c)
        k = cv2.waitKey(15) & 0xFF
        if k in (ord("q"), 27): break
    cv2.destroyAllWindows(); v.destroy_node(); rclpy.try_shutdown()

if __name__ == "__main__":
    main()
```


---

## `python/4_tools/rate_probe.py`

```python
"""파이프라인 각 토픽의 실제 수신 Hz와 지연(수신시각 - header.stamp)을 5초간 측정."""
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from interfaces_pkg.msg import DetectionArray, LaneTrajectory, MotionCommand
rclpy.init(); n = Node("rate_probe")
q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
st = {}
def cb(name, stamped=True):
    def f(m):
        s = st.setdefault(name, {"n": 0, "lat": [], "valid": 0})
        s["n"] += 1
        if stamped:
            s["lat"].append((n.get_clock().now().nanoseconds - (m.header.stamp.sec * 10**9 + m.header.stamp.nanosec)) / 1e6)
        if hasattr(m, "valid"): s["valid"] += int(m.valid)
        if hasattr(m, "left_speed"): s.setdefault("cmds", {}).setdefault("s%d l%d r%d" % (m.steering, m.left_speed, m.right_speed), 0); s["cmds"]["s%d l%d r%d" % (m.steering, m.left_speed, m.right_speed)] += 1
    return f
n.create_subscription(Image, "/image_raw", cb("/image_raw"), 10)
n.create_subscription(DetectionArray, "/detections", cb("/detections"), q)
n.create_subscription(DetectionArray, "/lane_detections", cb("/lane_detections"), q)
n.create_subscription(LaneTrajectory, "/lane_trajectory", cb("/lane_trajectory"), q)
n.create_subscription(MotionCommand, "/topic_control_signal", cb("/topic_control_signal", False), q)
T = 5.0; end = time.time() + T
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
for k in ["/image_raw", "/detections", "/lane_detections", "/lane_trajectory", "/topic_control_signal"]:
    s = st.get(k)
    if not s: print("%-22s 0 msgs" % k); continue
    lat = sorted(s["lat"]); extra = ""
    if lat: extra += " latency median %.0f ms, max %.0f ms" % (lat[len(lat)//2], lat[-1])
    if k == "/lane_trajectory": extra += " valid %d/%d" % (s["valid"], s["n"])
    if "cmds" in s: extra += " cmds %s" % s["cmds"]
    print("%-22s %5.1f Hz%s" % (k, s["n"] / T, extra))
```


---

## `python/4_tools/run_monitor.py`

```python
# -*- coding: utf-8 -*-
"""start_ab.sh 가 같이 띄우는 기록기: 경로/명령/검출을 10 Hz 로 CSV 에 남긴다 (<기록폴더>/timeline.csv).
경로 유효, 계수 a b c, 조향/속도 명령, 실선/점선 검출 수, YOLO 결과 속도를 본다. 주행에는 관여하지 않는다."""
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from interfaces_pkg.msg import DetectionArray, LaneTrajectory, MotionCommand


def main():
    out = sys.argv[1]
    rclpy.init(); n = Node("ab_run_monitor")
    q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
    st = {"traj": None, "cmd": None, "det": (0, 0), "n_det": 0}
    n.create_subscription(LaneTrajectory, "/lane_trajectory", lambda m: st.__setitem__("traj", m), q)
    n.create_subscription(MotionCommand, "/topic_control_signal", lambda m: st.__setitem__("cmd", m), q)

    def on_det(m):
        s = sum(d.class_name == "solid_line" for d in m.detections)
        d_ = sum(d.class_name == "dashed_line" for d in m.detections)
        st["det"] = (s, d_); st["n_det"] += 1
    n.create_subscription(DetectionArray, "/detections", on_det, q)
    f = open(out + "/timeline.csv", "w", newline=""); w = csv.writer(f)
    w.writerow(["t", "traj_valid", "a", "b", "c", "steer", "speed", "n_solid", "n_dashed", "det_hz"])
    t0 = time.time(); last = t0; last_n = 0; hz = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.02)
            now = time.time()
            if now - last >= 0.1:
                hz = 0.8 * hz + 0.2 * ((st["n_det"] - last_n) / (now - last)); last_n = st["n_det"]; last = now
                t, c = st["traj"], st["cmd"]
                w.writerow(["%.2f" % (now - t0), int(bool(t and t.valid)),
                            "%.4f" % t.a if t else "", "%.4f" % t.b if t else "", "%.4f" % t.c if t else "",
                            c.steering if c else "", c.left_speed if c else "", st["det"][0], st["det"][1], "%.1f" % hz])
                f.flush()
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
```


---

## `python/4_tools/steer_sequence_test.py`

```python
"""벤치 조향 순서 시험: 명령 목록을 20Hz 로 각각 N초씩 보내고 마지막에 s0l0r0. 보드 메시지는 board_monitor 가 기록.
사용: python3 steer_sequence_test.py "s-7l0r0:5" "s0l1r1:5" "s7l0r0:5" "s0l1r1:5" """
import sys, time, serial
steps = [(a.split(":")[0], float(a.split(":")[1])) for a in sys.argv[1:]]
s = serial.Serial(); s.port = "/dev/ttyACM0"; s.baudrate = 115200; s.timeout = 0.1; s.open()
try:
    for cmd, secs in steps:
        print(time.strftime("%H:%M:%S"), "send", cmd, "for", secs, "s", flush=True)
        t0 = time.time()
        while time.time() - t0 < secs:
            s.write((cmd + "\n").encode()); time.sleep(0.05)
finally:
    for _ in range(5): s.write(b"s0l0r0\n"); time.sleep(0.05)
    print(time.strftime("%H:%M:%S"), "stop sent", flush=True); s.close()
```


---

## `python/4_tools/webcam_pub.py`

```python
"""USB 웹캠 -> sensor_msgs/Image 발행 (팀 코드 수정 없이 시험용).
사용: python3 webcam_pub.py [device] [topic] [width] [height]"""
import sys
import cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/video4"
topic = sys.argv[2] if len(sys.argv) > 2 else "/webcam/image_raw"
W = int(sys.argv[3]) if len(sys.argv) > 3 else 640
H = int(sys.argv[4]) if len(sys.argv) > 4 else 480

class WebcamPub(Node):
    def __init__(self):
        super().__init__("webcam_pub")
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, W); self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        self.cap.set(cv2.CAP_PROP_FPS, 30); self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise SystemExit("camera open failed: " + dev)
        self.pub = self.create_publisher(Image, topic, 10)  # yolov8_node 기본 QoS(reliable)와 호환
        self.bridge = CvBridge()
        self.create_timer(1.0 / 30, self.tick)
        self.get_logger().info("publishing %s from %s at %dx%d" % (topic, dev, W, H))

    def tick(self):
        ok, f = self.cap.read()
        if not ok:
            return
        msg = self.bridge.cv2_to_imgmsg(f, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "webcam"
        self.pub.publish(msg)

def main():
    rclpy.init(); n = WebcamPub()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.cap.release(); n.destroy_node(); rclpy.try_shutdown()

if __name__ == "__main__":
    main()
```


---

## `scripts/car_all.sh`

```bash
#!/usr/bin/env bash
# 한 방에: 준비 -> (엔터) -> 주행. 사용: ~/jacapdi-docker/car_all.sh [초]
source "$(dirname "$0")/car_common.sh"
"$(dirname "$0")/car_prep.sh" || exit 1
read -r -p $'\n차를 출발 위치에 놓고 엔터 = 출발, Ctrl+C = 취소 ' _
exec "$(dirname "$0")/car_go.sh" "${1:-$DRIVE_SEC}"
```


---

## `scripts/car_common.sh`

```bash
# 공통 설정 (car_*.sh 가 source). 여기 값만 바꾸면 된다.
ALGO="${ALGO:-F}"                                            # 경로 알고리즘 A/B/C/D/E/F (E = B + 정지선 무시 + 점선 민감 + 더 멀리 봄, F = E + 경로 튀면 0.5초 무시). 예전대로: ALGO=B CONF=0.5
CONF="${CONF:-0.3}"                                          # YOLO threshold (E 는 0.3: 점선 놓치지 않게. 실선은 E 안에서 0.5로 다시 거름)
TUNE="${TUNE:-1}"                                             # 1: 슬라이더로 조정 가능한 제어기(tunable_controller.py) 사용, 0: 팀 Stanley 그대로
if [ "$TUNE" = 1 ]; then CTRL_YAML="${CTRL_YAML:-$HOME/jacapdi-docker/drive_controller_tunable.yaml}"
else CTRL_YAML="${CTRL_YAML:-$HOME/jacapdi-docker/drive_controller.yaml}"; fi   # 마지막 성공 주행 설정 (PWM 240, 조향 7)
DRIVE_SEC="${DRIVE_SEC:-120}"                                # car_go 기본 주행 시간(초)
FW_HEX="${FW_HEX:-$HOME/jacapdi-live-car-2026-09-24/arduino/driving_team_cal_1009c/driving_team_cal_1009c.ino.hex}"
FW_NAME="${FW_NAME:-DRIVING_TEAM_CAL_1009C}"                  # 보드 부팅 메시지에 있어야 할 이름
ACM=$(ls /dev/ttyACM* 2>/dev/null | head -1); ACM=${ACM:-/dev/ttyACM0}   # 호스트의 아두이노 포트 (ttyACM0 이 아닐 수도 있음)
C=jacapdi-humble
D=$HOME/jacapdi-docker
AVR=$HOME/Downloads/arduino-1.8.19/hardware/tools/avr

say()  { printf '\033[1;32m[car]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[car]\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31m[car] %s\033[0m\n' "$*"; exit 1; }

port_users() {   # /dev/ttyACM0 를 잡고 있는 호스트 프로세스 (Arduino IDE 시리얼 모니터 등)
  for p in /proc/[0-9]*; do ls -l "$p/fd" 2>/dev/null | grep -q "$(basename "$ACM")" && echo "$(basename "$p") $(tr '\0' ' ' < "$p/cmdline" | cut -c1-60)"; done
}

send_stop() {    # 모든 송신기 끄고 s0l0r0 5번 (컨테이너 안에서)
  docker exec "$C" bash -c 'pkill -INT -f "timeout -s INT"; pkill -INT -f "serial_communication_pkg/serial_sender_node"; sleep 1.5;
    for p in $(pgrep -f "serial_communication_pkg/serial_sender_node"); do kill -9 $p; done' >/dev/null 2>&1
  docker exec "$C" python3 -c "
import serial, time
s = serial.Serial(); s.port = '/dev/ttyACM0'; s.baudrate = 115200; s.timeout = 0.1; s.dsrdtr = False; s.open()
for _ in range(5): s.write(b's0l0r0\n'); time.sleep(0.05)
s.close()" >/dev/null 2>&1 && say "정지 명령 s0l0r0 전송" || warn "정지 명령 전송 실패 (아두이노 연결 확인). 펌웨어가 0.3초 뒤 스스로 정지함"
}
```


---

## `scripts/car_flash.sh`

```bash
#!/usr/bin/env bash
# 아두이노에 주행 펌웨어 업로드 (필요할 때만). 사용: ~/jacapdi-docker/car_flash.sh
source "$(dirname "$0")/car_common.sh"
[ -e "$ACM" ] || die "아두이노($ACM) 없음. USB 연결 확인"
[ -f "$FW_HEX" ] || die "펌웨어 파일 없음: $FW_HEX"
docker exec "$C" bash -c 'pkill -9 -f "board_monitor.py"; pkill -INT -f serial_sender_node' >/dev/null 2>&1 || true
sleep 1
U=$(port_users); [ -z "$U" ] || die "포트를 쓰는 프로그램이 있음 (Arduino IDE 시리얼 모니터 닫기): $U"
say "업로드: $FW_HEX"
"$AVR/bin/avrdude" -C"$AVR/etc/avrdude.conf" -patmega2560 -cwiring -P$ACM -b115200 -D -Uflash:w:"$FW_HEX":i 2>&1 \
  | grep -E "bytes of flash verified|error|mismatch" || die "업로드 실패"
sleep 2
python3 -c "
import serial, time
s = serial.Serial('$ACM', 115200, timeout=0.3); t = time.time(); b = b''
while time.time() - t < 3: b += s.read(256)
s.close(); print(b.decode(errors='replace').strip().splitlines()[:2])"
```


---

## `scripts/car_go.sh`

```bash
#!/usr/bin/env bash
# 모터 켜고 주행 (car_prep.sh 다음). 사용: ~/jacapdi-docker/car_go.sh [초]   Ctrl+C = 즉시 정지
# 주행 중 10 Hz 기록: <기록폴더>/drive_HHMMSS/{frames/*.png, frames.csv}  (LOG=0 이면 끔, RAW=1 이면 원본 PNG 도)
source "$(dirname "$0")/car_common.sh"
SEC="${1:-$DRIVE_SEC}"
docker ps -q -f name="$C" | grep -q . || die "컨테이너 없음. 먼저 ~/jacapdi-docker/car_prep.sh"
docker exec "$C" bash -c 'pgrep -f "ab_kit/algo/" >/dev/null' || die "파이프라인이 안 돌고 있음. 먼저 ~/jacapdi-docker/car_prep.sh"
RUN=$(cat /tmp/car_run_dir 2>/dev/null)
OUT=""
if [ "${LOG:-1}" = 1 ] && [ -n "$RUN" ]; then
  OUT="$RUN/drive_$(date +%H%M%S)"; mkdir -p "$OUT"
  docker exec -d "$C" bash -ic "python3 ~/jacapdi-docker/frame_logger.py $OUT 10 ${RAW:-0} > $OUT/logger.log 2>&1"
  say "10 Hz 기록 시작: $OUT"
fi
stop_logger() { [ -n "$OUT" ] || return 0; docker exec "$C" bash -c 'pkill -INT -f frame_logger.py' >/dev/null 2>&1; sleep 1.5
  say "기록: $(ls "$OUT/frames" 2>/dev/null | wc -l)장, $(du -sh "$OUT" 2>/dev/null | cut -f1) -> $OUT  (frames.csv)"; }
stop_now() { trap - INT TERM; echo; warn "정지 중..."; send_stop; stop_logger; [ -n "$RUN" ] && grep "FAULT:" "$RUN/board.log" 2>/dev/null | tail -2; exit 0; }
trap stop_now INT TERM
if [ "$TUNE" = 1 ]; then
  docker exec "$C" bash -c 'pgrep -f tunable_controller.py >/dev/null' || die "조정형 제어기가 안 돌고 있음. car_prep.sh 를 다시 실행"
  [ -n "$OUT" ] && cp "$D/tune/tune.json" "$OUT/tune_at_start.json"
  say "출발: ${SEC}초 (슬라이더 값: $(python3 -c "import json;d=json.load(open('$D/tune/tune.json'));print('Stanley %.2f, 방향 %.2f, PWM %s'%(d['stanley_gain'],d['heading_gain'],d['pwm_by_step']))"))  Ctrl+C = 정지"
else
  say "출발: ${SEC}초 ($(grep -E 'speed_command|max_steering' "$CTRL_YAML" | tr -s ' ' | tr '\n' ' '))  Ctrl+C = 정지"
fi
docker exec "$C" bash -c "DRIVE_SEC=$SEC ~/jacapdi-docker/timed_drive.sh" 2>&1 | grep -vE "RCLError|rcl_shutdown|Traceback|File |^\s|KeyboardInterrupt|ros2run" &
wait $!
stop_now
```


---

## `scripts/car_lap.sh`

```bash
#!/usr/bin/env bash
# 한 방에 한 바퀴: 준비 -> 차선 인식 확인 -> 5초 카운트다운 -> LAP_SEC 초 주행 -> 자동 정지 (10 Hz 기록 포함)
# 사용: ~/jacapdi-docker/car_lap.sh            (기본 75초 = 9/27 한 바퀴 주행 시간 기준)
#       ~/jacapdi-docker/car_lap.sh 70         (한 바퀴 시간을 초로 지정)
# 차는 바퀴 위치를 모른다(엔코더/GPS 없음). "한 바퀴"는 시간으로 정한다. Ctrl+C = 즉시 정지.
source "$(dirname "$0")/car_common.sh"
LAP_SEC="${1:-${LAP_SEC:-75}}"
"$D/car_prep.sh" || exit 1

say "차선 인식 확인 (경로 valid 80% 이상이 될 때까지, 최대 30초). 차를 2차선 출발 위치에 놓으세요"
ok=0
for i in $(seq 6); do
  line=$(docker exec "$C" bash -ic 'python3 ~/jacapdi-docker/rate_probe.py 2>/dev/null' 2>/dev/null | grep "/lane_trajectory")
  v=$(echo "$line" | grep -o "valid [0-9]*/[0-9]*" | cut -d' ' -f2)
  n=${v%/*}; m=${v#*/}
  if [ -n "$m" ] && [ "$m" -gt 0 ] && [ $((100 * n / m)) -ge 80 ]; then ok=1; say "   경로 valid $v - 차선 인식 OK"; break; fi
  warn "   경로 valid ${v:-0/0} - 차선이 잘 안 보임, 다시 확인"
done
[ "$ok" = 1 ] || die "차선이 안 보여서 출발하지 않음. 차를 2차선에 놓고: ~/jacapdi-docker/car_go.sh $LAP_SEC"

trap 'echo; warn "출발 취소"; exit 0' INT
for s in 5 4 3 2 1; do printf '\r\033[1;33m[car] %d초 뒤 출발 (Ctrl+C = 취소)\033[0m ' "$s"; sleep 1; done; echo
trap - INT
exec "$D/car_go.sh" "$LAP_SEC"
```


---

## `scripts/car_prep.sh`

```bash
#!/usr/bin/env bash
# 전원 켠 뒤 한 번: 장치 확인 -> 컨테이너 -> 파이프라인(웹캠/YOLO/경로/Stanley, 모터 제외) -> 보드 모니터 -> 화면 -> 준비 확인
# 사용: ~/jacapdi-docker/car_prep.sh          (ALGO=B CONF=0.5 기본, 바꾸려면 ALGO=C CONF=0.3 ~/jacapdi-docker/car_prep.sh)
source "$(dirname "$0")/car_common.sh"
say "1/6 장치 확인 (웹캠 C920, 아두이노) - 최대 30초 대기"
for i in $(seq 30); do
  CAM=$(for v in /sys/class/video4linux/video*; do [ "$(cat $v/index 2>/dev/null)" = 0 ] && grep -q C920 $v/name && echo /dev/$(basename $v) && break; done)
  ACM=$(ls /dev/ttyACM* 2>/dev/null | head -1)
  [ -n "$CAM" ] && [ -n "$ACM" ] && break; sleep 1
done
[ -n "$CAM" ] || die "웹캠 C920 이 안 보임. USB 확인"
[ -n "$ACM" ] || die "아두이노가 안 보임. USB 확인"
docker rm -f "$C" >/dev/null 2>&1 || true          # 이전 실행의 보드 모니터/송신기도 함께 정리
pkill -f "jacapdi-docker/board_monitor.py" 2>/dev/null || true
sleep 1
U=$(port_users); [ -z "$U" ] || die "아두이노 포트를 쓰는 프로그램이 있음 (Arduino IDE 시리얼 모니터 닫기): $U"
say "   웹캠 $CAM, 아두이노 $ACM (컨테이너 안에서는 /dev/ttyACM0)"

say "2/6 컨테이너 새로 만들기"
docker rm -f "$C" >/dev/null 2>&1 || true
"$D/run.sh" true 2>&1 | grep started || die "컨테이너 시작 실패"
docker exec -u root "$C" bash -c 'for d in /dev/video*; do chgrp video $d; chmod 660 $d; done; chgrp dialout /dev/ttyACM0; chmod 660 /dev/ttyACM0'

say "3/6 파이프라인 시작 (ALGO=$ALGO CONF=$CONF, $(basename "$CTRL_YAML")) - 모터는 아직 안 켬"
T0=$(date +%s)
docker exec -d "$C" bash -ic "CTRL_YAML=$CTRL_YAML ALGO=$ALGO CONF=$CONF bash ~/jacapdi-docker/ab_kit/start_ab.sh > /tmp/start_ab.log 2>&1"
RUN=""
for i in $(seq 20); do
  RUN=$(ls -td "$D"/ab_runs/*_"${ALGO}"_conf"${CONF}" 2>/dev/null | head -1)
  [ -n "$RUN" ] && [ "$(stat -c %Y "$RUN")" -ge "$T0" ] && break; RUN=""; sleep 1
done
[ -n "$RUN" ] || die "파이프라인 시작 실패: docker exec $C cat /tmp/start_ab.log"
echo "$RUN" > /tmp/car_run_dir

if [ "$TUNE" = 1 ]; then
  docker exec -d "$C" bash -ic "python3 ~/jacapdi-docker/tunable_controller.py $RUN > $RUN/tunable.log 2>&1"
  cp "$D/tune/tune.json" "$RUN/tune_at_start.json"
  say "   조정형 제어기 사용 (슬라이더: python3 ~/jacapdi-docker/tune_gui.py)"
fi
say "4/6 보드 모니터 (펌웨어 확인, FAULT 기록)"
docker exec -d "$C" bash -c "python3 ~/jacapdi-docker/board_monitor.py $RUN/board.log"
sleep 4
FW=$(grep -o "DRIVING_TEAM[^ ]*" "$RUN/board.log" | head -1)
ADC=$(grep -o "IDLE ADC=[0-9]*" "$RUN/board.log" | tail -1)
[ "$FW" = "$FW_NAME" ] && say "   펌웨어 $FW, 조향 $ADC" || warn "   펌웨어가 $FW_NAME 이 아님 (보드: ${FW:-응답없음}). 업로드: ~/jacapdi-docker/car_flash.sh"
A=${ADC#IDLE ADC=}   # 조향 가변저항 값이 정상 범위(351~583, 펌웨어 기준)인지 확인
if [ -n "$A" ] && { [ "$A" -lt 351 ] || [ "$A" -gt 583 ]; }; then
  printf '\033[1;31m[car]    !!! 조향 가변저항 값 %s = 비정상 (정상 391~543). 출발하면 펌웨어가 바로 모터를 끔 (adc_out_of_range).\n[car]    !!! 1023 = GND 선 빠짐/신호가 5V 에 닿음, 0 = 신호선 빠짐/GND 에 닿음. 배터리·USB 끄고 가변저항 배선 확인\033[0m\n' "$A"
fi

say "5/6 실시간 화면"
[ "${VIEW:-1}" = 1 ] && docker exec -d "$C" bash -ic 'python3 ~/jacapdi-docker/path_viewer.py /image_raw /lane_detections > /tmp/viewer.log 2>&1'

say "6/6 YOLO/경로 준비 대기 (보통 20~30초)"
for i in $(seq 60); do
  docker exec "$C" bash -c "grep -q 'frames=' $RUN/line_path.log 2>/dev/null && grep 'frames=' $RUN/line_path.log | tail -1 | grep -qv 'frames=0 '" && break; sleep 1
done
docker exec "$C" bash -ic 'python3 ~/jacapdi-docker/rate_probe.py 2>/dev/null | tail -3' 2>/dev/null | grep -vE "job control|terminal process group"
say "준비 완료. 기록 폴더: $RUN"
say "출발: ~/jacapdi-docker/car_go.sh   (기본 ${DRIVE_SEC}초, 예: car_go.sh 60)   비상정지: Ctrl+C 또는 ~/jacapdi-docker/car_stop.sh"
```


---

## `scripts/car_stop.sh`

```bash
#!/usr/bin/env bash
# 비상 정지 (모터 명령 끄기). 사용: ~/jacapdi-docker/car_stop.sh        전부 끄기(요약 저장): car_stop.sh all
source "$(dirname "$0")/car_common.sh"
docker ps -q -f name="$C" | grep -q . || die "컨테이너 없음"
send_stop
if [ "$1" = all ]; then
  docker exec "$C" bash -c 'kill -INT $(pgrep -f "bash /home/sean/jacapdi-docker/ab_kit/start_ab.sh" | head -1) 2>/dev/null; sleep 6;
    for p in $(pgrep -f "stanley_controller_node|tunable_controller.py|ab_kit/algo/|yolov8_node import|webcam_pub.py|run_monitor.py|board_monitor.py|path_viewer.py"); do kill -9 $p 2>/dev/null; done' >/dev/null 2>&1
  RUN=$(cat /tmp/car_run_dir 2>/dev/null); [ -f "$RUN/summary.txt" ] && head -8 "$RUN/summary.txt"
  say "파이프라인 종료"
fi
```


---

## `scripts/run.sh`

```bash
#!/usr/bin/env bash
# Start (or attach to) the jacapdi Humble container. Host Ubuntu 24.04 is untouched.
# Usage: ./run.sh            -> interactive shell
#        ./run.sh <cmd...>   -> run command in the running container
set -euo pipefail
NAME=jacapdi-humble
IMAGE=jacapdi/humble:latest
H=/home/sean

if [[ "$(docker inspect -f '{{.State.Running}}' $NAME 2>/dev/null)" != "true" ]]; then
  docker rm -f $NAME >/dev/null 2>&1 || true
  DEVS=()
  for d in /dev/video* /dev/ttyUSB*; do [[ -e $d ]] && DEVS+=(--device "$d"); done
  # 아두이노: USB 를 다시 꽂으면 ttyACM1 등으로 바뀔 수 있음 -> 컨테이너 안에서는 항상 /dev/ttyACM0 으로 보이게
  ACM=$(ls /dev/ttyACM* 2>/dev/null | head -1); [[ -n $ACM ]] && DEVS+=(--device "$ACM:/dev/ttyACM0")
  docker run -d --name $NAME \
    --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
    --network host --ipc host \
    -e DISPLAY="${DISPLAY:-:1}" -e XAUTHORITY=/tmp/.Xauthority -e QT_X11_NO_MITSHM=1 -e YOLO_CONFIG_DIR=/tmp/Ultralytics \
    -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${XAUTHORITY:-$H/.Xauthority}":/tmp/.Xauthority:ro \
    --group-add 44 --group-add 20 \
    --device-cgroup-rule='c 81:* rmw' --device-cgroup-rule='c 166:* rmw' --device-cgroup-rule='c 188:* rmw' \
    "${DEVS[@]}" \
    -v $H/team_ws:$H/team_ws \
    -v $H/sim-assets:$H/sim-assets \
    -v $H/models:$H/models \
    -v $H/jacapdi-transfer-2026-09-24:$H/jacapdi-transfer-2026-09-24 \
    -v $H/jacapdi-live-car-2026-09-24:$H/jacapdi-live-car-2026-09-24 \
    -v $H/jacapdi-docker:$H/jacapdi-docker \
    $IMAGE sleep infinity >/dev/null
  echo "started $NAME (devices: ${DEVS[*]:-none})"
fi
if [[ $# -gt 0 ]]; then
  exec docker exec -i $NAME bash -ic "$*"
else
  exec docker exec -it $NAME bash
fi
```


---

## `scripts/start_ab.sh`

```bash
#!/usr/bin/env bash
# 실차 A/B 시험 파이프라인 (도커 컨테이너 안에서 실행). 기존 start_pipeline.sh 와 같은 구성에
# 경로 알고리즘(ALGO)과 YOLO threshold(CONF)만 골라 넣는다.
#   웹캠 -> YOLO(best.pt, threshold=CONF) -> 경로 생성(ALGO) -> Stanley(drive_controller.yaml)
# serial_sender_node 는 여기서 실행하지 않는다 (기존과 같음. 모터는 따로 켠다).
#
# 사용 예
#   ALGO=B CONF=0.5 bash ~/jacapdi-docker/ab_kit/start_ab.sh
#   ALGO=A CONF=0.3 RECORD=1 bash ~/jacapdi-docker/ab_kit/start_ab.sh     # 결과 토픽 기록
#   ALGO=C RECORD=2 bash ~/jacapdi-docker/ab_kit/start_ab.sh              # + 카메라 영상 기록 (1분 약 1.6 GB)
# ALGO: A 원본 / B 점선 대체 / C B+커브 보정 / D C+안정화      CONF 기본 0.5
# 종료: Ctrl+C -> 요약 출력 + ~/jacapdi-docker/ab_runs/results.csv 에 한 줄 추가
set -e
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ALGO="${ALGO:-A}"; CONF="${CONF:-0.5}"; RECORD="${RECORD:-0}"
case "$ALGO" in
  A) F=A_orig.py ;; B) F=B_dashed_fallback.py ;; C) F=C_curve_offset.py ;; D) F=D_stable.py ;; E) F=E_curve_cross.py ;; F) F=F_outlier_hold.py ;;
  *) echo "ALGO 는 A, B, C, D, E 중 하나"; exit 1 ;;
esac
source /opt/ros/humble/setup.bash
source ~/team_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1 YOLO_CONFIG_DIR=/tmp/Ultralytics PYTHONUNBUFFERED=1
D=~/jacapdi-docker
TAG="$(date +%m%d_%H%M%S)_${ALGO}_conf${CONF}"
L="$D/ab_runs/$TAG"; mkdir -p "$L"
VENV_PY=~/.venvs/jacapdi/bin/python3
CTRL="${CTRL_YAML:-$D/drive_controller.yaml}"
DEV=${WEBCAM_DEV:-$(for v in /sys/class/video4linux/video*; do [ "$(cat $v/index)" = 0 ] && grep -q C920 $v/name && echo /dev/$(basename $v) && break; done)}
[ -n "$DEV" ] || { echo "C920 없음 (WEBCAM_DEV=/dev/videoN 로 지정 가능)"; exit 1; }
if ros2 node list 2>/dev/null | grep -qE "serial_sender|line_path_node|yolov8_node"; then
  echo "이미 실행 중인 노드가 있음 (serial_sender/line_path/yolov8) - 먼저 끄고 다시"; ros2 node list; exit 1
fi
{
  echo "tag=$TAG"; echo "algo=$ALGO ($F) md5=$(md5sum "$KIT/algo/$F" | cut -d' ' -f1)"; echo "conf=$CONF"
  echo "controller=$CTRL"; grep -E "speed_command|max_steering_step|stanley_gain|heading_gain" "$CTRL" | sed 's/^ */  /'
  echo "ipm=$D/ipm_c920.yaml"; head -1 "$D/ipm_c920.yaml"; echo "webcam=$DEV"; date
} > "$L/run_info.txt"
cat "$L/run_info.txt"

PIDS=()
python3 "$D/webcam_pub.py" "$DEV" /image_raw 640 480 > "$L/webcam.log" 2>&1 & PIDS+=($!)
sleep 2
"$VENV_PY" -c 'from camera_perception_pkg.yolov8_node import main; main()' --ros-args \
  -p model:=$HOME/models/best.pt -p device:=cuda:0 -p imgsz:=640 -p frame_skip:=1 \
  -p half:=true -p threshold:=$CONF -p image_topic:=/image_raw -p image_reliability:=1 -p use_sim_time:=false \
  > "$L/yolo.log" 2>&1 & PIDS+=($!)
python3 "$KIT/algo/$F" --ros-args -p use_sim_time:=false \
  -p mode:=solid_follow -p online_width:=true -p hold_last_sec:=${HOLD_LAST_SEC:-1.0} \
  -p min_rows:=0.06 -p max_gap:=0.40 > "$L/line_path.log" 2>&1 & PIDS+=($!)
ros2 run decision_making_pkg stanley_controller_node --ros-args --params-file "$CTRL" > "$L/stanley.log" 2>&1 & PIDS+=($!)
python3 "$KIT/run_monitor.py" "$L" > "$L/monitor.log" 2>&1 & PIDS+=($!)
if [ "$RECORD" -ge 1 ]; then
  TOPICS="/detections /lane_trajectory /lane_detections /topic_control_signal"
  [ "$RECORD" -ge 2 ] && TOPICS="$TOPICS /image_raw"
  ros2 bag record -o "$L/bag" $TOPICS > "$L/bag.log" 2>&1 & PIDS+=($!)
  echo "rosbag 기록: $L/bag ($TOPICS)"
fi

finish() {
  trap - INT TERM
  echo; echo "== 종료 중 ($TAG)"
  kill -INT "${PIDS[@]}" 2>/dev/null || true
  sleep 2
  kill -TERM "${PIDS[@]}" 2>/dev/null || true
  python3 "$KIT/summarize_run.py" "$L" | tee "$L/summary.txt"
  echo "기록 폴더: $L"
  echo "결과 표에 사람이 적을 것: 완주 여부 / 멈춘 곳 / 선 밟음 / 메모 -> ~/jacapdi-docker/ab_kit/시험기록표.csv"
  exit 0
}
trap finish INT TERM
echo "== 시작: ALGO=$ALGO CONF=$CONF (기록 $L). 모터는 따로 켠다. 끝낼 때 Ctrl+C"
wait
```


---

## `scripts/timed_drive.sh`

```bash
#!/usr/bin/env bash
# 시간 제한 주행: serial_sender 를 DRIVE_SEC(+부팅 대기) 동안만 실행하고, 끝나면 s0l0r0 을 여러 번 보낸다.
source /opt/ros/humble/setup.bash; source ~/team_ws/install/setup.bash; export ROS_LOCALHOST_ONLY=1
PORT=${PORT:-/dev/ttyACM0}; DRIVE_SEC=${DRIVE_SEC:-5}; BOOT_SEC=3
L=/tmp/pipeline; mkdir -p $L
# 주행 중 명령 기록
python3 - > $L/drive_cmds.log 2>&1 <<PY &
import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from interfaces_pkg.msg import MotionCommand, LaneTrajectory
rclpy.init(); n = Node("drive_recorder"); q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
t0 = time.time(); c = {}; tv = [0, 0]
def cb(m):
    if time.time() - t0 < $BOOT_SEC: return
    k = "s%d l%d r%d" % (m.steering, m.left_speed, m.right_speed); c[k] = c.get(k, 0) + 1
def tr(m):
    if time.time() - t0 < $BOOT_SEC: return
    tv[0] += 1; tv[1] += int(m.valid)
n.create_subscription(MotionCommand, "/topic_control_signal", cb, q); n.create_subscription(LaneTrajectory, "/lane_trajectory", tr, q)
while time.time() - t0 < $BOOT_SEC + $DRIVE_SEC: rclpy.spin_once(n, timeout_sec=0.05)
print("commands during drive window:", c); print("trajectory valid %d/%d" % (tv[1], tv[0]))
PY
REC=$!
echo "$(date +%T) serial_sender start ($PORT), drive ${DRIVE_SEC}s after ${BOOT_SEC}s boot"
timeout -s INT $((BOOT_SEC + DRIVE_SEC)) ros2 run serial_communication_pkg serial_sender_node --ros-args \
  -p port:=$PORT -p baud_rate:=115200 -p sub_topic:=/topic_control_signal -p use_sim_time:=false > $L/serial.log 2>&1
echo "$(date +%T) serial_sender stopped (exit $?)"
pkill -f "ros-args -p port:=$PORT" 2>/dev/null
sleep 0.5
python3 -c "
import serial, time
s = serial.Serial('$PORT', 115200, timeout=0.2)
for _ in range(5): s.write(b's0l0r0\n'); time.sleep(0.05)
time.sleep(0.3); print('stop sent; board says:', repr(s.read(512).decode(errors='replace')))
s.close()"
wait $REC; cat $L/drive_cmds.log; tail -3 $L/serial.log
```


---

## `config/drive_controller_tunable.yaml`

```yaml
# tunable mode: team Stanley output is redirected (unused); tunable_controller.py publishes /topic_control_signal
# 지면 주행 설정 (사용자 요청 2026-09-24): 벤치 설정 대비 PWM 6배(40->240), 조향 +-1 -> +-7 (펌웨어 최대).
# PWM 240 은 실차에서 검증되지 않은 값. 경로 소실/0.3s 타임아웃 시 정지.
stanley_controller_node:
  ros__parameters:
    use_sim_time: false
    sub_trajectory_topic: /lane_trajectory
    pub_topic: /topic_control_signal_team_unused
    left_speed_command: 240
    right_speed_command: 240
    lane_unavailable_speed: 0
    lane_timeout_sec: 0.3
    max_steering_step: 7
    steering_sign: -1.0
    vehicle_speed_mps: 0.5
    stanley_gain: 0.8
    heading_gain: 0.1
```


---

## `config/ipm_c920.yaml`

```yaml
# 2026-09-24 07:29:20 calib_pitch.py: f=560 h=0.607 pitch=17.00deg (rows 3837, lane width 0.847m). 전방 0.84~1.40m, 좌우 +-0.8m.
lane_info_extractor_node:
  ros__parameters:
    use_sim_time: false
    sub_detection_topic: /lane_detections
    drivable_class_name: lane
    ipm_src_points:
    - 24.5
    - 303.2
    - 615.5
    - 303.2
    - 776.8
    - 431.2
    - -136.8
    - 431.2
    ipm_src_base_width: 640.0
    ipm_src_base_height: 480.0
    ipm_dst_left_ratio: 0.3
    ipm_dst_right_ratio: 0.7
    bev_height_scale: 4.0
    lateral_meter_per_pixel: 0.00625
    longitudinal_meter_per_pixel: 0.0002918
    ego_x_px: -1.0
```


---

## `config/tune_current.json`

```json
{
 "stanley_gain": 0.8,
 "heading_gain": 0.25,
 "vehicle_speed_mps": 0.5,
 "curvature_gain": 0.05,
 "softening_speed_mps": 0.15,
 "front_axle_offset_m": 0.1,
 "max_steer_deg": 18.0,
 "max_steering_step": 7,
 "steering_sign": -1.0,
 "steer_trim": 0,
 "path_offset_m": 0.02,
 "lane_timeout_sec": 0.6,
 "pwm_by_step": [
  255,
  255,
  255,
  255,
  255,
  255,
  220,
  200
 ],
 "inside_bias_m": -0.03,
 "steer_rate": 1,
 "lane_guard_m": 0.1,
 "motor_balance": 2,
 "stall_boost_pwm": 255,
 "lane_guard_left_m": 0.18,
 "lane_guard_push": 2,
 "inside_bias_right_m": -0.13
}
```


---

## `config/tune_default.json`

```json
{
 "stanley_gain": 0.8,
 "heading_gain": 0.1,
 "vehicle_speed_mps": 0.5,
 "curvature_gain": 0.0,
 "softening_speed_mps": 0.15,
 "front_axle_offset_m": 0.0,
 "max_steer_deg": 20.0,
 "max_steering_step": 7,
 "steering_sign": -1.0,
 "steer_trim": 0,
 "path_offset_m": 0.0,
 "lane_timeout_sec": 0.3,
 "pwm_by_step": [
  240,
  240,
  240,
  240,
  240,
  240,
  240,
  240
 ],
 "inside_bias_m": 0.0,
 "steer_rate": 0,
 "lane_guard_m": 0.0
}
```


---

## `config/presets/preset_1009_best.json`

```json
{
 "stanley_gain": 0.8,
 "heading_gain": 0.25,
 "vehicle_speed_mps": 0.5,
 "curvature_gain": 0.2,
 "softening_speed_mps": 0.15,
 "front_axle_offset_m": 0.1,
 "max_steer_deg": 22.0,
 "max_steering_step": 7,
 "steering_sign": -1.0,
 "steer_trim": 0,
 "path_offset_m": 0.05,
 "lane_timeout_sec": 0.6,
 "pwm_by_step": [
  255,
  255,
  255,
  255,
  255,
  240,
  225,
  205
 ],
 "inside_bias_m": 0.03,
 "steer_rate": 1,
 "lane_guard_m": 0.1,
 "motor_balance": 2,
 "stall_boost_pwm": 255,
 "lane_guard_left_m": 0.18
}
```


---

## `config/presets/preset_1009_best2.json`

```json
{
 "stanley_gain": 0.8,
 "heading_gain": 0.25,
 "vehicle_speed_mps": 0.5,
 "curvature_gain": 0.1,
 "softening_speed_mps": 0.15,
 "front_axle_offset_m": 0.1,
 "max_steer_deg": 18.0,
 "max_steering_step": 7,
 "steering_sign": -1.0,
 "steer_trim": 0,
 "path_offset_m": 0.05,
 "lane_timeout_sec": 0.6,
 "pwm_by_step": [
  255,
  255,
  255,
  255,
  255,
  240,
  225,
  205
 ],
 "inside_bias_m": 0.03,
 "steer_rate": 1,
 "lane_guard_m": 0.1,
 "motor_balance": 2,
 "stall_boost_pwm": 255,
 "lane_guard_left_m": 0.18,
 "lane_guard_push": 2
}
```


---

## `config/presets/preset_1009_best3.json`

```json
{
 "stanley_gain": 0.8,
 "heading_gain": 0.25,
 "vehicle_speed_mps": 0.5,
 "curvature_gain": 0.05,
 "softening_speed_mps": 0.15,
 "front_axle_offset_m": 0.1,
 "max_steer_deg": 18.0,
 "max_steering_step": 7,
 "steering_sign": -1.0,
 "steer_trim": 0,
 "path_offset_m": 0.02,
 "lane_timeout_sec": 0.6,
 "pwm_by_step": [
  255,
  255,
  255,
  255,
  255,
  240,
  225,
  205
 ],
 "inside_bias_m": -0.03,
 "steer_rate": 1,
 "lane_guard_m": 0.1,
 "motor_balance": 2,
 "stall_boost_pwm": 255,
 "lane_guard_left_m": 0.18,
 "lane_guard_push": 2
}
```


---

## `config/presets/preset_1009_v3.json`

```json
{
 "stanley_gain": 0.8,
 "heading_gain": 0.25,
 "vehicle_speed_mps": 0.5,
 "curvature_gain": 0.2,
 "softening_speed_mps": 0.15,
 "front_axle_offset_m": 0.1,
 "max_steer_deg": 18.0,
 "max_steering_step": 7,
 "steering_sign": -1.0,
 "steer_trim": 0,
 "path_offset_m": 0.0,
 "lane_timeout_sec": 0.6,
 "pwm_by_step": [
  255,
  255,
  255,
  255,
  255,
  240,
  240,
  235
 ],
 "inside_bias_m": 0.06,
 "steer_rate": 1,
 "lane_guard_m": 0.17,
 "motor_balance": 0
}
```


---

## `config/presets/preset_best_20261009_1455.json`

```json
{
 "inside_bias_m": -0.03,
 "steer_rate": 1,
 "lane_guard_m": 0.1,
 "motor_balance": 2,
 "stall_boost_pwm": 255,
 "lane_guard_left_m": 0.18,
 "lane_guard_push": 2,
 "stanley_gain": 0.8,
 "heading_gain": 0.25,
 "vehicle_speed_mps": 0.5,
 "curvature_gain": 0.05,
 "softening_speed_mps": 0.15,
 "front_axle_offset_m": 0.1,
 "max_steer_deg": 18.0,
 "max_steering_step": 7,
 "steering_sign": -1.0,
 "steer_trim": 0,
 "path_offset_m": 0.02,
 "lane_timeout_sec": 0.6,
 "pwm_by_step": [
  255,
  255,
  255,
  255,
  255,
  240,
  225,
  205
 ],
 "inside_bias_right_m": 0.0
}
```


---

## `config/presets/preset_curve_aggressive.json`

```json
{
 "stanley_gain": 0.8,
 "heading_gain": 0.5,
 "vehicle_speed_mps": 0.5,
 "curvature_gain": 0.3,
 "softening_speed_mps": 0.15,
 "front_axle_offset_m": 0.3,
 "max_steer_deg": 16.0,
 "max_steering_step": 7,
 "steering_sign": -1.0,
 "steer_trim": 0,
 "path_offset_m": 0.0,
 "lane_timeout_sec": 0.3,
 "pwm_by_step": [
  240,
  240,
  240,
  240,
  255,
  240,
  240,
  215
 ],
 "inside_bias_m": 0.08
}
```


---

## `config/presets/preset_curve_mid.json`

```json
{
 "stanley_gain": 0.8,
 "heading_gain": 0.25,
 "vehicle_speed_mps": 0.5,
 "curvature_gain": 0.1,
 "softening_speed_mps": 0.15,
 "front_axle_offset_m": 0.1,
 "max_steer_deg": 18.0,
 "max_steering_step": 7,
 "steering_sign": -1.0,
 "steer_trim": 0,
 "path_offset_m": 0.0,
 "lane_timeout_sec": 0.6,
 "pwm_by_step": [
  255,
  255,
  255,
  255,
  255,
  240,
  220,
  200
 ],
 "inside_bias_m": 0.03,
 "steer_rate": 1,
 "lane_guard_m": 0.13
}
```


---

## `config/presets/preset_curve_mid_v1.json`

```json
{
 "stanley_gain": 0.8,
 "heading_gain": 0.25,
 "vehicle_speed_mps": 0.5,
 "curvature_gain": 0.1,
 "softening_speed_mps": 0.15,
 "front_axle_offset_m": 0.1,
 "max_steer_deg": 18.0,
 "max_steering_step": 7,
 "steering_sign": -1.0,
 "steer_trim": 0,
 "path_offset_m": 0.0,
 "lane_timeout_sec": 0.3,
 "pwm_by_step": [
  240,
  240,
  240,
  240,
  255,
  240,
  220,
  200
 ],
 "inside_bias_m": 0.0,
 "steer_rate": 1,
 "lane_guard_m": 0.13
}
```


---

## `python/team_original/serial_sender_node.py`

```python
import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from interfaces_pkg.msg import MotionCommand
from .lib import protocol_convert_func_lib as PCFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = "topic_control_signal"

# 아두이노 장치 이름 (ls /dev/ttyA* 명령을 터미널 창에 입력하여 확인)
PORT='/dev/ttyACM0'

# Arduino driving.ino uses Serial.begin(115200)
BAUD_RATE=115200
#----------------------------------------------

class SerialSenderNode(Node):
  def __init__(self, sub_topic=SUB_TOPIC_NAME):
    super().__init__('serial_sender_node')
    
    self.declare_parameter('sub_topic', sub_topic)
    self.declare_parameter('port', PORT)
    self.declare_parameter('baud_rate', BAUD_RATE)
    
    self.sub_topic = self.get_parameter('sub_topic').get_parameter_value().string_value
    self.port = self.get_parameter('port').get_parameter_value().string_value
    self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
    
    self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
    time.sleep(1)
    
    qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, 
                             history=QoSHistoryPolicy.KEEP_LAST, 
                             durability=QoSDurabilityPolicy.VOLATILE, 
                             depth=1)
    
    self.subscription = self.create_subscription(MotionCommand, self.sub_topic, self.data_callback, qos_profile)

  def data_callback(self, msg):
    steering = msg.steering
    left_speed = msg.left_speed
    right_speed = msg.right_speed

    serial_msg =  PCFL.convert_serial_message(steering, left_speed, right_speed)
    self.ser.write(serial_msg.encode())

  def stop(self):
    message = PCFL.convert_serial_message(0, 0, 0)
    self.ser.write(message.encode())
    self.ser.close()

def main(args=None):
  rclpy.init(args=args)
  node = SerialSenderNode()
  try:
      rclpy.spin(node)
      
  except KeyboardInterrupt:
      print("\n\nshutdown\n\n")
      steering = 0
      left_speed = 0
      right_speed = 0
      message = PCFL.convert_serial_message(steering, left_speed, right_speed)
      node.ser.write(message.encode())
      pass
    
  finally:
    node.stop()
    print('closed')
    
  node.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
```


---

## `python/team_original/yolov8_node.py`

```python
# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import os
from typing import List, Dict

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

from cv_bridge import CvBridge

from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints
from torch import cuda

from sensor_msgs.msg import Image
from interfaces_pkg.msg import Point2D
from interfaces_pkg.msg import BoundingBox2D
from interfaces_pkg.msg import Mask
from interfaces_pkg.msg import KeyPoint2D
from interfaces_pkg.msg import KeyPoint2DArray
from interfaces_pkg.msg import Detection
from interfaces_pkg.msg import DetectionArray

from std_srvs.srv import SetBool


class Yolov8Node(LifecycleNode):

    def __init__(self, **kwargs) -> None:
        super().__init__("yolov8_node", **kwargs)
        
        #---------------Variable Setting---------------
        # 딥러닝 모델 pt 파일명 작성
        #self.declare_parameter("model", "yolov8m.pt")
        # self.declare_parameter("model", "best.pt")
        # 원본 기본값: /home/minuk/ros2_ws/src/camera_perception_pkg/best_0610.pt
        self.declare_parameter("model", os.path.expanduser("~/models/best.pt"))
        
        # 추론 하드웨어 선택 (cpu / gpu) 
        # self.declare_parameter("device", "cpu")
        self.declare_parameter("device", "cuda:0")
        #----------------------------------------------
        
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("imgsz", 320)
        self.declare_parameter("frame_skip", 3)
        self.declare_parameter("half", True)
        self.declare_parameter("enable", True)
        self.declare_parameter("image_topic", "image_raw")
        # self.declare_parameter("image_topic", "image_raw_resized")
        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.BEST_EFFORT)
        self.frame_count = 0

        self.get_logger().info('Yolov8Node created')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Configuring {self.get_name()}')

        self.model = self.get_parameter(
            "model").get_parameter_value().string_value

        self.device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.imgsz = self.get_parameter(
            "imgsz").get_parameter_value().integer_value

        self.frame_skip = self.get_parameter(
            "frame_skip").get_parameter_value().integer_value

        self.half = self.get_parameter(
            "half").get_parameter_value().bool_value

        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value

        self.image_topic = self.get_parameter(
            "image_topic").get_parameter_value().string_value

        self.reliability = self.get_parameter(
            "image_reliability").get_parameter_value().integer_value

        self.image_qos_profile = QoSProfile(
            reliability=self.reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self._pub = self.create_lifecycle_publisher(
            DetectionArray, "detections", 10)
        self._srv = self.create_service(
            SetBool, "enable", self.enable_cb
        )
        self.cv_bridge = CvBridge()

        return TransitionCallbackReturn.SUCCESS

    def enable_cb(self, request, response):
        self.enable = request.data
        response.success = True
        return response

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Activating {self.get_name()}')

        try:
            self.yolo = YOLO(self.model)  # 모델 로딩
            self.yolo.fuse()
        except FileNotFoundError:
            self.get_logger().error(f"Error: Model file '{self.model}' not found!")
            return TransitionCallbackReturn.FAILURE
        except Exception as e:
            self.get_logger().error(f"Error while loading model '{self.model}': {str(e)}")
            return TransitionCallbackReturn.FAILURE

        # subs
        self._sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_cb,
            self.image_qos_profile
        )

        super().on_activate(state)

        return TransitionCallbackReturn.SUCCESS


    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Deactivating {self.get_name()}')

        del self.yolo
        if 'cuda' in self.device:
            self.get_logger().info("Clearing CUDA cache")
            cuda.empty_cache()

        self.destroy_subscription(self._sub)
        self._sub = None

        super().on_deactivate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f'Cleaning up {self.get_name()}')

        self.destroy_publisher(self._pub)

        del self.image_qos_profile

        return TransitionCallbackReturn.SUCCESS

    def parse_hypothesis(self, results: Results) -> List[Dict]:

        hypothesis_list = []

        box_data: Boxes
        for box_data in results.boxes:
            hypothesis = {
                "class_id": int(box_data.cls),
                "class_name": self.yolo.names[int(box_data.cls)],
                "score": float(box_data.conf)
            }
            hypothesis_list.append(hypothesis)

        return hypothesis_list

    def parse_boxes(self, results: Results) -> List[BoundingBox2D]:

        boxes_list = []

        box_data: Boxes
        for box_data in results.boxes:

            msg = BoundingBox2D()

            # get boxes values
            box = box_data.xywh[0]
            msg.center.position.x = float(box[0])
            msg.center.position.y = float(box[1])
            msg.size.x = float(box[2])
            msg.size.y = float(box[3])

            # append msg
            boxes_list.append(msg)

        return boxes_list

    def parse_masks(self, results: Results) -> List[Mask]:

        masks_list = []

        def create_point2d(x: float, y: float) -> Point2D:
            p = Point2D()
            p.x = x
            p.y = y
            return p

        mask: Masks
        for mask in results.masks:

            msg = Mask()

            msg.data = [create_point2d(float(ele[0]), float(ele[1]))
                        for ele in mask.xy[0].tolist()]
            msg.height = results.orig_img.shape[0]
            msg.width = results.orig_img.shape[1]

            masks_list.append(msg)

        return masks_list

    def parse_keypoints(self, results: Results) -> List[KeyPoint2DArray]:

        keypoints_list = []

        points: Keypoints
        for points in results.keypoints:

            msg_array = KeyPoint2DArray()

            if points.conf is None:
                continue

            for kp_id, (p, conf) in enumerate(zip(points.xy[0], points.conf[0])):

                if conf >= self.threshold:
                    msg = KeyPoint2D()

                    msg.id = kp_id + 1
                    msg.point.x = float(p[0])
                    msg.point.y = float(p[1])
                    msg.score = float(conf)

                    msg_array.data.append(msg)

            keypoints_list.append(msg_array)

        return keypoints_list

    def image_cb(self, msg: Image) -> None:
        self.frame_count += 1
        frame_skip = max(1, int(self.frame_skip))
        if self.frame_count % frame_skip != 0:
            return

        if self.enable:

            # convert image + predict
            # 시뮬 카메라(rgb8)는 bgr8로 변환, 그 밖은 원본 동작
            if msg.encoding == 'rgb8':
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
            results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.threshold,
                imgsz=self.imgsz,
                half=(self.half and 'cuda' in self.device),
                device=self.device
            )
            results: Results = results[0].cpu()

            if results.boxes:
                hypothesis = self.parse_hypothesis(results)
                boxes = self.parse_boxes(results)

            if results.masks:
                masks = self.parse_masks(results)

            if results.keypoints:
                keypoints = self.parse_keypoints(results)

            # create detection msgs
            detections_msg = DetectionArray()

            for i in range(len(results)):

                aux_msg = Detection()

                if results.boxes:
                    aux_msg.class_id = hypothesis[i]["class_id"]
                    aux_msg.class_name = hypothesis[i]["class_name"]
                    aux_msg.score = hypothesis[i]["score"]

                    aux_msg.bbox = boxes[i]

                if results.masks:
                    aux_msg.mask = masks[i]

                if results.keypoints:
                    aux_msg.keypoints = keypoints[i]

                detections_msg.detections.append(aux_msg)

            # publish detections
            detections_msg.header = msg.header
            self._pub.publish(detections_msg)

            del results
            del cv_image


def main():
    rclpy.init()
    node = Yolov8Node()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```
