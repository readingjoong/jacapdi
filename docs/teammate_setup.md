# 팀원 PC에서 gazebo-1031 브랜치 받기

저장소가 `~/jacapdi-docker` 에 있다고 가정합니다. main(중간평가 코드)은 그대로이고, 이 브랜치로 바꿔서 시험한 뒤 언제든 돌아갈 수 있습니다.

## 브랜치 받기

로컬 수정부터 확인합니다.

```bash
cd ~/jacapdi-docker && git status
```

브랜치로 바꿉니다.

```bash
cd ~/jacapdi-docker && git fetch origin && git checkout gazebo-1031
```

중간평가 코드로 돌아갈 때는 이것입니다.

```bash
cd ~/jacapdi-docker && git checkout main
```

- **tune.json 수정은 유지됩니다.** 두 브랜치에서 tune.json 이 같아서, 노트북에서 슬라이더로 바꾼 값은 브랜치를 바꿔도 그대로 남습니다.
- **다른 파일을 고쳤다면 먼저 확인이 필요합니다.** 경로 노드나 car.sh 를 노트북에서 직접 고쳤다면 브랜치 전환이 거부됩니다. 그 수정을 어떻게 할지 먼저 정해야 합니다.
- **도커 이미지 재빌드는 테스트용입니다.** 바뀐 것은 테스트 도구(python3-pytest) 한 줄뿐이라 주행에는 필요 없습니다. 테스트를 돌리려면 다시 빌드합니다.

```bash
docker build -t jacapdi/humble:latest ~/jacapdi-docker
```

## Claude Code 에 붙여 넣을 지시문

```text
~/jacapdi-docker 저장소를 origin 의 gazebo-1031 브랜치로 전환해줘.
전환 전에 git status 로 로컬 수정이 있으면 지우거나 덮지 말고 나한테 먼저 보여줘.
전환 뒤 docs 폴더의 가제보 보고서와 README 의 실차 시험 프로토콜 절을 읽고 요약해줘.
컨테이너(car.sh shell) 안에서 python3 -m pytest -q tests 를 돌려 결과를 알려줘.
커밋, 푸시, main 병합은 하지 마. 차를 움직이는 명령(car_go.sh, car.sh all, car.sh lap)은 내가 직접 실행할게.
```

차를 움직이는 명령을 사람이 직접 치게 한 것은 예전 실차 키트와 같은 원칙입니다.

## 그다음

시험 순서는 [가제보 결과 정리](gazebo_1031.md)의 실차 시험 순서, 정본은 README 의 "실차 1차 시험 프로토콜" 절입니다. 측정 → 2차선 회귀 → 1차선 → 차선 변경 순서입니다.
