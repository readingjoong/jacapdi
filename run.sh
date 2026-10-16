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
