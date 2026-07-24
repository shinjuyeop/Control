#!/bin/bash
# STM_A와 STM_B의 micro-ROS 정적 라이브러리를 kaiev26_msgs 포함으로 재생성한다.
#
# 필요 조건: docker, 네트워크. STM 보드는 필요 없다.
# 실행 위치: Control 저장소 루트에서 `bash firmware/tools/regen_microros.sh`
#
# 결과물: firmware/STM_A/Middlewares/micro_ros/{libmicroros.a, microros_include/,
# available_ros2_types, built_packages} 및 STM_B 동일 경로. 두 보드는 같은
# STM32G474(cortex-m4, fpv4-sp-d16, hard float)라서 산출물을 공유한다.
#
# 재실행이 필요한 경우는 펌웨어가 사용하는 kaiev26_msgs 타입이 바뀔 때뿐이다.
# 그때는 KAIEV26_MSGS_COMMIT을 새 커밋으로 올리고 이 스크립트를 다시 돌린 뒤,
# 산출물과 펌웨어 코드, host 코드를 한 PR로 묶어서 올린다.
set -euo pipefail

# 버전 고정. 바꿀 때는 아래 manifest도 함께 갱신된다.
UTILS_REPO=https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git
UTILS_COMMIT=${UTILS_COMMIT:-bd531b273c1bcd070b3143c5642128ec75a6f04e} # humble
MSGS_REPO=${MSGS_REPO:-https://github.com/TeamKAI-DL/kaiev26_msgs.git}
KAIEV26_MSGS_COMMIT=${KAIEV26_MSGS_COMMIT:-10fbff33a2fc67260f6e429aff5dca19a0001822}
DOCKER_IMAGE=microros/micro_ros_static_library_builder:humble
MCU_CFLAGS="-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -Os -g0"

REPO_ROOT=$(cd "$(dirname "$0")/../.." && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

echo "[1/5] 도구 저장소와 메시지 패키지를 고정 커밋으로 받는다"
git clone -q "$UTILS_REPO" "$WORK/project/micro_ros_stm32cubemx_utils"
git -C "$WORK/project/micro_ros_stm32cubemx_utils" checkout -q "$UTILS_COMMIT"
rm -rf "$WORK/project/micro_ros_stm32cubemx_utils/.git"

EP="$WORK/project/micro_ros_stm32cubemx_utils/microros_static_library/library_generation/extra_packages"
if [ -n "${KAIEV26_MSGS_DIR:-}" ]; then
  rsync -a --exclude .git "$KAIEV26_MSGS_DIR/" "$EP/kaiev26_msgs/"
else
  git clone -q "$MSGS_REPO" "$EP/kaiev26_msgs"
  git -C "$EP/kaiev26_msgs" checkout -q "$KAIEV26_MSGS_COMMIT"
  rm -rf "$EP/kaiev26_msgs/.git"
fi

echo "[2/5] colcon.meta에 kaiev26_msgs 항목을 추가하고 CFLAGS 응답용 Makefile을 만든다"
python3 - "$WORK/project/micro_ros_stm32cubemx_utils/microros_static_library/library_generation/colcon.meta" <<'PY'
import json, sys
p = sys.argv[1]
m = json.load(open(p))
m["names"]["kaiev26_msgs"] = {"cmake-args": ["-DBUILD_TESTING=OFF"]}
json.dump(m, open(p, "w"), indent=4)
PY
printf 'print_cflags:\n\t@echo "%s"\n' "$MCU_CFLAGS" > "$WORK/project/Makefile"

echo "[3/5] docker로 라이브러리를 생성한다 (수 분 소요)"
echo y | docker run --rm -i -v "$WORK/project":/project \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
  "$DOCKER_IMAGE"

OUT="$WORK/project/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros"
test -f "$OUT/libmicroros.a"
test -d "$OUT/microros_include/kaiev26_msgs"

echo "[4/5] 두 보드의 Middlewares/micro_ros에 산출물을 반영한다"
for BOARD in STM_A STM_B; do
  DEST="$REPO_ROOT/firmware/$BOARD/Middlewares/micro_ros"
  rm -rf "$DEST/microros_include"
  cp -R "$OUT/microros_include" "$DEST/microros_include"
  cp "$OUT/libmicroros.a" "$DEST/libmicroros.a"
  cp "$OUT/available_ros2_types" "$DEST/available_ros2_types"
  cp "$OUT/built_packages" "$DEST/built_packages"
done

echo "[5/5] manifest를 기록한다"
IMAGE_DIGEST=$(docker inspect --format='{{index .RepoDigests 0}}' "$DOCKER_IMAGE" 2>/dev/null || echo "$DOCKER_IMAGE")
cat > "$REPO_ROOT/firmware/tools/microros_manifest.txt" <<MANIFEST
generated_by=firmware/tools/regen_microros.sh
ros_distro=humble
docker_image=$IMAGE_DIGEST
micro_ros_stm32cubemx_utils=$UTILS_COMMIT
kaiev26_msgs=$KAIEV26_MSGS_COMMIT
mcu_cflags=$MCU_CFLAGS
MANIFEST

echo "완료. PC 쪽 kaiev26_msgs 서브모듈 커밋이 $KAIEV26_MSGS_COMMIT 인지 확인할 것."
