#!/usr/bin/env bash
#
# setup_nuc_system.sh — AMR 인지 스택 실행 전 1회용 시스템 설정.
#
# 배경(2026-07-27): sensor_fusion 에 /detections 가 안 나오던 근본원인은
#   CycloneDDS 가 큰 이미지 메시지(RGB 1280x720 ≈ 2.7MB)를 전달하지 못한 것.
#   camera_info(작은 메시지)는 되는데 image_raw(큰 메시지)만 드롭됨.
#   -> 소켓 수신버퍼를 키워야 하는데, 리눅스 커널 net.core.rmem_max 가 그 상한이라
#      root 권한으로 rmem_max 를 올려야 한다(launch 로는 불가).
#   추가로 카메라 스트림 안정성을 위해 USB autosuspend 도 끈다.
#
#   CYCLONEDDS_URI(설정파일 지정)는 full_system.launch.py 가 자동으로 넣어주므로
#   로봇 실행 자체엔 .bashrc 가 필요 없다. 다만 CLI 디버그 도구
#   (ros2 topic hz/echo 등)를 위해 .bashrc 에도 export 를 추가한다.
#
# 사용:  일반 사용자로 실행(내부에서 필요 시 sudo 호출).
#   cd ~/test/amr_ws && ./scripts/setup_nuc_system.sh
#   (GRUB 변경 시 재부팅 필요)
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CYCLONE_SRC="$SCRIPT_DIR/../src/amr_perception_bringup/config/cyclonedds.xml"
GRUB_CHANGED=0

if [[ ! -f "$CYCLONE_SRC" ]]; then
  echo "ERROR: cyclonedds.xml 을 찾을 수 없음: $CYCLONE_SRC" >&2
  exit 1
fi

echo "[1/4] 커널 UDP 수신버퍼(net.core.rmem_max) 영구 설정..."
echo 'net.core.rmem_max=2147483647' | sudo tee /etc/sysctl.d/60-cyclonedds.conf >/dev/null
sudo sysctl --system >/dev/null
echo "      rmem_max = $(cat /proc/sys/net/core/rmem_max)"

echo "[2/4] USB autosuspend 비활성(GRUB, 재부팅 후 적용)..."
if grep -q 'usbcore.autosuspend=-1' /etc/default/grub; then
  echo "      이미 설정됨, 건너뜀"
else
  sudo sed -i 's/\(GRUB_CMDLINE_LINUX_DEFAULT="[^"]*\)"/\1 usbcore.autosuspend=-1"/' /etc/default/grub
  sudo update-grub
  GRUB_CHANGED=1
fi

echo "[3/4] CycloneDDS 설정파일 배치 -> ~/cyclonedds.xml (CLI 도구용)..."
cp "$CYCLONE_SRC" "$HOME/cyclonedds.xml"

echo "[4/4] CYCLONEDDS_URI 를 ~/.bashrc 에 추가(CLI 터미널용)..."
if grep -q 'CYCLONEDDS_URI' "$HOME/.bashrc"; then
  echo "      이미 있음, 건너뜀"
else
  {
    echo ''
    echo '# AMR: CycloneDDS 큰 메시지 전달 설정 (setup_nuc_system.sh)'
    echo 'export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml'
  } >> "$HOME/.bashrc"
fi

echo ''
echo "완료."
if [[ "$GRUB_CHANGED" == "1" ]]; then
  echo "  ⚠️ GRUB(autosuspend) 변경됨 → 'sudo reboot' 로 재부팅 필요."
fi
echo "  새 터미널을 열거나 'source ~/.bashrc' 로 CYCLONEDDS_URI 를 적용하세요."
echo "  확인: echo \$CYCLONEDDS_URI / cat /proc/sys/net/core/rmem_max / cat /sys/module/usbcore/parameters/autosuspend"
