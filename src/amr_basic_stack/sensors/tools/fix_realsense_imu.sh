#!/usr/bin/env bash
# D455 모션 모듈(IMU) 고착 복구.
# ---------------------------------------------------------------
# 증상:  카메라 노드가 기동 직후 SIGSEGV 로 죽는다.
#        [ERROR] process has died [pid ..., exit code -11]
#        직전에 이 경고가 나온다:
#        WARNING (backend-hid.cpp:740) HID set_power 1 failed for .../iio:deviceN/buffer/enable
#
# 원인:  카메라 프로세스를 정상 종료하지 않고 죽이면(kill/pkill/SIGKILL) 리눅스 IIO HID
#        센서 버퍼가 enable=1 인 채로 남는다. 이미 활성인 버퍼는 재설정이 안 되므로
#        librealsense 의 set_power 1 이 실패하고 그대로 세그폴트한다.
#        2026-09-03 실측: 카메라를 pkill 한 뒤 다음 런치가 이 방식으로 죽었다.
#
# 예방:  카메라 런치는 반드시 Ctrl+C(SIGINT)로 내려라. 그러면 버퍼가 0 으로 풀린다.
#        (검증: SIGINT 종료 후 buffer/enable = 0, SIGKILL 후 = 1)
#
# 사용:  ./fix_realsense_imu.sh        (sudo 불필요 — 버퍼 파일이 0666 이다)
# ---------------------------------------------------------------
set -eo pipefail

# [r] 대괄호는 이 스크립트 자신의 명령줄이 패턴에 걸리지 않게 한다 (ps|grep 관용구와 같은 이유).
if pgrep -f '[r]ealsense2_camera_node' > /dev/null; then
    echo "[ERROR] 카메라 노드가 아직 돌고 있다. 먼저 Ctrl+C 로 내려라."
    pgrep -af '[r]ealsense2_camera_node' | sed 's/ --ros-args.*//'
    exit 1
fi

found=0
for d in /sys/bus/iio/devices/iio:device*; do
    [ -e "$d/buffer/enable" ] || continue
    name=$(cat "$d/name" 2>/dev/null || echo "?")
    case "$name" in accel_3d|gyro_3d) ;; *) continue ;; esac
    found=1
    before=$(cat "$d/buffer/enable")
    if [ "$before" = "0" ]; then
        echo "  $name: 이미 해제됨 (enable=0)"
    elif echo 0 > "$d/buffer/enable" 2>/dev/null; then
        echo "  $name: enable $before -> $(cat "$d/buffer/enable")  ✓ 해제"
    else
        echo "  $name: 쓰기 실패 — sudo 로 다시 시도하거나 USB 를 뺐다 꽂아라"; exit 1
    fi
done

[ "$found" = "1" ] || { echo "[ERROR] accel_3d/gyro_3d IIO 장치가 없다. 카메라 USB 연결을 확인해라."; exit 1; }
echo "복구 완료. 런치를 다시 띄우고 'ros2 topic hz /camera/gyro/sample' 이 200 나오는지 확인해라."
