#!/bin/bash
# wsl_rviz.sh  —  Windows WSL2(Ubuntu 24.04 + Jazzy)에서 라파1 지도를 실시간으로 보기
#
#   WSL 에서 한 번만:
#     scp jungju@192.168.0.57:~/tts_Self_driving_buger/tools/wsl_rviz.sh ~/ && bash ~/wsl_rviz.sh
#   다음부터는:
#     bash ~/wsl_rviz.sh
#
# 전제: Windows 쪽 .wslconfig 에 networkingMode=mirrored (+ wsl --shutdown 한 번)
#       → WSL 안에서 `ip -4 addr` 에 192.168.0.x 가 보여야 한다.

PI=192.168.0.57
RVIZ=~/carto.rviz

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=40
export TURTLEBOT3_MODEL=waffle_pi

echo "== 1. 네트워크 (mirrored 모드면 192.168.0.x 가 보여야 함)"
ip -4 -o addr | awk '{print "   ", $2, $4}'
if ! ip -4 -o addr | grep -q '192\.168\.0\.'; then
    echo "   ✗ 192.168.0.x 주소가 없음 → NAT 모드. Windows 에서 .wslconfig 에 networkingMode=mirrored 넣고 wsl --shutdown"
    exit 1
fi
ping -c 1 -W 2 $PI >/dev/null && echo "   ✓ 라파1($PI) ping OK" || echo "   ✗ 라파1 ping 실패 (같은 Wi-Fi 인지 확인)"

echo "== 2. RViz 설정 받기"
if scp -q jungju@$PI:~/tts_Self_driving_buger/config/carto.rviz jungju@$PI:~/tts_Self_driving_buger/tools/place.py ~/; then
    echo "   ✓ $RVIZ, ~/place.py"
else
    echo "   ✗ scp 실패 — 기존 $RVIZ 사용"
fi

echo "== 3. 로봇 모델 패키지"
if ! ros2 pkg prefix turtlebot3_description >/dev/null 2>&1; then
    sudo apt install -y ros-jazzy-turtlebot3-description
fi

# 토픽 확인 단계는 WSL 에서 수십 초씩 걸려 뺐다 (연결은 2026-10-02 확인됨).
# 안 보이면 RViz 왼쪽 Displays 에 경고가 뜬다 → 그때 수동 확인:
#   ros2 topic list --no-daemon --spin-time 5
export ROS_STATIC_PEERS=$PI      # 멀티캐스트가 막혀도 라파1 을 직접 찾게

echo "== 4. RViz2 실행"
exec rviz2 -d $RVIZ
