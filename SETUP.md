# 새 로봇 구축 가이드 — 라파1 / 라파2 설치

맨바닥(SD카드 굽기)부터 동작 확인까지. `PORTING.md`가 "무엇을 바꿀지"라면
이 문서는 "무엇을 설치할지"입니다.

---

## 0. 먼저 알아둘 것 — 두 라파의 역할이 완전히 다릅니다

| | 라파1 (로봇 본체) | 라파2 (센서) |
|---|---|---|
| 하는 일 | 모터·LiDAR 제어, 조이스틱, 안전 개입 | ToF 읽기, 단차 판정 |
| **ROS2** | **필요** (Jazzy) | **불필요** ← 중요 |
| MQTT | 브로커 + 구독 | 발행 |
| 설치 시간 | 1~2시간 (ROS2 빌드) | **10분** |
| 권장 하드웨어 | Pi 5 (4GB 이상) | Pi 4 / Pi Zero 2W 로도 충분 |

**라파2에는 ROS2를 깔지 마세요.** 단차 감지 모듈(`tof_cliff_detector.py`,
`cliff_bridge.py`)은 순수 Python + MQTT로만 돌아갑니다. ROS2를 깔면 설치 시간이
1시간 늘고 얻는 게 없습니다.

```
   [라파2]  ToF ─ 시리얼 ─ 판정 ─┐
   ROS2 없음                     │  MQTT (WiFi)
                                 ▼
   [라파1]  조이스틱 → 가드노드 → 모터     ← 여기만 ROS2
            + MQTT 브로커(mosquitto)
```

> 라파를 1대로 합쳐도 됩니다. 그 경우 ToF를 라파1에 직접 꽂고 모든 프로세스를
> 한 기기에서 띄우면 됩니다 (`mqtt.host: 127.0.0.1`). 2대로 나눈 이유는
> 센서 처리가 주행 제어를 방해하지 않게 하기 위함입니다.

---

## 1. 공통 — OS 설치

### 1-1. 어떤 우분투?

**Ubuntu Server 24.04 LTS (64-bit)** — 두 대 모두 동일.

| 항목 | 값 | 이유 |
|---|---|---|
| 버전 | **24.04 LTS (Noble)** | ROS2 Jazzy가 24.04 전용. 22.04면 Jazzy 설치 불가 |
| 종류 | **Server** (Desktop 아님) | 모니터 없이 SSH로 쓰므로 GUI 불필요. Desktop은 메모리만 먹음 |
| 아키텍처 | **64-bit (arm64)** | 32-bit는 ROS2 미지원 |

> **22.04를 쓰고 싶다면** ROS2 Humble을 쓰게 됩니다. 그 경우 `cliff_guard_node.py`의
> 메시지 타입을 `TwistStamped` → `Twist`로 바꿔야 합니다 (`PORTING.md` 4-1장).
> 지원 종료는 Jazzy가 2029년, Humble이 2027년이라 **24.04 + Jazzy를 권장**합니다.

### 1-2. SD카드 굽기 (두 대 공통)

**Raspberry Pi Imager** 사용:

1. OS 선택 → `Other general-purpose OS` → `Ubuntu` → **`Ubuntu Server 24.04.x LTS (64-bit)`**
2. ⚙️ 설정(톱니바퀴)에서 **미리** 입력:
   - 호스트명: `rapa1` / `rapa2` (구분되게)
   - 사용자명·비밀번호
   - **WiFi SSID·비밀번호** (국가: KR)
   - **SSH 활성화** ← 체크 안 하면 접속 불가
3. 쓰기 → Pi에 꽂고 부팅

### 1-3. 첫 부팅 후 (두 대 공통)

```bash
ssh <사용자>@<IP>

# 자동 업데이트 끄기 — 작업 중 apt 잠금이 걸리는 것을 막는다
sudo bash -c 'cat > /etc/apt/apt.conf.d/20auto-upgrades << EOF
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";
EOF'

# 절전 끄기 — 로봇이 주행 중 멈추면 안 된다
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

# 시리얼 포트 접근 권한 (★ 빠뜨리기 쉬움)
sudo usermod -aG dialout $USER

sudo apt update && sudo apt upgrade -y
sudo reboot
```

> **`dialout` 그룹을 빠뜨리면** `/dev/ttyACM0` 을 열 때 `Permission denied` 가
> 납니다. ToF도 OpenCR도 안 됩니다. **재부팅(또는 재로그인)해야 적용됩니다.**

### 1-4. 고정 IP ★ 반드시

**지난번 IP가 바뀌어 전부 연결이 끊긴 적이 있습니다.** MQTT 브로커 주소가
설정 파일에 박혀 있으므로, DHCP로 두면 공유기가 주소를 바꾸는 순간 멈춥니다.

**방법 A — 공유기에서 고정 (권장, 간단)**
공유기 관리페이지 → DHCP 예약 → 각 라파의 MAC 주소에 IP 고정.
라파 설정을 건드리지 않아 가장 안전합니다. MAC 확인:
```bash
ip link show wlan0 | grep ether
```

**방법 B — 라파에서 고정 (netplan)**
```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```
```yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: no
      addresses: [192.168.0.37/24]       # 원하는 고정 IP
      routes:
        - to: default
          via: 192.168.0.1               # 공유기 주소
      nameservers:
        addresses: [8.8.8.8, 1.1.1.1]
      access-points:
        "와이파이이름":
          password: "비밀번호"
```
```bash
sudo chmod 600 /etc/netplan/50-cloud-init.yaml
sudo netplan apply
```

> 2.4GHz보다 **5GHz 대역**을 권장합니다. 블루투스(2.4GHz)와 간섭해서
> 패드를 연결하는 순간 SSH가 끊기는 문제를 겪었습니다.

---

## 2. 라파2 설치 (센서) — 먼저 하세요, 10분이면 끝납니다

ROS2가 없어 간단합니다. **여기부터 하면 ToF가 도는 걸 일찍 확인**할 수 있습니다.

### 2-1. 패키지

```bash
sudo apt install -y python3-paho-mqtt python3-serial python3-yaml python3-numpy git
```

딱 이게 전부입니다. 버전은 24.04 기본값으로 충분합니다
(검증 환경: paho-mqtt 1.6.1, pyserial 3.5, PyYAML 6.0.1, numpy 1.26.4).

> `pip3 install` 대신 **apt를 쓰세요.** 24.04는 PEP 668이 적용돼 시스템 파이썬에
> pip 설치가 막혀 있습니다(`externally-managed-environment` 오류).
> 굳이 pip를 써야 하면 가상환경(`python3 -m venv`)을 쓰세요.

### 2-2. 코드 받기

```bash
git clone https://github.com/jungju3756-source/tts_Self_driving_buger.git ~/cliff_mqtt
cd ~/cliff_mqtt
```

### 2-3. ToF 센서 연결 확인

Pico(VL53L8CX)를 USB로 꽂고:

```bash
ls -l /dev/serial/by-id/
```
```
usb-Raspberry_Pi_Pico_2_1A24615E8E6F08FC-if00    ← 이 경로를 복사
```

**`/dev/ttyACM0` 대신 반드시 이 by-id 경로를 쓰세요.** ACM 번호는 재부팅마다
바뀌고, 센서를 2개 이상 달면 거의 확실히 뒤섞입니다.

데이터가 나오는지 확인:
```bash
python3 -c "
import serial; s=serial.Serial('/dev/serial/by-id/usb-...-if00',115200,timeout=2)
for _ in range(12): print(s.readline().decode(errors='replace').rstrip())"
```
`Frame #...` 과 `R0 412 418 ----` 같은 줄이 보이면 정상입니다.

> `cat /dev/ttyACM0` 으로는 **아무것도 안 보일 수 있습니다.** Pico CDC 펌웨어가
> 호스트의 DTR 신호가 올라와야 출력하기 때문입니다. pyserial은 포트를 열 때
> DTR을 자동으로 올리므로 위 파이썬 코드로 확인하세요.

### 2-4. 설정

```bash
nano ~/cliff_mqtt/config/cliff_config.yaml
```
```yaml
mqtt:
  host: 192.168.0.37        # ← 라파1의 고정 IP

sensor:
  port: /dev/serial/by-id/usb-Raspberry_Pi_Pico_2_...-if00   # ← 2-3에서 복사한 경로
```

로봇 치수·속도 관련 값은 `PORTING.md` 3장을 보고 맞추세요.

---

## 3. 라파1 설치 (로봇 본체)

### 3-1. ROS2 Jazzy

```bash
# 로케일
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 저장소 등록
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
ARCH=$(dpkg --print-architecture)
CODENAME=$(. /etc/os-release && echo $UBUNTU_CODENAME)
sudo bash -c "echo 'deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu ${CODENAME} main' > /etc/apt/sources.list.d/ros2.list"
sudo apt update
```

**⚠️ 여기서 의존성 다운그레이드를 먼저 하세요 (안 하면 설치 실패)**

Ubuntu 24.04 보안 업데이트가 ROS2 패키지와 충돌합니다. 이걸 건너뛰면
`held broken packages` 오류로 막힙니다:

```bash
sudo apt install -y --allow-downgrades \
  liblz4-1=1.9.4-1build1 \
  libzstd1=1.5.5+dfsg2-2build1 \
  "zlib1g=1:1.3.dfsg-3.1ubuntu2" \
  libbz2-1.0=1.0.8-5.1
```

> 위 버전 번호가 안 맞다고 나오면 `apt-cache policy liblz4-1` 로 설치 가능한
> 버전을 확인해 맞추세요. 시간이 지나면 번호가 달라질 수 있습니다.

```bash
# ROS2 본체 (GUI 없는 서버용)
sudo DEBIAN_FRONTEND=noninteractive apt install -y ros-jazzy-ros-base ros-dev-tools
```

### 3-2. 로봇 제어 패키지

```bash
sudo apt install -y \
  ros-jazzy-dynamixel-sdk \
  ros-jazzy-turtlebot3-msgs \
  ros-jazzy-hls-lfcd-lds-driver \
  ros-jazzy-joy \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-xacro ros-jazzy-urdf \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
  ros-jazzy-tf2 ros-jazzy-tf2-ros
```

| 패키지 | 용도 |
|---|---|
| `dynamixel-sdk` | 모터 제어 |
| `turtlebot3-msgs` | TB3 메시지 타입 |
| `hls-lfcd-lds-driver` | LiDAR (LDS-02) |
| **`joy`** | **패드 읽기** |
| **`teleop-twist-joy`** | **축 → 속도 변환** |

> `joy` / `teleop-twist-joy` 는 **apt로 설치하세요.** 기존 로봇에서는 소스를
> 클론해 빌드했는데, apt에 멀쩡한 버전(joy 3.3.0 / teleop 2.6.5)이 있어
> 빌드할 이유가 없습니다.

> **전동휠체어처럼 TurtleBot3가 아닌 로봇**이라면 `dynamixel-sdk`,
> `turtlebot3-msgs`, `hls-lfcd-lds-driver` 는 불필요합니다. 그 로봇의 드라이버를
> 쓰되, `/cmd_vel` 을 구독하기만 하면 이 모듈은 그대로 붙습니다.

### 3-3. TurtleBot3 워크스페이스 (TB3를 쓸 때만)

```bash
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/ld08_driver.git

# SBC에 불필요한 것 제거 (빌드 시간 단축)
rm -rf turtlebot3/turtlebot3_cartographer turtlebot3/turtlebot3_navigation2

cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --parallel-workers 1
```

> **30분~1시간 걸립니다.** `--parallel-workers 1` 은 메모리 부족으로 빌드가
> 죽는 것을 막기 위함입니다. 커피 한 잔 하고 오세요.

### 3-4. MQTT 브로커

```bash
sudo apt install -y mosquitto mosquitto-clients python3-paho-mqtt python3-yaml python3-numpy

sudo bash -c 'printf "listener 1883 0.0.0.0\nallow_anonymous true\n" \
  > /etc/mosquitto/conf.d/cliff.conf'
sudo systemctl enable --now mosquitto

# 확인 — 0.0.0.0:1883 이 보여야 외부에서 붙을 수 있다
ss -ltn | grep 1883
```

> `allow_anonymous true` 는 **개발용**입니다. 실제 차량에 올릴 때는 인증을
> 켜고 `config/cliff_config.yaml` 의 `mqtt.username/password` 를 채우세요.

### 3-5. 환경변수

```bash
cat >> ~/.bashrc << 'EOF'
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export ROS_DOMAIN_ID=40
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
EOF
source ~/.bashrc
```

> **`ROS_DOMAIN_ID` 는 ROS2를 쓰는 모든 기기에서 같아야 합니다.** 다르면
> 토픽이 서로 안 보입니다. 라파2는 ROS2를 안 쓰므로 무관합니다.
> (기존 환경에 `40` / `55` / `30` 이 섞여 있었습니다 — 새 로봇에서는 하나로 통일하세요.)

### 3-6. USB 포트 권한 (OpenCR)

```bash
sudo cp ~/turtlebot3_ws/install/turtlebot3_bringup/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules \
  /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 3-7. 코드 받기

```bash
git clone https://github.com/jungju3756-source/tts_Self_driving_buger.git ~/cliff_mqtt
```

`config/cliff_config.yaml` 의 `mqtt.host` 를 **라파1 자신의 IP**(또는 `127.0.0.1`)로
맞춥니다.

### 3-8. 블루투스 패드

```bash
bluetoothctl
# scan on → 패드를 페어링 모드로 → 주소 확인
# pair E4:17:D8:E6:37:F9
# trust E4:17:D8:E6:37:F9      ← trust 해야 다음 부팅에 자동 연결
# connect E4:17:D8:E6:37:F9
# quit

ls /dev/input/js0      # 있으면 성공
```

축 번호 실측 (**패드마다 다릅니다**):
```bash
ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0 &
ros2 topic echo /joy --field axes
```
D-pad를 눌러 몇 번째 값이 바뀌는지 보고 `config/teleop_8bitdo.yaml` 을 고칩니다.

---

## 4. 동작 확인 — 이 순서대로

하나씩 확인하세요. 중간에 건너뛰면 어디가 문제인지 알 수 없습니다.

### ① 라파2 — 센서 단독 (브로커 없이)

```bash
cd ~/cliff_mqtt
python3 tools/test_logic.py              # 25종 통과해야 함
python3 rapa2/tof_cliff_detector.py --no-mqtt -v
```
`cliff=False zone=NONE` 이 흐르면 정상입니다.

### ② 기준면 생성 ★ 안 하면 동작 안 함

센서를 **최종 위치에 고정**하고 평평한 바닥 위에서:
```bash
python3 rapa2/tof_cliff_detector.py --calibrate 60     # 6초, 건드리지 말 것
```

### ③ 라파1 — 브로커 연결

```bash
# 라파1에서 구독 대기
mosquitto_sub -h 127.0.0.1 -t 'wheelchair/#' -v

# 라파2에서 발행
python3 rapa2/tof_cliff_detector.py -v
```
라파1 쪽에 JSON이 흐르면 네트워크가 뚫린 것입니다.
안 보이면 → 방화벽, IP 오타, `0.0.0.0:1883` 바인딩 확인.

### ④ 시각화

라파1에서 `python3 tools/tof_web.py` 를 띄우고 PC 브라우저로
`http://<라파1 IP>:8080` 접속. 8×8 숫자가 보이면:
- 평평한 바닥 → 배너가 **초록 CLEAR**
- 책상 끝 → **빨강 STOP**

여기서 **좌우 방향이 맞는지** 꼭 확인하세요. 틀리면 `col0_is_left` 를 뒤집습니다.
(틀린 채로 두면 우회를 낭떠러지 쪽으로 합니다.)

### ⑤ 주행 — 빈 차로 먼저

```bash
# 라파1 — 터미널 4개
ros2 launch turtlebot3_bringup robot.launch.py
ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0
ros2 run teleop_twist_joy teleop_node \
    --ros-args --params-file ~/cliff_mqtt/config/teleop_8bitdo.yaml \
    -r /cmd_vel:=/cmd_vel_manual
python3 ~/cliff_mqtt/rapa1/cliff_guard_node.py

# 바퀴가 손으로 헛돌면
ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"

# 확인용
python3 ~/cliff_mqtt/rapa1/drive_monitor.py
```

확인 항목:
- [ ] 패드로 전후진·회전이 되는가
- [ ] 단차 앞에서 **전진이 막히는가**
- [ ] 그 상태에서 **후진·회전은 되는가** (안 되면 사용자가 갇힘)
- [ ] 라파2 검출기를 강제 종료하면 `FAULT` 가 뜨는가 (LWT 확인)

---

## 5. 자동 시작 (선택)

매번 터미널 4개를 띄우기 번거로우면 systemd로 등록합니다. **라파2 예시:**

```bash
sudo tee /etc/systemd/system/cliff-detector.service << 'EOF'
[Unit]
Description=ToF Cliff Detector
After=network-online.target

[Service]
Type=simple
User=jungju2
WorkingDirectory=/home/jungju2/cliff_mqtt
ExecStart=/usr/bin/python3 rapa2/tof_cliff_detector.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable --now cliff-detector
systemctl status cliff-detector
```

`Restart=always` 덕에 죽어도 5초 뒤 되살아납니다. `cliff_bridge.py` 도 같은
방식으로 등록하세요.

> **주행 노드는 자동 시작을 권하지 않습니다.** 전원을 넣자마자 로봇이 명령을
> 받을 수 있는 상태가 되는 건 위험합니다.

---

## 6. 트러블슈팅 — 실제로 겪은 것들

| 증상 | 원인 | 해결 |
|---|---|---|
| `/dev/ttyACM0` Permission denied | `dialout` 그룹 없음 | `usermod -aG dialout $USER` 후 **재부팅** |
| 센서 포트가 뒤바뀜 | ACM 번호가 재부팅마다 변함 | `/dev/serial/by-id/` 경로 사용 |
| `cat /dev/ttyACM0` 무반응 | Pico가 DTR 신호를 기다림 | pyserial로 열면 정상 |
| MQTT 연결 실패 | IP가 DHCP로 바뀜 | **고정 IP** 설정 |
| pip 설치 거부 | 24.04 PEP 668 | `apt install python3-*` 사용 |
| ROS2 설치 `held broken packages` | 24.04 보안 업데이트 충돌 | 3-1장 다운그레이드 **먼저** |
| 패드 눌러도 안 움직임 ① | `autorepeat_rate` 누락 | `-p autorepeat_rate:=20.0` |
| 패드 눌러도 안 움직임 ② | 축 번호가 패드와 불일치 | `ros2 topic echo /joy --field axes` 로 실측 |
| 패드 눌러도 안 움직임 ③ | 리맵 누락 | `-r /cmd_vel:=/cmd_vel_manual` |
| 방향이 반대 | scale 부호 | `teleop_*.yaml` 의 `scale_linear.x` 부호 반전 |
| 바퀴가 손으로 헛돎 | 모터 토크 꺼짐 | `/motor_power` 서비스 호출 |
| 주행 중 노드가 죽음 | **배터리 전압 강하** | 충전 또는 SMPS (11.3V/31%에서 재현됨) |
| 패드 연결하니 SSH 끊김 | WiFi(2.4G)·BT 간섭 | **5GHz WiFi** 사용 |
| 평지에서 계속 STOP | 기준면 없음/어긋남 | `--calibrate 60` 재실행 |
| 우회를 낭떠러지 쪽으로 함 | `col0_is_left` 반대 | 설정 뒤집기 |

---

## 7. 설치 요약표

복사해서 체크리스트로 쓰세요.

### 라파2 (10분)
```bash
sudo usermod -aG dialout $USER && sudo reboot
sudo apt install -y python3-paho-mqtt python3-serial python3-yaml python3-numpy git
git clone https://github.com/jungju3756-source/tts_Self_driving_buger.git ~/cliff_mqtt
ls -l /dev/serial/by-id/                    # 경로 복사 → cliff_config.yaml
nano ~/cliff_mqtt/config/cliff_config.yaml  # mqtt.host, sensor.port
python3 ~/cliff_mqtt/rapa2/tof_cliff_detector.py --calibrate 60
```

### 라파1 (1~2시간)
```bash
sudo usermod -aG dialout $USER && sudo reboot
# ROS2 Jazzy  (3-1장 — 다운그레이드 먼저!)
sudo apt install -y ros-jazzy-ros-base ros-dev-tools
sudo apt install -y ros-jazzy-dynamixel-sdk ros-jazzy-turtlebot3-msgs \
  ros-jazzy-hls-lfcd-lds-driver ros-jazzy-joy ros-jazzy-teleop-twist-joy \
  ros-jazzy-xacro ros-jazzy-urdf ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher ros-jazzy-tf2 ros-jazzy-tf2-ros
sudo apt install -y mosquitto mosquitto-clients python3-paho-mqtt python3-yaml python3-numpy
# 워크스페이스 빌드 (3-3장) → 환경변수 (3-5장) → 블루투스 (3-8장)
git clone https://github.com/jungju3756-source/tts_Self_driving_buger.git ~/cliff_mqtt
```

---

## 관련 문서

| 문서 | 내용 |
|---|---|
| `README.md` | 모듈 구조, 판정 원리, 실행 명령 |
| **`PORTING.md`** | **이식 시 바꿔야 할 설정값** (로봇 폭·속도·제동거리) |
| `SETUP.md` | 이 문서 — 맨바닥부터 설치 |

설치가 끝나면 **`PORTING.md` 3장(로봇 치수·속도)** 을 반드시 읽으세요.
터틀봇 기본값을 그대로 쓰면 큰 로봇에서는 위험합니다.
