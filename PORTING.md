# 다른 로봇으로 옮기기 — 이식 가이드

이 모듈은 **TurtleBot3 Burger + 라즈베리파이 2대**에서 만들어졌습니다.
다른 로봇(전동휠체어 등)으로 옮길 때 **무엇을 반드시 바꿔야 하는지**를 정리합니다.

알고리즘과 상태머신은 로봇과 무관하므로 그대로 쓸 수 있습니다.
바꿔야 하는 것은 ① 네트워크 주소 ② 센서 장착 ③ 로봇 치수·속도, 세 가지입니다.

---

> **OS·패키지 설치부터 해야 한다면 → [`SETUP.md`](SETUP.md)** 를 먼저 보세요.
> 이 문서는 설치가 끝난 뒤 "설정을 무엇으로 바꿀지"를 다룹니다.

## 0. 30초 요약 — 최소 이식 절차

```bash
# 1) 새 로봇에 받기
git clone <레포주소> ~/cliff_mqtt && cd ~/cliff_mqtt

# 2) 의존성
pip3 install pyserial paho-mqtt pyyaml numpy
sudo apt install -y mosquitto mosquitto-clients

# 3) 설정 3줄 수정  (config/cliff_config.yaml)
#    mqtt.host        → 새 브로커 IP
#    sensor.port      → 새 ToF 장치 경로
#    guard.*          → 새 로봇 속도 (아래 3장)

# 4) 센서 고정한 뒤 평지에서 기준면 생성 (6초)
python3 rapa2/tof_cliff_detector.py --calibrate 60

# 5) 검증 — 브로커 없이 돌아감
python3 tools/test_logic.py
```

> **4번을 건너뛰면 동작하지 않습니다.** `baseline` 모드는 기준면 파일이 없으면
> 시작 시 에러를 내고 종료합니다. 기준면은 센서 자세에 묶여 있어 다른 로봇의
> 것을 그대로 쓸 수 없습니다.

---

## 1. 반드시 바꿔야 하는 것 — 안 바꾸면 **동작하지 않음**

### 1-1. MQTT 브로커 주소

| 파일 | 줄 | 현재 값 | 바꿀 값 |
|---|---|---|---|
| `config/cliff_config.yaml` | `mqtt.host` | `192.168.0.37` | 새 브로커 IP |

이 한 줄을 **세 프로세스가 공유**합니다 (detector / bridge / guard). 브로커는
어느 기기에 둬도 되지만, 보통 로봇 본체에 둡니다.

> ⚠️ **개발 중 실제로 겪은 문제** — 라즈베리파이 IP가 DHCP로 바뀌면 이 값이
> 어긋나 전부 연결 실패합니다. 새 로봇에서는 **고정 IP를 쓰거나**, 브로커를
> 로컬에 두고 `127.0.0.1`로 지정하세요.

문서·주석에도 옛 IP가 남아 있습니다 (동작에는 영향 없음):
`README.md`, `tools/tof_web.py`, `tools/replay.py`, `rapa1/drive_monitor.py`

### 1-2. ToF 장치 경로

| 파일 | 줄 | 현재 값 |
|---|---|---|
| `config/cliff_config.yaml` | `sensor.port` | `/dev/ttyACM0` |

**`/dev/ttyACM*` 번호는 재부팅마다 바뀝니다.** 센서를 2개 이상 달면 거의 확실히
뒤섞입니다. 새 로봇에서는 **by-id 경로**를 쓰세요:

```bash
ls -l /dev/serial/by-id/
# → /dev/serial/by-id/usb-Raspberry_Pi_Pico_2_1A24615E8E6F08FC-if00
```

```yaml
sensor:
  port: /dev/serial/by-id/usb-Raspberry_Pi_Pico_2_1A24615E8E6F08FC-if00
```

udev 심링크(`/dev/tof`)를 만드는 방법도 있습니다 — `turtlebot3_ws/IMPLEMENTATION.md`의
`99-pico-tof.rules` 참고.

### 1-3. 평지 기준면 (`config/ground_baseline.yaml`)

**이 파일은 현재 센서 자세에 묶여 있습니다. 반드시 새로 만들어야 합니다.**

판정 방식(`detect.mode: baseline`)이 "이 셀이 평지에서 평소 몇 mm를 찍는가"를
기억해두고 비교하는 구조라, 센서 각도·높이가 바뀌면 기준값이 전부 무의미해집니다.

```bash
# 센서를 최종 위치에 고정 → 평평한 바닥 위에서 → 6초간 정지
python3 rapa2/tof_cliff_detector.py --calibrate 60
```

다시 떠야 하는 경우:
- 다른 로봇에 옮겼을 때
- 브래킷을 건드렸을 때 / 각도를 바꿨을 때
- 차체 높이가 바뀌었을 때 (사람이 탄 전동휠체어는 서스펜션이 내려앉음)

> 기준면이 어긋나도 **위험한 방향으로 틀리지는 않습니다.** 거리가 가까워지는
> 쪽은 절대 낙차로 판정하지 않기 때문입니다. 다만 감도가 떨어지므로,
> 시연·운용 전에는 반드시 다시 뜨세요.

---

## 2. 센서 장착 — 하드웨어 전제

### 2-1. 펌웨어 출력 포맷 (가장 중요)

`rapa2/tof_cliff_detector.py`의 `SerialFrameSource`가 **이 텍스트 포맷**을 파싱합니다:

```
Frame #114297
       C0   C1   C2   C3   C4   C5   C6   C7
R0    412  418 ----  405  399  402  410  415
...
R7    ...
```

- `R<숫자>` 로 시작하는 줄에서 8개 값을 읽음
- 단위 **mm**, `----` = 측정 실패
- 그 외 줄(헤더, ASCII 히트맵)은 무시

**다른 펌웨어를 쓰면 `SerialFrameSource.read_frame()` 만 고치면 됩니다.**
판정 로직은 `(8,8) numpy 배열, mm, 실패는 NaN` 만 받으면 되므로 영향 없습니다.

바이너리 프로토콜이나 I2C 직결로 바꿀 경우에도 같은 자리만 교체하세요.

### 2-2. 센서 방향

| 설정 | 의미 |
|---|---|
| `sensor.row0_is_top` | R0 행이 영상 **위쪽(먼 쪽)** 이면 `true` |
| `sensor.col0_is_left` | C0 열이 로봇 기준 **좌측**이면 `true` |

이 둘이 틀리면 **우회 방향이 반대로 나갑니다** (낭떠러지 쪽으로 틀어버림).
반드시 실측으로 확인하세요:

```bash
python3 tools/tof_web.py      # 브라우저에서 8x8 숫자 확인
```

손을 센서 **왼쪽 아래**에 대고, 화면에서 값이 줄어드는 칸이 **왼쪽 아래**인지 봅니다.
반대면 해당 설정을 뒤집습니다.

### 2-3. 장착 각도·높이 — `geometric` 모드에서만 필요

```yaml
mount_height_m: 0.190   # 지면 ~ 센서 광학중심
tilt_deg: 13.8          # 수평 기준 아래로 기운 각
```

현재 기본값인 **`baseline` 모드는 이 값을 쓰지 않습니다.** 무시해도 됩니다.
`geometric` 모드(낙차 깊이를 cm로 알고 싶을 때)를 쓸 때만 실측하세요 —
`tools/fit_mount.py` 가 평지 데이터로 역산해 줍니다.

---

## 3. 로봇 치수·속도 — 안 바꾸면 **위험**

TurtleBot3 Burger는 **폭 18cm, 최고 0.22m/s** 입니다.
전동휠체어는 **폭 60~70cm, 1.5~2m/s** 입니다. 아래 값을 그대로 쓰면 안 됩니다.

### 3-1. 로봇 폭

```yaml
detect:
  center_band_m: 0.12    # 진행 통로의 반폭 [m]
```

"이 범위 안에 낙차가 있으면 전방 단차(=정지 대상)"를 가르는 값입니다.
**로봇 반폭 + 여유**로 잡으세요.

| 로봇 | 전폭 | 권장 `center_band_m` |
|---|---|---|
| TurtleBot3 Burger | 0.18 m | 0.12 (현재) |
| 전동휠체어 (일반) | 0.65 m | **0.40 ~ 0.45** |

**작게 잡으면 위험합니다** — 바퀴가 지나갈 자리의 낭떠러지를 "측면"으로 분류해
정지하지 않고 지나갑니다.

### 3-2. 속도

```yaml
guard:
  detour_linear: 0.08      # 우회 전진 속도 [m/s]
  detour_angular: 0.8      # 우회 선회 각속도 [rad/s]
  max_angular: 1.2         # 각속도 상한 [rad/s]
  degraded_speed_scale: 0.3  # 센서 이상 시 속도 배율
```

`detour_angular: 0.8 rad/s` 는 **초당 46°** 입니다. 작은 터틀봇에는 적당하지만
사람이 탄 휠체어에서는 급격합니다. **0.3~0.4 rad/s 이하**를 권장하고, 실차에서
사람을 태우기 전에 빈 차로 먼저 확인하세요.

### 3-3. 제동거리 — 가장 중요한 안전 항목

**이 모듈은 제동거리를 계산하지 않습니다.** 낭떠러지를 감지하면 즉시 정지
명령을 보낼 뿐, "지금 속도로 멈출 수 있는 거리인가"는 보지 않습니다.

| 로봇 | 속도 | 공주+제동거리(대략) | 현재 감지거리 |
|---|---|---|---|
| TurtleBot3 | 0.2 m/s | ~5 cm | 0.3~1.0 m ✅ 충분 |
| 전동휠체어 | 1.5 m/s | **1.5~2 m** | 0.3~1.0 m ❌ **부족** |

**휠체어 속도에서는 현재 센서 배치로 늦습니다.** 대응이 필요합니다:

1. **센서를 더 멀리 보게 장착** (기울기를 줄여 전방 2~3m 확보)
2. **속도 제한** — 감지거리 안에서 멈출 수 있는 속도로 상한
3. **속도 비례 정지거리** — `cliff_bridge.py`에 속도 입력을 추가해
   `필요정지거리 = v²/(2a) + v·t_reaction` 로 판정 문턱을 동적으로 조정

3번이 정석이지만 현재 구현에는 **없습니다.** 실차 적용 시 반드시 추가하세요.

### 3-4. 정지 유지 시간

```yaml
command:
  min_stop_s: 1.0     # 우회 전에 완전 정지를 유지할 시간
```

사람이 탄 차량에서는 급정지 후 바로 선회하면 불안합니다. **1.5~2.0초**로
늘리는 것을 권장합니다.

---

## 4. ROS2 연동 — 라파1 쪽 (`rapa1/cliff_guard_node.py`)

이 노드만 ROS2에 의존합니다. 검출기·브리지는 순수 Python + MQTT입니다.

### 4-1. 메시지 타입 — **ROS2 배포판마다 다름**

```python
self._pub = self.create_publisher(TwistStamped, self.cmd_out, 10)
```

**ROS2 Jazzy의 `turtlebot3_node`는 `TwistStamped`를 요구합니다.**
다른 배포판·다른 로봇에서는 보통 `Twist`입니다.

| 배포판 / 로봇 | `/cmd_vel` 타입 |
|---|---|
| Jazzy + TurtleBot3 | `TwistStamped` |
| Humble 이하 / 대부분의 로봇 | `Twist` |

`Twist`를 쓰는 로봇이면 `cliff_guard_node.py`에서 `TwistStamped` → `Twist`로
바꾸고 `header` 설정 두 줄을 지우면 됩니다 (`_tick_impl`, 예외 처리 블록 양쪽).

입력 쪽은 파라미터로 전환 가능합니다:
```bash
ros2 run ... --ros-args -p cmd_in_stamped:=true
```

### 4-2. 토픽 이름

```yaml
guard:
  cmd_in:  /cmd_vel_manual    # 조이스틱 출력
  cmd_out: /cmd_vel           # 로봇 입력
  state_topic: /cliff/state
```

새 로봇의 토픽 이름에 맞추세요. **중요한 건 배선 순서입니다** — 조종 입력이
이 노드를 **거쳐서** 모터로 가야 안전 개입이 걸립니다.

```
조이스틱 → /cmd_vel_manual → [cliff_guard_node] → /cmd_vel → 모터
                                     ↑
                            MQTT 안전 지시
```

조이스틱이 `/cmd_vel`로 직접 발행하면 **개입이 무시됩니다.** teleop 쪽에
리맵을 걸어야 합니다:
```bash
ros2 run teleop_twist_joy teleop_node -r /cmd_vel:=/cmd_vel_manual
```

### 4-3. ROS_DOMAIN_ID

현재 `40`. 새 로봇의 값에 맞추세요. **다르면 토픽이 서로 안 보입니다.**

### 4-4. 조이스틱 축 번호 (`config/teleop_8bitdo.yaml`)

8BitDo Micro 전용 실측값입니다 (`axis_linear.x: 7`, `axis_angular.yaw: 6`).
**패드가 바뀌면 거의 확실히 다릅니다.** 실측하세요:

```bash
ros2 topic echo /joy --field axes
```

`joy_node`는 반드시 `autorepeat_rate`를 주고 띄우세요 — 없으면 버튼을 눌러도
메시지가 한 번만 나가고, 가드의 "0.5초 입력 없음 = 정지" 규칙에 걸립니다.

```bash
ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0
```

---

## 5. 그대로 가져가도 되는 것

로봇과 무관하므로 **손대지 마세요.**

| 영역 | 내용 |
|---|---|
| 판정 알고리즘 | `baseline` / `jump` / `geometric` 세 방식 |
| 오탐 억제 | 공간 군집(4-이웃) · 시간 투표(M/N) · 해제 히스테리시스 |
| 상태머신 | `CLEAR → STOP → DETOUR` 전이, 우회 방향 래치 |
| 안전 설계 | LWT 기반 링크 감시, `DEGRADED` / `FAULT` 구분 |
| MQTT 토픽 구조 | `wheelchair/tof/*`, `wheelchair/safety/command` |
| 검증 도구 | `test_logic.py`(25종), `replay.py`, `tof_web.py` |

### 유지해야 할 안전 설계 3가지

옮기는 과정에서 **이 세 가지는 깨뜨리지 마세요.** 실제 사고를 막는 장치입니다.

1. **`DEGRADED` ≠ `FAULT`**
   센서가 불확실하면 정지가 아니라 **30% 감속**입니다. 움직이지 못하는
   휠체어도 위험하기 때문입니다 (횡단보도 한가운데서 멈추면 더 위험).
   센서 두절이 확정됐을 때만 `FAULT`로 정지합니다.

2. **정지 중에도 후진·회전은 허용** (`allow_reverse_when_stopped: true`)
   막는 것은 **전진뿐**입니다. 이게 없으면 낭떠러지 앞에서 사용자가 갇힙니다.

3. **우회 방향 래치** (`cliff_bridge.py`의 `self.detour_side`)
   한 번 정한 우회 방향은 위험이 해제될 때까지 바꾸지 않습니다.
   없으면 `LEFT→RIGHT→LEFT`로 0.5초마다 뒤집혀 제자리에서 떨게 됩니다
   (실제로 겪었던 문제).

---

## 6. 이식 체크리스트

새 로봇에서 순서대로 확인하세요.

### 설치
- [ ] `pip3 install pyserial paho-mqtt pyyaml numpy`
- [ ] `sudo apt install mosquitto mosquitto-clients`
- [ ] 브로커 기동 확인 — `ss -ltn | grep 1883`

### 설정
- [ ] `mqtt.host` → 새 브로커 IP (고정 IP 권장)
- [ ] `sensor.port` → **by-id 경로**로 지정
- [ ] `detect.center_band_m` → 로봇 반폭 + 여유
- [ ] `guard.detour_*` → 새 로봇 속도 (휠체어는 대폭 감속)
- [ ] `command.min_stop_s` → 1.5~2.0초
- [ ] `guard.cmd_in` / `cmd_out` → 새 로봇 토픽명
- [ ] ROS_DOMAIN_ID 일치
- [ ] `/cmd_vel` 메시지 타입 확인 (`Twist` vs `TwistStamped`)

### 센서
- [ ] 펌웨어 출력 포맷이 `R0 ... R7` 텍스트인지 확인
- [ ] `row0_is_top` / `col0_is_left` 실측 확인 (**우회 방향 반대 방지**)
- [ ] 센서 최종 고정 후 `--calibrate 60` 실행

### 검증 — 순서대로
- [ ] `python3 tools/test_logic.py` → 25종 통과 (브로커 불필요)
- [ ] `python3 tools/tof_web.py` → 브라우저에서 숫자·방향 확인
- [ ] **평지에서 CLEAR 유지** — 최소 60프레임 오탐 0
- [ ] **실제 단차에서 STOP** — 반복 재현되는지
- [ ] 조이스틱 전진 → 단차 앞 정지 확인 (**빈 차로 먼저**)
- [ ] 정지 중 후진·회전 되는지 확인
- [ ] 검출기 강제 종료 → `FAULT` 뜨는지 (LWT 동작 확인)

### 실차 적용 전 추가 작업
- [ ] **제동거리 대응** (3-3장) — 속도 비례 정지거리 또는 속도 상한
- [ ] MQTT 인증 활성화 (현재 `allow_anonymous true` — 개발용)

---

## 7. 알려진 한계

정직하게 적습니다. 실차 적용 시 보완이 필요한 부분입니다.

| 항목 | 현재 상태 | 영향 |
|---|---|---|
| **제동거리 미고려** | 속도와 무관하게 동일 문턱 | 고속 주행 시 **감지해도 늦음** |
| 센서 1개 | 전방만 감시 | 측면 낙차 사각지대 |
| 기준면이 자세에 묶임 | 차체 기울면 재캘리브 필요 | 탑승자 체중·노면 경사 영향 |
| 낙차 깊이 모름 | `baseline` 모드는 `depth_m: null` | 5cm 턱과 1m 낭떠러지를 구분 못 함 |
| MQTT 무인증 | `allow_anonymous true` | 개발용. 실차는 인증 필수 |
| 검은 바닥 오검출 가능성 | 무응답을 낙차로 해석 | `blind_min_valid_rate`로 완화했으나 잔존 |

### 센서 3개로 늘릴 때

`cliff_bridge.py`가 `sensor.id` 로 구분해 융합하도록 확장하면 됩니다.
`config/cliff_config.yaml`의 `sensor:` 블록을 리스트로 바꾸고 `yaw_deg`만 다르게
준 뒤 detector를 3개 띄우는 구조입니다 (`README.md` 참고).

장치 경로는 **반드시 by-id**로 고정하세요 — `/dev/ttyACM*` 번호는 재부팅마다
뒤섞입니다. 참고 구현이 `~/ros2_tof/run_tof_publishers.sh` 에 있습니다
(Pico 3대를 by-id로 고정 + 무응답 시 DTR 토글 자동 복구).

---

## 8. 파일별 이식 영향도

| 파일 | 이식 시 |
|---|---|
| `rapa2/tof_cliff_detector.py` | 펌웨어 포맷 다르면 `SerialFrameSource`만 수정 |
| `rapa2/cliff_bridge.py` | **수정 불필요** (설정만) |
| `rapa1/cliff_guard_node.py` | 메시지 타입·토픽명 확인 |
| `config/cliff_config.yaml` | **전면 재검토** |
| `config/ground_baseline.yaml` | **삭제 후 재생성** |
| `config/teleop_8bitdo.yaml` | 패드 바뀌면 축 번호 재실측 |
| `tools/*` | 그대로 사용 가능 |
