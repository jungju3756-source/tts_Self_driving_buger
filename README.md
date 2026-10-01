# 단차(낭떠러지) 감지 및 회피 모듈 — MQTT 기반 (Non-ROS2)

전동휠체어 운전자보조시스템(ADAS) 중 **단차 추락 방지** 부분입니다.
평소에는 조이스틱 입력을 그대로 따르고, 단차를 감지했을 때만 개입합니다.
좁은 통로/VFH 경로 탐색은 이 모듈의 범위가 아닙니다.

## 구성

```
                 라파2 (192.168.0.39)                    라파1 (192.168.0.37)
 ┌───────────────────────────────────────────┐      ┌──────────────────────────┐
 │  ToF (VL53L8CX 8x8, 전방 하향 장착)        │      │  8BitDo 패드 (Bluetooth) │
 │            │ USB 시리얼                    │      │       │                  │
 │            ▼                               │      │       ▼                  │
 │  ① tof_cliff_detector.py                   │      │  joy → teleop_twist_joy  │
 │     지면 끊김 판정 + 공간/시간 필터         │      │       │ /cmd_vel_manual  │
 │            │                               │      │       ▼                  │
 │            │ MQTT  wheelchair/tof/cliff    │      │  cliff_guard_node.py     │
 │            ▼                               │      │  (하드 오버라이드)       │
 │  ② cliff_bridge.py                         │      │       │ /cmd_vel         │
 │     상태머신 → 회피 지시                    │      │       ▼                  │
 │            │                               │      │  turtlebot3_node         │
 └────────────┼───────────────────────────────┘      │       ▼   OpenCR         │
              └── MQTT wheelchair/safety/command ────►│  모터                    │
                                                      └──────────────────────────┘
                              브로커(mosquitto)는 라파1에 둔다
```

| 파일 | 위치 | 역할 |
|---|---|---|
| `rapa2/tof_cliff_detector.py` | 라파2 | **산출물 ①** ToF 8x8 → 단차 판정 → MQTT 발행 |
| `rapa2/cliff_bridge.py` | 라파2 | **산출물 ②** 단차 이벤트 → 회피 지시 상태머신 |
| `rapa1/cliff_guard_node.py` | 라파1 | 회피 지시로 조이스틱 명령을 덮어쓰는 ROS2 노드 |
| `config/cliff_config.yaml` | 공통 | 세 프로세스가 공유하는 설정 |
| `rapa1/drive_monitor.py` | 라파1 | 조이스틱 → 개입 → 모터를 한 줄로 보는 실시간 모니터 |
| `tools/tof_web.py` | 브로커 있는 곳 | **웹 시각화** — 브라우저에서 8x8 히트맵 + 판정 + 지시 |
| `tools/replay.py` | 아무 곳 | 녹화 프레임으로 판정 검증 / 기준면 생성 |
| `tools/aim.py` | 라파2 | **센서 조준 도우미** — 화면 보며 각도/비틀림 맞춤 |
| `tools/fit_mount.py` | 라파2 | 장착 형상 역산 — 평지 데이터로 각도·높이·행방향 측정 |
| `tools/diag.py` | 라파2 | 셀 단위 진단 (측정거리 / 기준면 / 깊이 / 판정) |
| `tools/mqtt_monitor.py` | 아무 곳 | 터미널 모니터 (8x8 격자 + 이벤트 + 지시) |
| `tools/test_logic.py` | 아무 곳 | 브로커 없이 상태머신 25종 검증 |

`aim.py` / `fit_mount.py` / `diag.py` 는 `geometric` 모드용 보조 도구입니다.
`jump` 모드만 쓸 거면 없어도 됩니다.

---

## 판정 원리

판정 방식이 세 가지 있습니다. `config/cliff_config.yaml` 의 `detect.mode` 로 고릅니다.

| 모드 | 필요한 것 | 알 수 있는 것 | 언제 쓰나 |
|---|---|---|---|
| **`baseline`** (기본) | 평지에서 6초 정지 1회 | "이 셀치고 유난히 멀다" | 센서를 고정한 뒤 — 오탐이 가장 적음 |
| `jump` | 없음 | "여기서 지면이 끊겼다" | 기준면을 뜰 시간도 없을 때 |
| `geometric` | 장착 각도·높이 실측 + 평지 캘리브레이션 | "몇 cm 꺼졌다" | 물리적 낙차 깊이가 필요할 때 |

실기기 검증 결과 **`baseline` 모드**를 기본으로 삼았습니다. `jump` 모드는 평평한
바닥에서 60프레임 중 60프레임을 낭떠러지로 오판했고, `baseline` 모드는 같은
데이터에서 0프레임이었습니다. 이유는 아래 `jump` 절의 한계 설명에 있습니다.

### `baseline` — 셀마다 자기 기준값 (기본)

평지에서 각 셀이 평소 몇 mm를 찍는지 외워두고(`--calibrate 60`, 6초),
운전 중에는 그 셀 기준값의 `base_ratio`(기본 1.5)배를 넘거나 평소 잘 잡히던
셀의 응답이 사라졌을 때만 낙차로 봅니다.

```
  R5 셀 기준값 1591mm → 이 셀은 1591mm 가 '정상', 2387mm 넘어야 낙차
  R0 셀 기준값  268mm → 이 셀은  268mm 가 '정상',  402mm 넘어야 낙차
```

같은 프레임 안의 다른 셀과 비교하지 않기 때문에, **센서가 비뚤게 달렸든
위쪽 행이 지면을 스치듯 보든 셀별 기준값이 알아서 흡수합니다.** 장착 각도·높이
실측은 필요 없고, 평지에 두고 6초 정지 한 번이면 끝입니다.

> **주의** — 기준면은 그때의 센서 자세에 묶입니다. 브래킷을 건드리거나 각도를
> 바꾸면 반드시 다시 뜨세요. 6초면 됩니다.
> ```bash
> python3 rapa2/tof_cliff_detector.py --calibrate 60   # 평지에 두고 정지 상태로
> ```
> 안 뜨면 위험 방향으로 틀리지는 않습니다 — 자세가 바뀌어 거리가 *가까워*지는
> 쪽은 절대 낙차로 판정하지 않기 때문입니다. 다만 감도가 떨어집니다.

### `jump` — 지면이 끊기는 지점 찾기

각 열을 **가까운 행 → 먼 행** 순서로 훑습니다. 평평한 바닥이라면 거리는 완만하게
늘어납니다. 어느 지점에서 갑자기 `jump_ratio`(기본 2.5)배 넘게 뛰거나 응답이
사라지면, 거기서 지면이 끊긴 것이고 그 너머는 전부 허공으로 봅니다.

```
  R0  196   ─┐
  R1  214    │ 완만하게 증가 → 지면
  R2 1146   ─┴─ 5.4배 급증 → 여기가 낭떠러지
```

장착 각도·높이를 몰라도 되고 캘리브레이션도 필요 없습니다. 대신 "몇 cm 꺼졌는지"
같은 물리량은 알 수 없습니다.

**이 모드의 한계 — 실기기에서 확인됨.** 배열 위쪽 행은 수평선에 가까워서 *평평한
바닥에서도* 거리가 4~5배 뜁니다. 게다가 센서가 조금이라도 비틀려(롤) 달리면 같은
행 안에서도 좌우 열이 두 배씩 갈립니다. 실측 예:

```
        C0    C1    C2    C3    C4    C5    C6    C7
  R5   653   723   814  1622  1591  1493  1652  1450   ← 전부 평평한 바닥
  R3   380   375   370   369   366   361   354   349
```

왼쪽 열은 653mm, 오른쪽 열은 1652mm — **같은 평지인데 2.5배** 차이 납니다.
`max_ground_mm`(기본 800mm) 게이트로 일부는 막히지만, 위 예처럼 직전 행이
800mm 안쪽이면 그대로 통과해 오탐이 납니다. 문턱값을 올리면 진짜 낙차도 같이
놓칩니다. **하나의 문턱값으로는 이 정상 기하와 실제 낙차를 가를 수 없습니다.**
그래서 셀마다 기준값을 갖는 `baseline` 모드가 기본입니다.

### `geometric` — 꺼진 깊이를 계산

센서는 지면에서 `h` 높이에 수평 기준 `tilt` 만큼 아래로 기울어 달려 있습니다.
각 셀은 고유한 시선 벡터 `v`를 가지므로, **지면이 평평하다면** 그 셀이 측정해야
할 거리는 기하학적으로 결정됩니다.

```
d_expected = h / (-v_z)
```

실제 측정거리 `d`가 이보다 멀면 그 시선은 지면 아래를 통과한 것이고,
꺼진 깊이는 수직 성분만 뽑으면 됩니다.

```
depth = d * (-v_z) - h        [m]   (양수 = 지면보다 아래)
```

**"거리가 멀어졌다"가 아니라 "실제로 몇 cm 꺼졌다"로 판정**하므로, 행마다
기대거리가 다른 문제(윗행은 멀리, 아랫행은 가까이 봄)를 자동으로 흡수합니다.

이 모드를 쓰려면 `tools/fit_mount.py`로 장착 형상을 역산하고 `--calibrate`로 평지
기준면을 떠야 합니다. **센서를 건드리면 둘 다 다시 해야 합니다** — 각도가 8~10°만
틀어져도 기준면 전체가 무의미해집니다(실기기에서 확인).

### 오탐 억제 — 두 모드 공통

| 단계 | 내용 | 파라미터 |
|---|---|---|
| 공간 | 4-이웃 연결 3셀 이상이어야 인정 | `min_cluster` |
| 시간 | 최근 5프레임 중 3회 이상 검출돼야 확정 | `window`, `set_votes` |
| 해제 | 연속 8프레임 깨끗해야 해제 (히스테리시스) | `clear_frames` |
| 거리 | (jump) 직전 지면이 80cm 안쪽일 때만 급변 인정 | `max_ground_mm` |
| 깊이 | (geometric) 5cm 이상 꺼져야 낙차 셀 | `min_cliff_depth_mm` |

무응답 셀(`----`)은 낭떠러지의 전형적 증상이지만 **검은 카펫/젖은 바닥도 동일**하므로,
무응답만으로 판정할 때는 더 큰 군집(`invalid_min_cluster: 5`)을 요구합니다.

여기에 더해 **셀별 응답률 필터**가 있습니다. 캘리브레이션 때 각 셀이 평지에서 몇 %나
값을 돌려주는지 기록해 두고, `blind_min_valid_rate`(기본 0.9) 미만인 셀은 '무응답 =
낭떠러지' 규칙에서 제외합니다. 입사각이 얕은 먼 행은 평지에서도 응답이 자주 끊겨서,
이 필터가 없으면 **평평한 바닥에서 오탐이 납니다**(실기기에서 확인). 제외된 셀도 값이
돌아올 때의 깊이 판정에는 계속 쓰이므로, 감지 능력을 잃지는 않습니다.

---

## 회피 지시 (PDF 슬라이드 3 준수)

**후진하지 않습니다.** 전방 단차는 정지 후 우회, 측면 단차는 전진 유지 + 반대편 보정입니다.

| action | 발생 조건 | 라파1 동작 |
|---|---|---|
| `CLEAR` | 위험 없음 | 조이스틱 그대로 통과 |
| `STOP` | 진행 통로(±12 cm) 안에 단차 | 전진 금지. **후진·제자리회전은 허용**(탈출로) |
| `DETOUR_LEFT/RIGHT` | 정지 1초 후, 지면이 성한 쪽 | 사용자가 전진을 원할 때만 저속 우회 |
| `CORRECT_LEFT/RIGHT` | 측면에만 단차 | 전진 속도 유지 + 각속도에 반대편 보정 |
| `DEGRADED` | 센서/링크 이상 (위험은 없었음) | 정지가 아니라 속도 30% 제한 |
| `FAULT` | **위험 감지 중** 링크 상실 | `STOP`과 동일 |

`DEGRADED`와 `FAULT`를 나눈 이유: 휠체어는 못 움직이는 것도 위험합니다. 위험이
확인되지 않은 상태에서 센서가 죽었다고 탑승자를 가둬서는 안 되므로 감속으로 강등하고,
위험이 살아 있는 채로 눈을 잃었을 때만 정지를 유지합니다.

### 안전 설계

- **우회 방향은 한 번 정하면 위험이 해제될 때까지 고정**됩니다. `free_side`는 프레임마다
  흔들릴 수 있는데, 그때마다 방향을 뒤집으면 좌우로 떨기만 하고 빠져나가지 못합니다.
  (실기기 로그에서 실제로 0.5초 간격으로 좌우가 뒤집히는 것을 확인하고 넣은 처리입니다.)
- 지시는 이벤트 발생 시점뿐 아니라 **10 Hz로 계속 재전송**됩니다. 라파1은 신호가
  끊기는 것 자체(`watchdog_s`)를 이상으로 감지합니다.
- detector와 bridge 모두 MQTT **LWT(유언장)**를 등록합니다. 프로세스가 죽으면
  브로커가 대신 `offline`/`FAULT`를 발행합니다.
- 안전 개입은 항상 조이스틱보다 우선합니다(하드 오버라이드).

---

## 설치

```bash
# 라파1, 라파2 공통
sudo apt install -y python3-paho-mqtt python3-serial python3-yaml python3-numpy

# 라파1 (브로커)
sudo apt install -y mosquitto mosquitto-clients
echo -e "listener 1883 0.0.0.0\nallow_anonymous true" | sudo tee /etc/mosquitto/conf.d/cliff.conf
sudo systemctl enable --now mosquitto
```

`allow_anonymous true`는 개발용입니다. 실제 차량에 올릴 때는 사용자/비밀번호를
설정하고 `config/cliff_config.yaml`의 `mqtt.username/password`를 채우세요.

## 실행

```bash
# ── 라파2 ──────────────────────────────────────────
python3 rapa2/tof_cliff_detector.py -v      # 감지기
python3 rapa2/cliff_bridge.py -v            # 지시 생성기

# geometric 모드를 쓸 때만 — 평평한 바닥에 세워둔 채 최초 1회
python3 tools/fit_mount.py                       # 장착 형상 역산 → config 에 입력
python3 rapa2/tof_cliff_detector.py --calibrate 60

# ── 라파1 ──────────────────────────────────────────
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=40
ros2 launch turtlebot3_bringup robot.launch.py
ros2 run joy joy_node --ros-args -p autorepeat_rate:=20.0
ros2 run teleop_twist_joy teleop_node \
    --ros-args --params-file config/teleop_8bitdo.yaml -r /cmd_vel:=/cmd_vel_manual
python3 rapa1/cliff_guard_node.py

# 바퀴가 손으로 헛돌면 토크를 켠다
ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"

# ── 확인 ───────────────────────────────────────────
python3 rapa1/drive_monitor.py     # 조이스틱 → 개입 → 모터 한 줄로
python3 tools/mqtt_monitor.py      # 8x8 격자 + 이벤트 + 지시 (터미널)
python3 tools/tof_web.py           # 8x8 히트맵 (브라우저) → http://192.168.0.37:8080
```

### 하드웨어 없이 검증

```bash
python3 rapa2/tof_cliff_detector.py --source sim -v   # 합성 낭떠러지 생성
python3 tools/test_logic.py                           # 상태머신 25종 검증
```

### 웹 시각화 (`tools/tof_web.py`)

브로커가 있는 기기(라파1)에서 띄우면 같은 공유기에 붙은 PC·폰 브라우저에서 봅니다.
표준 라이브러리 + `paho-mqtt` 만 쓰며, 프레임은 SSE(Server-Sent Events)로 밀어냅니다.

| 화면 요소 | 의미 |
|---|---|
| 칸 안의 숫자 | 그 셀의 측정거리 [mm], `----` 는 무응답 |
| 칸 색 | 빨강 = 가까움 → 청록 = 멂 (범위는 화면 우상단에서 조절) |
| 노란 사각형 | ROI — 판정에 실제로 쓰는 영역 (`max_valid_range_mm` 로 결정) |
| 흰 테두리 | 그 프레임에서 낭떠러지로 판정된 셀 |
| 빗금 + 흰 테두리 | 무응답인데 낭떠러지로 본 셀 |
| 상단 배너 | 라파1로 나가는 안전 지시 (CLEAR / STOP / DETOUR_*) |
| ROI 최소거리 | ROI 안에서 가장 가까운 측정값 + 최근 추이 |

`--record out.jsonl` 을 붙이면 프레임을 그대로 파일에 남깁니다 (보고서·재생용).

> **왜 GStreamer가 아닌가**
> GStreamer는 영상 파이프라인용입니다. ToF 한 프레임은 숫자 64개(약 1KB)라,
> 영상으로 인코딩해 보내면 받는 쪽에서는 픽셀만 남아 숫자를 다시 읽을 수 없고
> 지연도 커집니다. 숫자를 숫자 그대로 보내는 지금 방식이 가볍고 정확합니다.
> GStreamer는 나중에 카메라 영상 위에 ToF를 겹칠 때 쓰면 됩니다.

---

## 장착 형상 — `geometric` 모드를 쓸 때만 필요

`jump` 모드(기본)에서는 이 값들이 쓰이지 않습니다.

`tools/fit_mount.py`가 평지 데이터에서 자동으로 역산합니다. 눈대중으로 각도를 재지
마세요. 실제로 "45°로 달았다"고 한 센서가 측정해보니 13.8°였습니다.

| 키 | 확정값 | 확인 방법 |
|---|---|---|
| `sensor.mount_height_m` | **0.190** | `fit_mount.py` (잔차 5.0 mm) |
| `sensor.tilt_deg` | **13.8** | `fit_mount.py` |
| `sensor.row0_is_top` | **false** | `fit_mount.py` (반대로 두면 잔차 69 mm) |
| `sensor.col0_is_left` | **true** | 왼쪽에 물체 놓고 `diag.py`로 열 확인 |

**센서를 건드리면 반드시 `fit_mount.py` → `--calibrate`를 다시 하세요.** 각도가
8~10° 틀어지면 기준면 전체가 무의미해져 오탐/미탐이 함께 발생합니다.

`col0_is_left`가 뒤집히면 측면 단차에서 보정이 **낭떠러지 쪽으로** 나갑니다.
반드시 물체 테스트로 확인하세요 (`fit_mount.py`는 좌우를 알아낼 수 없습니다 —
평지는 좌우 대칭이라 원리적으로 불가능).

## 센서 3개로 확장할 때

`config/cliff_config.yaml`의 `sensor:` 블록을 복사해 `yaw_deg`만 바꾸고
(예: 좌 `+30`, 정면 `0`, 우 `-30`), detector를 3개 띄운 뒤 각각 다른 `sensor.id`를
주면 됩니다. 기하 모델이 yaw를 이미 반영하므로 판정 코드는 손댈 필요가 없습니다.
`cliff_bridge.py`는 여러 센서의 이벤트를 받아 **가장 위험한 것 하나**를 고르도록
`_on_message`에서 sensor id별로 나눠 담는 처리만 추가하면 됩니다.

## MQTT 토픽

| 토픽 | 발행자 | QoS | 내용 |
|---|---|---|---|
| `wheelchair/tof/cliff` | detector | 1 | `{cliff, zone, n_cells, range_m, depth_m, free_side, cells, votes}` |
| `wheelchair/tof/frame` | detector | 0 | 8x8 원본 거리 (디버그, 3 Hz) |
| `wheelchair/tof/status` | detector | 1 retained | `online` / `no_data` / `offline`(LWT) |
| `wheelchair/safety/command` | bridge | 1 | `{action, reason, override, gain, valid_until, zone, range_m}` |
