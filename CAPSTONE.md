# 페스코 캡스톤 — LLM 음성 대화형 자율주행 로봇 (TurtleBot3 Waffle Pi)

> **이 문서 하나로 처음부터 똑같이 구성할 수 있게** 쓴 인수인계 문서입니다.
> 작성: 2026-10-02 · 기준 기기: 라파1 (`jungju4`, Raspberry Pi 5) · 저장소: `github.com/jungju3756-source/tts_Self_driving_buger`
>
> 처음 읽는다면 **0장 → 1장(블록도) → 6장(설치) → 8장(사용법)** 순서로 보세요.
> 막히면 **12장(문제 해결)** 에 실제로 겪은 문제와 해결법이 다 있습니다.

---

## 목차

0. [한눈에 보기](#0-한눈에-보기)
1. [블록도](#1-블록도)
2. [하드웨어 · 네트워크](#2-하드웨어--네트워크)
3. [소프트웨어 · 버전](#3-소프트웨어--버전)
4. [음성: STT · LLM · TTS 무엇을 왜 썼나](#4-음성-stt--llm--tts-무엇을-왜-썼나)
5. [저장소 구조](#5-저장소-구조)
6. [설치 — 로봇(Pi 5) 맨바닥부터](#6-설치--로봇pi-5-맨바닥부터)
7. [설치 — Windows PC (WSL RViz · 브라우저)](#7-설치--windows-pc-wsl-rviz--브라우저)
8. [사용법 (매일 하는 순서)](#8-사용법-매일-하는-순서)
9. [노드 · 토픽 · 명령 인터페이스](#9-노드--토픽--명령-인터페이스)
10. [설정값과 바꾼 이유](#10-설정값과-바꾼-이유)
11. [검증 결과](#11-검증-결과)
12. [문제 해결 (실제로 겪은 것)](#12-문제-해결-실제로-겪은-것)
13. [남은 일](#13-남은-일)
14. [부록: 명령 모음](#14-부록-명령-모음)

---

## 0. 한눈에 보기

### 과제 (RFP 요약)
- **기업**: ㈜페스코(군산) · **로봇**: ROBOTIS TurtleBot3 **Waffle Pi**
- LiDAR SLAM(**Cartographer**)으로 지도 → **Nav2** 자율주행
- "여기를 ○○로 저장해", "○○로 가줘" 같은 **음성 명령**으로 장소 저장·이동, 도착/실패 음성 안내
- 비상정지는 **LLM 을 거치지 않는** 안전 경로
- **정량 목표**: 명령 해석 ≥ 90 %, 음성 응답 ≤ 3 초, 도달 오차 ±0.25 m

### 지금 되는 것 (2026-10-02)

| 단계 | 내용 | 상태 |
|---|---|---|
| 0 | Pi 에 ROS 2 Jazzy · Cartographer · Nav2 설치 | ✅ |
| 1 | Windows WSL2 RViz ↔ Pi 통신 | ✅ |
| 2 | Cartographer 지도 (`~/maps/lab_v2`) | ✅ |
| 3 | Nav2 + AMCL 주행 | ✅ 1→2→3→4 **4/4 성공**, 오차 평균 0.069 m (AMCL 기준) |
| 4 | 장소 저장 (RViz 클릭 / **패드 A 버튼** / 웹 / CLI), 마커 | ✅ |
| — | **웹 조종석** `https://<로봇IP>:8443` — 지도, 클릭 이동, **벽 그리기**, 음성, 대화창, 지연 시간 표시 | ✅ |
| 5·6 | **Gemini Live 음성** (듣기·판단·말하기 한 세션) | ✅ 실 API 확인: 첫 음성 **≈1.2 초** |
| 7 | 비상정지(키워드·버튼·스페이스), 되묻기 | ✅ 기본 / 호출어(wake word) 미구현 |
| 8 | 정량 평가 (발화 100개, 테이프 실측) | ⬜ |

### 하루 사용 3줄 요약
```bash
# 로봇(Pi): 전부 한 번에 (bringup + 위치추정 + Nav2 + 패드 + 장소/벽 + 웹 서버)
ros2 launch ~/tts_Self_driving_buger/rapa1/nav.launch.py
# PC 브라우저(Chrome):  https://192.168.0.57:8443   → 장소 버튼 클릭 or 🎤 "3번으로 가줘"
```

---

## 1. 블록도

### 1-1. 전체 배치 (하드웨어 · 네트워크)

```
                        같은 공유기 (Wi-Fi 13205-2.4GHz, 192.168.0.0/24)
 ┌──────────────────────── 로봇: TurtleBot3 Waffle Pi ────────────────────────┐
 │  Raspberry Pi 5 (8GB)  Ubuntu 24.04 + ROS 2 Jazzy     IP 192.168.0.57     │
 │   ├─ USB ── OpenCR 1.0 ── Dynamixel XM430 ×2 (바퀴), IMU, 부저               │
 │   ├─ USB ── LDS-02 LiDAR (CP2102 USB-UART)                                 │
 │   └─ Bluetooth ── 8BitDo Micro 게임패드 (D 모드)                            │
 │                                                                            │
 │   ROS 2 노드 전부 + 웹 서버(web/server.py, :8443 HTTPS/WSS)                  │
 └──────────────▲───────────────────────────────▲───────────────────────────┘
                │ DDS (ROS_DOMAIN_ID=40)        │ HTTPS + WebSocket
                │                               │
 ┌──────────────┴──── Windows 11 PC ────────────┴───────────────────────────┐
 │  WSL2 Ubuntu 24.04 + Jazzy  (mirrored 네트워크)   ← RViz2 (선택, 디버깅용)  │
 │  Chrome  https://192.168.0.57:8443   ← 조종 웹 페이지 (주 사용 화면)         │
 │     └ 웹캠 마이크 · 스피커  (음성 입출력은 PC 에서)                          │
 └────────────────────────────────────────────────────────────────────────────┘
                                                │ wss (서버가 중계, API 키는 Pi 에만)
                                       Google Gemini Live API (gemini-3.8-live)
```

### 1-2. ROS 2 노드 구성 (`rapa1/nav.launch.py` 하나로 다 뜸)

```
 [센서/구동]                       [위치 추정 · 주행]                      [응용]
 OpenCR ─► turtlebot3_node ──/odom, TF odom→base_footprint──┐
                 ▲ /cmd_vel (TwistStamped)                    │
                 │                                            ▼
 LDS-02 ─► ld08_driver ──/scan──────────────────────────►  AMCL ── TF map→odom
                                                              ▲   /amcl_pose ─► pose_keeper (마지막 위치 저장/복원)
 ~/maps/lab_v2.yaml ─► map_server ──/map──────────────────────┤
                                                              ▼
                     wall_manager ──/keepout_filter_mask──► Nav2 (planner·controller·bt_navigator
                       ▲ /walls/cmd   /costmap_filter_info     ·behavior·velocity_smoother·collision_monitor)
                       │                                          │ navigate_to_pose (action)
 8BitDo ─► joy_node ─/joy─► teleop_twist_joy ─/cmd_vel_joy─► pad_gate ─/cmd_vel─► turtlebot3_node
                │                                          (움직일 때만 통과)        ▲
                └─/joy─► place_manager (A 1초 → 현재 위치 저장, 로봇 '삑')           │ Nav2 → /cmd_vel
                           ▲ /places/cmd  ▼ /places/list, /places/markers           │
                           │                                                         │
                     web/server.py (rclpy 노드 + tornado 웹 서버) ──────────────────┘
                           ▲ WebSocket /ws          ▲ wss
                        브라우저 페이지         Gemini Live
```

### 1-3. 음성 파이프라인 — 두 가지 모드

**(A) Gemini Live 모드 (캡스톤 본선, RFP 의 LLM 음성)**
```
 웹캠 마이크 ─► 브라우저 AudioWorklet ─► 16 kHz PCM16, 100 ms 묶음 ─WSS─► server.py ─wss─► Gemini Live
   (로봇이 말하는 중엔 업로드 중단 = half-duplex, 자기 목소리 재인식 방지)          │  STT+LLM+TTS 한 세션
                                                                                     │
 스피커 ◄─ 브라우저 24 kHz 재생 ◄─WSS(바이너리)─ server.py ◄── 음성(24 kHz PCM) ──────┤
                                                    ▲                                │
              대화창 자막 ◄─ input/output 전사 ──────┤◄── inputTranscription ─────────┤
                                                    │                                │
            Nav2 ◄─ goto(장소) ◄─ 장소명 재검증 ◄────┴◄── toolCall navigate_to(…) ────┘
                     (intent.resolve_place: 등록 안 된 이름은 절대 이동 안 함)
   비상정지: inputTranscription 에 "멈춰/정지/그만" 이 보이면 LLM 응답을 기다리지 않고 즉시 stop
   출발 시점: "○○로 갈게요" 음성 재생이 끝난 뒤 (안 나오면 8초 안전망)
```

**(B) 브라우저 음성 모드 (API 키 없이, 예비)**
```
 마이크 ─► Chrome Web Speech API (ko-KR, STT) ─► 문장 ─WSS─► server.py ─► intent.parse_command (규칙)
                                                                           ├ go → Nav2
 스피커 ◄─ Chrome speechSynthesis (TTS) ◄─ 응답 문장 ◄────────────────────────┘ stop/where/list/ask
```

### 1-4. "3번으로 가줘" 한 번의 흐름 (Gemini 모드)

```
 t=0      사용자 발화 끝 (브라우저가 마이크 레벨로 감지)
 +0.3     Gemini 자동 음성감지(침묵 600 ms)로 턴 종료 → 전사 "3번으로 가줘" (대화창 🎙 인식)
 +0.5     toolCall navigate_to{destination:"3번"} → 서버 resolve_place 확인 → pending_go=3번   (⚙ 판단)
 +1.2     Gemini 음성 "3번으로 갈게요" 첫 소리 재생                                           (🔊 응답)
 +2.6     재생 끝 → 브라우저 playback_idle → 서버 → NavigateToPose(3번)  🚀 출발
 …        이동 중: 모델 음성 차단(nav_silent), "멈춰"는 즉시 정지
 도착     Nav2 SUCCEEDED → 서버가 "[시스템 안내] 3번에 도착했습니다" 주입 → Gemini 가 말로 안내
```

---

## 2. 하드웨어 · 네트워크

| 항목 | 값 | 비고 |
|---|---|---|
| 로봇 | TurtleBot3 **Waffle Pi** | 폭 0.306 m, 길이 0.281 m, 최고속도 0.26 m/s |
| SBC | Raspberry Pi 5 8GB | hostname `jungju4` (= 라파1) |
| 제어보드 | OpenCR 1.0 | `/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00` |
| LiDAR | **LDS-02** (LD08) | CP2102, 115200 bps, 회전당 205~209 점 |
| 게임패드 | 8BitDo Micro | **D 모드**, MAC `E4:17:D8:E6:37:F9` (K 모드는 MAC 다름) |
| 원격 PC | Windows 11 + WSL2 Ubuntu 24.04 | 사용자 `aim88`, WSL IP 192.168.0.30 / .7 (유선+Wi-Fi) |
| 마이크/스피커 | **PC 웹캠 마이크**, PC 스피커 | 브라우저에서 선택 |
| 네트워크 | 같은 공유기 192.168.0.0/24 | 로봇 192.168.0.57 (고정 권장) |
| 학교망 주의 | 군산대 방화벽(SOAR)이 일부 사이트 차단 | 8bitdo.com 차단 확인. Google API 는 정상 |

---

## 3. 소프트웨어 · 버전

### 로봇 (Pi 5)
| 구분 | 이름 · 버전 |
|---|---|
| OS | Ubuntu **24.04.5** LTS (noble), 커널 6.8.0-raspi |
| ROS | ROS 2 **Jazzy** (ros-base 0.11.0) — RMW Fast DDS(기본) |
| TurtleBot3 | 소스 `ROBOTIS-GIT/turtlebot3` 브랜치 **jazzy** (`0c0be84`), `ld08_driver` jazzy (`00dba81`) → `~/turtlebot3_ws` |
| SLAM | `ros-jazzy-cartographer` 2.0.9004, `cartographer-ros` 2.0.9003 (slam_toolbox 2.8.5 도 설치돼 있음, 비교용) |
| 내비 | `ros-jazzy-navigation2` / `nav2-bringup` **1.3.13** (AMCL, DWB, KeepoutFilter) |
| 패드 | `ros-jazzy-joy` 3.3.0, `teleop-twist-joy` 2.6.5, `bluez` 5.72 |
| 웹 서버 | Python 3.12 + **tornado 6.4** (`python3-tornado`, rosbridge 의존성으로 이미 설치됨), numpy, PIL, yaml — **pip 설치 없음** |
| 음성 LLM | **Gemini Live API** `gemini-3.8-live` (v1beta BidiGenerateContent, 원시 WebSocket) |
| 기타 | openssl (자체 서명 인증서), mosquitto (기존 낭떠러지 모듈용) |

### Windows PC
| 구분 | 이름 |
|---|---|
| 브라우저 | **Chrome** (마이크·Web Speech·AudioWorklet) |
| WSL | Ubuntu 24.04 + ROS 2 Jazzy desktop (RViz2) — 선택 사항 |

---

## 4. 음성: STT · LLM · TTS 무엇을 왜 썼나

### 결론
| 역할 | 본선 (Gemini 모드) | 예비 (브라우저 모드, 키 불필요) |
|---|---|---|
| **STT** (말→글) | Gemini Live 내장 (`inputAudioTranscription`) | Chrome **Web Speech API** (`ko-KR`) |
| **LLM** (판단) | **Gemini Live `gemini-3.8-live`** + 함수 호출(tools) | 규칙 해석 `web/intent.py` |
| **TTS** (글→말) | Gemini Live 내장 음성 (`voiceName: Kore`, 24 kHz) | Chrome `speechSynthesis` (ko-KR) |
| 비상정지 | 전사 텍스트 키워드 → **LLM 안 거침** | 인식 중간 결과 키워드 → 즉시 |

### 왜 Gemini Live 하나로 (A안)
- STT → LLM → TTS 를 따로 세 번 호출하면 왕복이 쌓여 **3초 목표가 빠듯**. Live 는 한 WebSocket 에서 듣고·판단하고·말함.
- 팀 이전 프로젝트(RobotNav 안드로이드 앱)에서 이미 써 본 방식 → 겪은 문제(말하면서 출발, 이동 중 잡음 반응, 장소명 오인)의 해결책을 그대로 가져옴.
- 실측: **첫 음성 0.8~1.2 초**, 도구 호출 0.4~0.6 초 (11장).
- 분리형(B안: 단계별 호출)은 시간이 남으면 "통합형 vs 분리형 지연 비교" 실험으로 보고서에 사용.

### 왜 SDK 가 아니라 원시 WebSocket 인가
Pi 에 pip 가 없고(Ubuntu 24.04 는 시스템 pip 막힘), tornado 가 이미 있어서 `web/gemini_live.py` 가 프로토콜을 직접 처리한다 (setup → realtimeInput → serverContent/toolCall → toolResponse).

### 음성 동작 규칙 (RobotNav 에서 가져온 것)
1. **장소 이름만 말해도 이동** ("3번" = 3번으로 가). "~어디야" 처럼 위치만 물으면 바로 출발하지 않고 되묻기.
2. 모델이 고른 장소명은 **항상 서버에서 재검증** (`intent.resolve_place`) — 등록 안 된 이름·40자 초과·`<>` 포함은 거부. 숫자 접미사만 떼고 **한글은 자르지 않음** (RobotNav 의 "화장실→화장" 버그 수정판).
3. **안내 멘트가 다 재생된 뒤 출발** (dispatchAfterPlayback). Gemini 는 이동 요청 때 턴 종료가 두 번 오므로(도구 턴·음성 턴) **음성이 나온 뒤의 턴 종료**에서만 출발. 멘트가 안 나오면 **8초 안전망**.
4. **이동 중 침묵** (nav_silent): 모델이 뭐라 하든 오디오를 내보내지 않음. 도착/실패/정지 때 "[시스템 안내 · 최우선]…" 을 주입해 말하게 함.
5. **half-duplex**: 로봇이 말하는 동안 마이크 업로드 중단 (스피커 소리를 다시 듣지 않게).

### API 키
- 위치: **Pi 의 `~/.config/robot_web/gemini_api_key`** (권한 600, git 밖). `#` 줄은 주석, 키는 한 줄.
- 키 형식: 기존 `AIza…`(39자) 외에 **`AQ…`(53자)** 형식도 있음 — 서버는 형식으로 거르지 않고 실제 호출로 확인.
- 웹 페이지 음성 칸에서 "Gemini Live" 선택 시 키 입력란이 뜨며, 거기 넣어도 같은 파일에 저장.
- 발급: https://aistudio.google.com/apikey
- **절대 git·문서·채팅에 키를 남기지 말 것.**

---

## 5. 저장소 구조

`~/tts_Self_driving_buger` (원래 ToF 낭떠러지 감지 프로젝트 저장소에 캡스톤을 같이 넣음)

```
tts_Self_driving_buger/
├── CAPSTONE.md                ← 이 문서
├── SETUP.md                   새 로봇 맨바닥 설치 가이드 (낭떠러지 모듈 기준, 3장 참고)
├── config/
│   ├── cartographer_lds02.lua   Cartographer 설정 (ROBOTIS lds_2d 복사, 거리만 LDS-02 0.16~8 m)
│   ├── nav2_waffle_pi.yaml      Nav2 설정 (ROBOTIS waffle_pi 복사 + 반지름·속도·허용오차·Keepout)
│   ├── teleop_8bitdo.yaml       패드 축/버튼 매핑 (D 모드 실측)
│   ├── web.yaml                 웹 서버 포트, Gemini 모델·음성
│   ├── carto.rviz               WSL RViz 설정 (지도·스캔·궤적·장소 마커)
│   ├── slam_toolbox.yaml, slam.rviz   (slam_toolbox 비교용)
│   └── cliff_config.yaml, ground_baseline.yaml   (낭떠러지 모듈 — 캡스톤과 무관)
├── rapa1/                     로봇(Pi)에서 도는 것
│   ├── nav.launch.py            ★ 평소 실행: localize + 패드 + 장소 + 위치기억 + 벽 + 웹
│   ├── localize.launch.py       bringup + map_server + AMCL + Nav2  (nav:=false 면 위치추정만)
│   ├── carto.launch.py          ★ 지도 만들 때: bringup + Cartographer
│   ├── place_manager.py         장소 저장/삭제/별칭, 마커, 패드 A 저장
│   ├── wall_manager.py          가상 벽 → Nav2 Keepout 마스크
│   ├── pose_keeper.py           마지막 AMCL 위치 저장 → 다음 실행 때 초기 위치 자동
│   ├── pad_gate.py              패드 명령이 Nav2 와 안 싸우게 거름
│   ├── slam.launch.py, scan_resample.py   (slam_toolbox 비교용)
│   └── cliff_guard_node.py, drive_monitor.py   (낭떠러지 모듈)
├── web/                       조종 웹 페이지
│   ├── server.py                rclpy 노드 + tornado HTTPS/WebSocket + 음성 세션
│   ├── gemini_live.py           Gemini Live WebSocket 클라이언트
│   ├── intent.py                장소명 해석·재검증, 정지어, 규칙 해석
│   └── static/index.html        페이지 (지도 캔버스, 버튼, 음성, 대화창) — 단일 파일
└── tools/
    ├── place.py                 장소 명령 CLI (Pi·WSL 어디서든)
    ├── goto.py                  장소로 주행 + 도착 오차 측정 (왕복 시험)
    └── wsl_rviz.sh              WSL 에서 RViz 띄우기
```

**저장소 밖 (기기마다 따로)**
```
~/maps/lab_v2.{yaml,pgm,pbstream}   지도 (지도는 공간마다 새로 만듦)
~/maps/places.yaml                  장소 (map 좌표)
~/maps/walls.yaml                   가상 벽
~/maps/last_pose.yaml               마지막 위치
~/.config/robot_web/gemini_api_key  API 키 (600)
~/.config/robot_web/{cert,key}.pem  자체 서명 HTTPS 인증서 (처음 실행 때 자동 생성)
```

---

## 6. 설치 — 로봇(Pi 5) 맨바닥부터

> 예상 시간 2~3시간 (빌드 30~60분 포함). **`sudo` 가 필요한 단계는 본인이 직접** 실행.

### 6-1. OS
- Raspberry Pi Imager → **Ubuntu Server 24.04 LTS (64-bit)** → SD 카드. Wi-Fi·사용자·SSH 미리 설정.
- 부팅 후:
```bash
sudo apt update && sudo apt install -y openssh-server net-tools git curl
hostname -I                       # IP 확인 → 공유기에서 고정 할당 권장
```

### 6-2. ⚠️ apt 소스에 `noble-updates` 확인 (안 하면 ROS 설치가 깨짐)
`held broken packages (libdbus-1-dev, libpcre2-dev, libselinux1-dev)` 가 나오면 이것 때문이다.
```bash
sudo cp /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list.d/ubuntu.sources.bak
sudo nano /etc/apt/sources.list.d/ubuntu.sources
#   첫 블록의  Suites: noble  →  Suites: noble noble-updates noble-backports
sudo apt update && sudo apt upgrade -y
```

### 6-3. ROS 2 Jazzy + 필요한 패키지 (root)
```bash
# 자동 업데이트·절전 끄기, 시리얼 권한
printf 'APT::Periodic::Update-Package-Lists "0";\nAPT::Periodic::Unattended-Upgrade "0";\n' | sudo tee /etc/apt/apt.conf.d/20auto-upgrades
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
sudo usermod -aG dialout,input $USER          # dialout=OpenCR/LiDAR, input=게임패드  → 재로그인 필요

# 로케일 + ROS 저장소
sudo apt install -y locales software-properties-common
sudo locale-gen en_US en_US.UTF-8 && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update

# ROS 기본 + TurtleBot3 의존 + 캡스톤(SLAM·Nav2·패드·웹)
sudo apt install -y \
  ros-jazzy-ros-base ros-dev-tools \
  ros-jazzy-dynamixel-sdk ros-jazzy-turtlebot3-msgs ros-jazzy-hls-lfcd-lds-driver \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
  ros-jazzy-tf2-ros ros-jazzy-tf-transformations libboost-system-dev libudev-dev \
  ros-jazzy-cartographer ros-jazzy-cartographer-ros \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server \
  ros-jazzy-joy ros-jazzy-teleop-twist-joy \
  ros-jazzy-rosbridge-suite \
  python3-tornado python3-numpy python3-pil python3-yaml \
  bluez openssl

# USB 권한 (OpenCR / LiDAR)
sudo tee /etc/udev/rules.d/99-turtlebot3-cdc.rules > /dev/null <<'EOF'
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ENV{ID_MM_DEVICE_IGNORE}="1", MODE:="0666"
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE:="0666"
ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo rosdep init || true
sudo systemctl enable --now bluetooth
```

### 6-4. TurtleBot3 워크스페이스 빌드 (사용자)
> ⚠️ ROBOTIS SBC 가이드는 `turtlebot3_cartographer`, `turtlebot3_navigation2` 를 **지우라고** 하지만, 캡스톤은 **지우면 안 된다** (Nav2 파라미터 원본이 거기 있음).
```bash
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/ld08_driver.git
cd ~/turtlebot3_ws && source /opt/ros/jazzy/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -y -r
colcon build --symlink-install --parallel-workers 1        # Pi 5 에서 약 1.5분~ (처음엔 더 걸림)
```

### 6-5. 환경변수 (`~/.bashrc` 끝에)
```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export ROS_DOMAIN_ID=40            # 팀원끼리 같은 공유기면 **서로 다른 번호** 쓸 것 (예: 41)
export TURTLEBOT3_MODEL=waffle_pi
export LDS_MODEL=LDS-02
```
`source ~/.bashrc` 후 **재로그인** (dialout·input 그룹 적용).

### 6-6. 프로젝트 받기
```bash
git clone https://github.com/jungju3756-source/tts_Self_driving_buger.git ~/tts_Self_driving_buger
mkdir -p ~/maps
```
> 내 로봇의 IP·OpenCR 시리얼이 다르면 확인:
> `ls /dev/serial/by-id/` → OpenCR 경로가 다르면 `rapa1/carto.launch.py`, `rapa1/localize.launch.py` 의 `OPENCR` 상수를 고친다.

### 6-7. 첫 동작 확인 (bringup)
```bash
ros2 launch turtlebot3_bringup robot.launch.py \
  usb_port:=/dev/serial/by-id/usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00
# 다른 터미널
ros2 topic hz /scan        # ≈ 10~11 Hz
ros2 topic hz /odom        # ≈ 20 Hz
ros2 topic echo --once /battery_state --field voltage     # 11~12.6 V
```
바퀴가 손으로 헛돌면: `ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}"`, 그래도 엔코더가 0 이면 **OpenCR 모터 전원 스위치/케이블** (12장).

### 6-8. 8BitDo Micro 게임패드 (블루투스)
1. 패드 뒷면 스위치 **D** (S=스위치, K=키보드 — K 면 키보드로 잡혀 ROS 가 못 읽음)
2. HOME 으로 켜고 **PAIR 1초** → LED 빠르게 깜빡 (PC 등 다른 기기와 연결돼 있으면 먼저 끊기)
3. Pi 에서 **한 bluetoothctl 세션 안에서** (단발 명령으로 하면 bond 가 저장 안 됨):
```bash
bluetoothctl
  agent NoInputNoOutput
  default-agent
  scan on            # "8BitDo Micro gamepad" 의 MAC 확인
  pair  <MAC>
  trust <MAC>        # 다음 부팅에 자동 연결
  connect <MAC>
  info  <MAC>        # Paired/Bonded/Trusted/Connected 모두 yes
  quit
cat /sys/class/input/event*/device/name   # "8BitDo Micro gamepad" (끝에 Keyboard 붙으면 K 모드)
```
4. 버튼 확인: `ros2 run joy joy_node` + `ros2 topic echo /joy` — D-pad 상하 `axes[7]`(위 +1), 좌우 `axes[6]`(**왼쪽 +1**), A=`buttons[0]`, L=6, R=7(터보).

### 6-9. 지도 만들기 (공간마다 한 번)
```bash
ros2 launch ~/tts_Self_driving_buger/rapa1/carto.launch.py
# 다른 터미널: 패드 조종 (nav.launch 를 안 띄운 상태라면)
ros2 run joy joy_node & ros2 run teleop_twist_joy teleop_node --ros-args --params-file ~/tts_Self_driving_buger/config/teleop_8bitdo.yaml
```
- **처음 5초 가만히** → 벽을 따라 **천천히** (회전은 짧게 톡톡, 터보 금지) → **출발점으로 돌아와** 루프 닫기
- WSL RViz 로 지도가 그려지는 걸 보면서 하면 좋다 (7장)
- 저장:
```bash
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$HOME/maps/lab_v2.pbstream', include_unfinished_submaps: true}"
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab_v2 --ros-args -p save_map_timeout:=15.0
```
- 다른 이름이면 실행 때 `map:=$HOME/maps/<이름>.yaml`

### 6-10. Gemini API 키
```bash
mkdir -p ~/.config/robot_web && chmod 700 ~/.config/robot_web
nano ~/.config/robot_web/gemini_api_key      # 키 한 줄 붙여 넣기
chmod 600 ~/.config/robot_web/gemini_api_key
```
(또는 웹 페이지 음성 칸 → Gemini Live → 키 입력)

### 6-11. 전체 실행
```bash
ros2 launch ~/tts_Self_driving_buger/rapa1/nav.launch.py
```
- 정상이면 로그에 `Managed nodes are active` **두 번**, `Opened joystick`, `pose_keeper ... AMCL 이 초기 위치를 받음`, `https://<IP>:8443`
- 첫 실행이면 `last_pose.yaml` 이 없어서 위치를 한 번 찍어야 함 (웹 🧭 또는 RViz 2D Pose Estimate)
- 웹 없이: `nav.launch.py web:=false`

---

## 7. 설치 — Windows PC (WSL RViz · 브라우저)

### 7-1. 브라우저 (필수)
- **Chrome** 으로 `https://<로봇IP>:8443` → "연결이 비공개로…" → **고급 → 계속** (자체 서명 인증서)
- 마이크 권한 허용. https 여야 브라우저가 마이크를 연다.
- 웹캠 마이크: 페이지 음성 칸의 마이크 목록에서 선택 (Gemini 모드). 브라우저 음성 모드는 Chrome 기본 마이크를 쓰므로 `chrome://settings/content/microphone` 에서 웹캠 마이크 선택.

### 7-2. WSL2 + RViz (선택, 디버깅·지도 작성 때 유용)
1. `C:\Users\<사용자>\.wslconfig`
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
2. **관리자 PowerShell**
   ```powershell
   Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
   Get-NetConnectionProfile                                   # Public 인 것 전부 ↓
   Set-NetConnectionProfile -InterfaceAlias "이더넷" -NetworkCategory Private
   Set-NetConnectionProfile -InterfaceAlias "Wi-Fi 3" -NetworkCategory Private   # 이름은 위 출력대로
   wsl --shutdown
   ```
3. WSL 안 (Ubuntu 24.04): ROS 2 Jazzy **desktop** 설치(공식 문서와 같음) 후
   ```bash
   sudo apt install -y ros-jazzy-turtlebot3-description
   echo 'export ROS_DOMAIN_ID=40' >> ~/.bashrc     # 로봇과 같은 번호
   scp jungju@192.168.0.57:~/tts_Self_driving_buger/tools/wsl_rviz.sh ~/ && bash ~/wsl_rviz.sh
   ```
   다음부터 `bash ~/wsl_rviz.sh` (ip 확인 → 설정·place.py 받기 → `ROS_STATIC_PEERS` 지정 → rviz2)
4. `ip -4 addr` 에 192.168.0.x 가 보여야 mirrored 가 된 것.

---

## 8. 사용법 (매일 하는 순서)

1. 로봇 전원 ON → Pi 부팅 → `ros2 launch ~/tts_Self_driving_buger/rapa1/nav.launch.py`
2. PC Chrome → `https://192.168.0.57:8443`
3. **위치 확인**: 지도에서 파란 로봇이 실제 위치와 맞는지. 다르면 🧭 **로봇 위치 지정** → 실제 위치 클릭 → 앞 방향으로 드래그
4. 이동: 오른쪽 **장소 버튼** 또는 지도 위 **장소 핀** 클릭
5. 정지: 빨간 **■ 정지** / **스페이스 키** / 음성 "멈춰"
6. 음성: 음성 칸 → **Gemini Live** 선택 → 마이크(웹캠) 선택 → 🎤 → "3번으로 가줘" · "지금 어디야" · "어디 갈 수 있어"
7. 끄기: 터미널 Ctrl+C (마지막 위치는 `last_pose.yaml` 에 저장돼 다음에 자동 복원)

### 웹 페이지 기능
| 영역 | 기능 |
|---|---|
| 지도 | 실시간 로봇 위치·방향, 계획 경로(초록 점선), 장소 핀, 벽(빨강), 확대/축소(휠·＋－), 드래그 이동 |
| 👆 보기·이동 | 장소 핀 클릭 = 그곳으로 이동 |
| ✏️ 벽 그리기 | 드래그로 선분 → 로봇이 넘지 않음 (빨간 띠 = 로봇 중심이 못 들어가는 영역, 양쪽 0.20 m) |
| 🧽 벽 지우기 | 벽 클릭 / "모두 지우기" |
| 📍 장소 추가 | 지도 클릭 → 이름 입력 |
| 🧭 로봇 위치 지정 | 클릭+드래그 (= RViz 2D Pose Estimate) |
| 로봇 상태 | 대기/이동 중/도착/실패, 남은 거리, 도착 시간·오차 |
| 음성 | 엔진 선택, 마이크 선택·🎚 테스트(레벨 막대), 🎤, 글로 입력 |
| 대화 | 내 말/로봇 말 말풍선 + 매 턴 **🎙 인식 · ⚙ 판단 · 🔊 응답 시간**, 응답 평균·최대·3초 이내 비율 |
| 상단 | 연결 상태 점, 배터리 % · V |

### 장소 저장 방법 4가지
| 방법 | 방법 | 이름 |
|---|---|---|
| 패드 | 패드로 몰고 가서 **A 1초** → 로봇 "삑" | 자동 `N번` (비어 있는 가장 작은 번호) |
| 웹 | 📍 장소 추가 → 클릭 → 이름 | 직접 |
| RViz | Publish Point(위치) 또는 2D Goal Pose(위치+방향, `/place_pose`) → `python3 place.py save 이름` | 직접 |
| CLI | `python3 tools/place.py here 이름` (로봇 현재 위치) | 직접 |

이름 바꾸기/별칭: `place.py rename 5번 정수기`, `place.py alias 정수기 물` (음성 인식 보정용)

### 시험 주행 (정량 평가용)
```bash
python3 ~/tts_Self_driving_buger/tools/goto.py 1번 2번 3번 4번          # 차례로, 끝에 성공률·오차 요약
python3 ~/tts_Self_driving_buger/tools/goto.py 1번 2번 1번 2번 1번 2번 1번 2번 1번 2번   # 왕복 10회
```
※ 오차는 **AMCL 추정 기준**. RFP 제출용은 바닥 테이프 기준점으로 **실측** 따로.

---

## 9. 노드 · 토픽 · 명령 인터페이스

### 9-1. 우리가 만든 노드
| 노드 | 파일 | 구독 | 발행/제공 |
|---|---|---|---|
| place_manager | rapa1/place_manager.py | `/places/cmd` String, `/clicked_point`, `/place_pose`, `/joy`, TF | `/places/list` (latched JSON), `/places/event`, `/places/markers`, `/sound` 호출 |
| wall_manager | rapa1/wall_manager.py | `/walls/cmd` String(JSON), `/map` | `/keepout_filter_mask`, `/costmap_filter_info` (latched), `/walls/list`, `/walls/event` |
| pose_keeper | rapa1/pose_keeper.py | `/amcl_pose` | `/initialpose` (시작 시, AMCL 응답 올 때까지 2초마다) |
| pad_gate | rapa1/pad_gate.py | `/cmd_vel_joy` | `/cmd_vel` (0 이 아닐 때만 + 놓을 때 0 세 번) |
| web_server | web/server.py | `/map`, `/places/*`, `/walls/*`, `/plan`, `/battery_state`, TF | `/places/cmd`, `/walls/cmd`, `/clicked_point`, `/initialpose`, `/cmd_vel`(정지), `navigate_to_pose` 액션 |

### 9-2. 장소 명령 `/places/cmd` (std_msgs/String, 공백 구분)
```
save <이름>          방금 찍은 점(/clicked_point 또는 /place_pose) 저장
here <이름>          로봇 현재 위치·방향 저장
delete <이름> | rename <이전> <새이름> | alias <이름> <별칭> | list
```
결과 `/places/event`: `{"ok":true,"cmd":"save","name":"정수기","place":{...}}` / `{"ok":false,"error":"..."}`
`~/maps/places.yaml`:
```yaml
frame: map
places:
  1번: {x: 0.582, y: -1.806, yaw: null, aliases: [], saved: '2026-10-02 15:04:36'}
```

### 9-3. 벽 명령 `/walls/cmd` (JSON)
```json
{"cmd":"add","x1":1.0,"y1":-1.0,"x2":2.0,"y2":-1.0}
{"cmd":"delete","id":3}
{"cmd":"clear"}
```

### 9-4. 웹소켓 `/ws` (브라우저 ↔ server.py)
| 방향 | 메시지 |
|---|---|
| 브라우저→서버 | `goto{name}`, `stop{why}`, `wall_add{x1,y1,x2,y2}`, `wall_delete{id}`, `wall_clear`, `place_add{x,y,name}`, `place_cmd{text}`, `initial_pose{x,y,yaw}`, `say{text}`, `voice_start`, `voice_stop`, `playback_idle`, **바이너리 = 마이크 PCM16 16 kHz** |
| 서버→브라우저 | `hello`, `map`(+`/map.png`), `places`, `walls`, `pose`(5 Hz), `plan`, `battery`, `nav{state,target,remaining,error_m,took}`, `say_reply`, `voice{event:ready/input_text/output_text/tool_call/turn_complete/interrupted/dispatch/error/closed}`, `log`, **바이너리 = Gemini 음성 PCM16 24 kHz** |

### 9-5. Gemini 도구 (함수 호출)
| 도구 | 인자 | 서버 동작 |
|---|---|---|
| `navigate_to` | destination | resolve_place 재검증 → 안내 멘트 재생 후 출발 |
| `stop_navigation` | — | 목표 취소 + `/cmd_vel` 0 |
| `list_places` | — | 장소 이름 목록 |
| `get_status` | — | "4번에서 0.4미터…", 이동 상태 |

---

## 10. 설정값과 바꾼 이유

### config/nav2_waffle_pi.yaml (ROBOTIS `waffle_pi.yaml` 기준)
| 항목 | 원본 → 변경 | 이유 |
|---|---|---|
| `robot_radius` (local/global) | 0.15 → **0.22** | Waffle Pi 281×306 mm 외접원. 0.15 면 벽에 긁힘 |
| `max_vel_x`, `max_speed_xy` | 0.3 → **0.26** | 실제 최고속도 |
| `goal_checker.xy_goal_tolerance` | 0.25 → **0.12** | RFP ±0.25 m 를 여유 있게 |
| `filters: [keepout_filter]` (양 costmap) | 없음 → 추가 | 웹에서 그린 가상 벽. `filter_info_topic: /costmap_filter_info` |

### config/cartographer_lds02.lua (ROBOTIS `turtlebot3_lds_2d.lua` 기준)
| 항목 | 원본 → 변경 | 이유 |
|---|---|---|
| `TRAJECTORY_BUILDER_2D.min_range` | 0.12 → **0.16** | LDS-02 사양 |
| `TRAJECTORY_BUILDER_2D.max_range` | 3.5 → **8.0** | 원본은 LDS-01 기준 |
| 스캔 토픽 | 원본 `/scan` | Cartographer 는 점 개수가 달라도 받음 (slam_toolbox 는 아님 → scan_resample) |

### config/teleop_8bitdo.yaml
`axis_linear.x: 7`, `axis_angular.yaw: 6`, `scale_linear 0.20 / 터보 0.26`, `scale_angular 1.0 / 터보 1.5` (**양수**, 음수로 하면 좌우 반대), `enable_turbo_button: 7`(R), `require_enable_button: false`, **`publish_stamped_twist: true`** (Jazzy turtlebot3_node 는 TwistStamped)

### config/web.yaml
```yaml
port: 8443
gemini: {model: gemini-3.8-live, voice: Kore, silence_ms: 600}
```
사용 가능한 Live 모델 확인 (키 필요):
```bash
curl -s -H "x-goog-api-key: $(grep -v '^#' ~/.config/robot_web/gemini_api_key | head -1)" \
  "https://generativelanguage.googleapis.com/v1beta/models?pageSize=200" | grep -B1 -A8 bidiGenerateContent | grep '"name"'
```
(2026-10-02 기준: gemini-3.8-live, gemini-3.8-live-extended-thinking, gemini-3.1-flash-live-preview, gemini-2.5-flash-native-audio-* …)

### place_manager / wall_manager 파라미터
| 파라미터 | 기본 | 설명 |
|---|---|---|
| `save_button` | -1 (nav.launch 는 **0**=A) | 패드 저장 버튼 |
| `hold_sec` | 1.0 | 누르고 있어야 하는 시간 |
| `half_width` (wall) | 0.20 | 벽 양쪽 금지 폭 (≈ 로봇 반지름. Keepout 은 inflation 후 적용돼 안 부풀려지므로) |

---

## 11. 검증 결과

| 항목 | 결과 | 조건 |
|---|---|---|
| 지도 | `lab_v2` 벽 직선·겹침 없음 | Cartographer, 3분·10.5 m 주행 |
| 주행 | **4/4 성공**, 오차 평균 **0.069 m**, 최대 0.112 m | 1→2→3→4, AMCL 기준, 18~61 초/구간 |
| 회전 응답 | 명령 0.6 rad/s → IMU 0.58 일정 | pad_gate 적용 후 (전엔 0.3~0.5 흔들림) |
| 가상 벽 | 벽 위 global costmap 0 → **100(치명)** → 지우면 0 | KeepoutFilter |
| Gemini Live | 세션 준비 0.5~0.9 s, 도구 호출 0.4~0.6 s, **첫 음성 0.8~1.2 s** | "1번으로 가줘", "어디 갈 수 있어", "정수기로 가줘"(미등록 → 거절, 이동 안 함) |
| 규칙 해석 | "일번 가"→1번, "삼번째로 데려다줘"→3번, "이번에 4번 가"→4번, "1번이랑 2번"→되묻기, "서울"→정지 아님 | intent.py |

---

## 12. 문제 해결 (실제로 겪은 것)

| 증상 | 원인 | 해결 |
|---|---|---|
| ROS 설치 `held broken packages` | apt 소스에 `noble-updates` 없음 | 6-2 |
| bringup `Failed to open the port(/dev/ttyACM0)` | OpenCR 전원 재투입 시 ttyACM0→1 로 번호 바뀜 | 항상 `/dev/serial/by-id/...` |
| LiDAR 스캔 없음, ld08_driver CPU 100% | LiDAR 도 ttyUSB0→1 로 바뀜 | 재시작 (by-id) |
| 토크 ON 인데 엔코더 0, 바퀴 헛돎 | 다이나믹셀 전원 미공급 | OpenCR 전원 스위치·케이블 |
| slam_toolbox 지도 거의 안 그려짐 | LDS-02 점 개수 205~209 가변 → 스캔 버림 | `scan_resample.py` (Cartographer 는 불필요) |
| 지도에 벽이 여러 각도로 겹침 | 빠른 회전 (1~1.5 rad/s) | 천천히, 루프 닫기 |
| SLAM 끝낸 뒤 RViz 클릭·`here` 안 됨 | finish_trajectory 후 Cartographer 가 map TF 중단 | map_server + AMCL (`localize.launch.py`) |
| Nav2 bringup 실패 `transform from base_link to map did not become available` | AMCL 초기 위치 없음 | `pose_keeper.py` 자동 / 웹 🧭 |
| **Nav2 주행이 덜컥거리고 제자리 회전** | teleop_twist_joy 가 대기 중에도 0 명령을 **~14 Hz** 로 `/cmd_vel` 에 보냄 | `pad_gate.py` 경유 (`/cmd_vel_joy` 리맵) |
| 패드로 눌러도 로봇 안 움직임 | Jazzy 는 `/cmd_vel` = TwistStamped | `publish_stamped_twist: true` |
| 패드 좌우 반대 | `scale_angular` 부호 | 양수 (joy 표준: 왼쪽 = +1). 누른 순서로 추정하지 말고 IMU 로 확인 |
| 패드가 `…Keyboard` 로 잡힘 | 8BitDo K 모드 | 스위치 D → 재페어링 (모드마다 MAC 다름) |
| `Rejected connection from !bonded device` | bluetoothctl 단발 명령 → bond 미저장 | 6-8 처럼 한 세션에서 agent 등록 후 pair |
| `/dev/input/event*` 못 읽음 | `input` 그룹 아님 | `usermod -aG input` → 재로그인 |
| WSL 에서 토픽 안 보임 | Hyper-V 방화벽, 네트워크 Public | 7-2 |
| `wsl_rviz.sh` 가 4단계에서 멈춤 | ros2 데몬이 파이프 물고 있음 | 토픽 확인 단계 제거 (현재 버전) |
| 브라우저 마이크 안 열림 | http 주소 | **https** 로 접속 |
| Gemini 키 "없음" | 키 형식 `AQ…` 를 서버가 거름 (수정됨) | 현재 버전은 형식 검사 안 함 |
| AMCL 이 RViz 위치 거부 `extrapolation into the future` | PC·Pi 시계 차 | 다시 찍기 / 웹 🧭 사용 (서버가 Pi 시계로 찍음) |
| `pkill -f`/`pgrep -f` 로 정리하다 자기 셸까지 죽음 | 패턴이 자기 명령줄에도 걸림 | `ps -eo pid,args` 로 PID 확인 후 `kill <PID>` |
| 웹 서버가 SIGTERM 에 안 꺼짐 | rclpy 스레드 | 현재 버전은 처리됨 |

---

## 13. 남은 일

- [ ] **정량 평가 (8단계)**: 평가용 발화 100개 이상(해석 성공률), 대화창의 응답 시간 통계, 바닥 테이프 기준점 실측 오차, 왕복 10회 중 9회 이상
- [ ] **호출어(wake word)** — 계획: 로컬 키워드 감지(예: Porcupine) 후 세션 열기. 지금은 🎤 버튼
- [ ] 장소 저장을 **음성으로** ("여기를 정수기로 저장해") — `save_place` 도구 추가 (place_manager `here` 연결만 하면 됨)
- [ ] 2번 구간 61초 (진행 없음 4회 + 회전 복구) — 벽 0.38 m 근처. 원인 확인
- [ ] Nav2 `use_composition:=True` 로 CPU 절감 (노드 10여 개가 각 8~17 %)
- [ ] (선택) 분리형(STT→LLM→TTS) 과 지연 비교 → 보고서
- [ ] SETUP.md 에 이 문서 내용 반영

---

## 14. 부록: 명령 모음

```bash
# ── 실행 ──
ros2 launch ~/tts_Self_driving_buger/rapa1/nav.launch.py                 # 평소
ros2 launch ~/tts_Self_driving_buger/rapa1/nav.launch.py web:=false
ros2 launch ~/tts_Self_driving_buger/rapa1/nav.launch.py map:=$HOME/maps/<이름>.yaml
ros2 launch ~/tts_Self_driving_buger/rapa1/carto.launch.py               # 지도 만들기

# ── 장소 ──
python3 ~/tts_Self_driving_buger/tools/place.py list
python3 ~/tts_Self_driving_buger/tools/place.py here 충전기
python3 ~/tts_Self_driving_buger/tools/place.py rename 5번 정수기
python3 ~/tts_Self_driving_buger/tools/place.py alias 정수기 물
python3 ~/tts_Self_driving_buger/tools/place.py delete 1번

# ── 주행 시험 ──
python3 ~/tts_Self_driving_buger/tools/goto.py 1번 2번 3번 4번

# ── 상태 확인 ──
ros2 node list
ros2 topic hz /scan ; ros2 topic hz /odom
ros2 topic echo --once /battery_state --field percentage
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic info -v /cmd_vel              # 발행자: collision_monitor, pad_gate, web_server … (teleop 직접 X)

# ── 정리 (PID 로!) ──
ps -eo pid,args | grep "[n]av.launch.py"     # 대괄호 트릭: grep 자기 자신 제외
kill -INT <PID>

# ── 로봇 끄기 ──
sudo shutdown -h now
```

> 이 문서와 코드가 다르면 **코드가 맞다**. 고친 사람이 이 문서도 같이 고쳐 주세요.
