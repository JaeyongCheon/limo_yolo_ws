# LIMO YOLO Workspace

ROS 2 기반의 Behavior Tree와 YOLO 객체 인식을 통합한 로봇 자율 주행 시스템입니다.

## 📋 목차

1. [개요](#개요)
2. [프로젝트 구조](#프로젝트-구조)
3. [주요 기능](#주요-기능)
4. [설치 방법](#설치-방법)
5. [사용 방법](#사용-방법)
6. [설정 파일](#설정-파일)
7. [모듈 설명](#모듈-설명)
8. [시나리오](#시나리오)
9. [YOLO 통합](#yolo-통합)
10. [트러블슈팅](#트러블슈팅)

---

## 🎯 개요

이 프로젝트는 다음 기술들을 통합하여 로봇의 자율 주행 및 객체 인식 기능을 제공합니다:

- **py_trees**: Python 기반 Behavior Tree 라이브러리
- **ROS 2**: 로봇 운영 체제
- **YOLO**: 실시간 객체 인식 (Ultralytics)
- **Nav2**: ROS 2 네비게이션 스택
- **Pygame**: Behavior Tree 시각화

### 주요 특징

- ✅ Behavior Tree 기반 로봇 제어
- ✅ YOLO 객체 인식 통합
- ✅ 실시간 BT 시각화
- ✅ ROS 2 네비게이션 지원
- ✅ 모듈화된 시나리오 시스템
- ✅ TurtleBot3 시뮬레이션 지원

---

## 📁 프로젝트 구조

```
limo_yolo_ws/
├── modules/                    # 핵심 모듈
│   ├── agent.py               # 에이전트 관리
│   ├── base_bt_nodes.py       # 기본 BT 노드
│   ├── base_bt_nodes_ros.py   # ROS BT 노드
│   ├── bt_constructor.py      # BT 구성
│   ├── bt_runner.py           # BT 실행 엔진
│   ├── bt_visualiser.py       # BT 시각화
│   ├── ros_bridge.py          # ROS 브리지
│   └── utils.py               # 유틸리티 함수
│
├── scenarios/                  # 시나리오 폴더
│   ├── example_turtlesim/     # TurtleSim 예제
│   ├── nav2_turtlebot3/       # Nav2 + TurtleBot3
│   └── test/                  # 테스트 시나리오
│
├── src/yolo_ros/              # YOLO ROS 패키지
│   ├── yolo_bringup/          # YOLO 런치 파일
│   ├── yolo_msgs/             # YOLO 메시지 정의
│   └── yolo_ros/              # YOLO ROS 노드
│
├── build/                      # 빌드 출력
├── install/                    # 설치된 패키지
├── log/                        # 로그 파일
├── runs/                       # YOLO 실행 결과
│
├── main.py                     # 메인 실행 파일
├── config.yaml                 # 설정 파일
├── yolo_node.py               # YOLO 노드 구현
├── video_publisher.py         # 비디오 퍼블리셔
├── requirements.txt           # Python 의존성
├── run.sh                     # 실행 스크립트
│
├── best.pt                    # 커스텀 YOLO 모델
├── yolo*.pt                   # 다양한 YOLO 모델 파일
│
├── docs/                      # 문서
└── README.md                  # 이 파일
```

---

## ⚙️ 주요 기능

### 1. Behavior Tree 시스템

- **실시간 BT 실행**: 30Hz로 동작하는 Behavior Tree 엔진
- **시각화**: Pygame 기반 실시간 BT 상태 시각화
- **모듈화**: 재사용 가능한 BT 노드 라이브러리
- **XML 기반 구성**: XML 파일로 BT 구조 정의

### 2. ROS 2 통합

- **ROS 브리지**: ROS 2 토픽, 서비스, 액션 지원
- **Nav2 통합**: 네비게이션 목표 설정 및 추적
- **이미지 처리**: 카메라 이미지 수신 및 처리
- **비동기 통신**: asyncio 기반 비동기 ROS 통신

### 3. YOLO 객체 인식

- **다중 모델 지원**: YOLOv5 ~ YOLOv12, YOLO-World, YOLOE
- **실시간 감지**: 비디오 스트림에서 실시간 객체 인식
- **ROS 통합**: 감지 결과를 ROS 메시지로 퍼블리시
- **세그멘테이션**: 인스턴스 세그멘테이션 지원

---

## 🚀 설치 방법

### 1. 사전 요구사항

```bash
# Ubuntu 22.04 LTS
# ROS 2 Humble

# ROS 2 설치 확인
ros2 --version
```

### 2. 워크스페이스 클론

```bash
cd ~
git clone https://github.com/JaeyongCheon/limo_yolo_ws.git
cd limo_yolo_ws
```

### 3. Python 의존성 설치

```bash
pip3 install -r requirements.txt
pip3 install -r src/yolo_ros/requirements.txt
```

**requirements.txt 내용:**
- `pygame` - BT 시각화
- `py-trees` - Behavior Tree 라이브러리
- `pyyaml` - YAML 설정 파일
- `imageio` - 이미지 처리
- `pandas`, `matplotlib`, `seaborn` - 데이터 분석
- `opencv-python` - 컴퓨터 비전
- `ultralytics` - YOLO 모델

### 4. ROS 패키지 빌드

```bash
# 워크스페이스 루트에서
colcon build --symlink-install

# 환경 소싱
source install/local_setup.bash
```

---

## 📖 사용 방법

### 기본 실행

#### 1. Behavior Tree 실행

```bash
cd ~/limo_yolo_ws
python3 main.py --config=config.yaml
```

#### 2. YOLO 노드 실행

**방법 1: ROS 2 런치 파일**
```bash
source install/local_setup.bash
ros2 launch yolo_bringup yolov8.launch.py
```

**방법 2: 단독 실행**
```bash
python3 yolo_node.py
```

#### 3. 비디오 퍼블리셔 (테스트용)

```bash
python3 video_publisher.py
```

### TurtleBot3 시나리오

#### 1단계: 시뮬레이션 실행

시뮬레이션 환경(Gazebo 또는 기타)과 Nav2를 실행합니다.

#### 2단계: py_bt_ros 실행

```bash
cd ~/limo_yolo_ws
python3 main.py
```

#### 3단계: Rviz2에서 목표 설정

1. Rviz2 상단의 `2D Nav Goal` 버튼 클릭
2. 맵에서 목표 위치 클릭 및 드래그로 방향 설정
3. 로봇이 자동으로 이동 시작

---

## ⚙️ 설정 파일

### config.yaml

```yaml
scenario: scenarios.test          # 실행할 시나리오 모듈

agent:
  namespaces: ""                  # ROS 네임스페이스
  behavior_tree_xml: "new_test.xml"  # BT XML 파일 경로

bt_runner:
  bt_tick_rate: 30.0              # BT 틱 레이트 (Hz)
  bt_visualiser:
    enabled: True                 # BT 시각화 활성화
    screen_width: 600            # 화면 너비
    screen_height: 600           # 화면 높이
  profiling_mode: False           # 프로파일링 모드
```

### YOLO 노드 파라미터

```python
# yolo_node.py 파라미터
model_path: '/path/to/best.pt'   # YOLO 모델 경로
input_topic: '/image_raw'         # 입력 이미지 토픽
detection_topic: '/detections'    # 감지 결과 토픽
publish_image: True               # 결과 이미지 퍼블리시 여부
conf_threshold: 0.25              # 신뢰도 임계값
```

---

## 🧩 모듈 설명

### modules/agent.py
- 에이전트 생성 및 관리
- ROS 네임스페이스 처리
- 다중 에이전트 지원

### modules/base_bt_nodes.py
- 기본 BT 노드 클래스 정의
- Action, Condition, Decorator 노드
- 재사용 가능한 노드 라이브러리

### modules/base_bt_nodes_ros.py
- ROS 통합 BT 노드
- Nav2GoToGoal: 목표 지점 이동
- GetImage: 이미지 캡처
- 토픽/서비스/액션 통신 노드

### modules/bt_constructor.py
- XML에서 BT 구조 파싱
- BT 트리 구성 및 검증
- 노드 인스턴스 생성

### modules/bt_runner.py
- BT 실행 엔진
- 틱 주기 관리
- 키보드 입력 처리 (일시정지, 재시작 등)
- 시각화 연동

### modules/bt_visualiser.py
- Pygame 기반 BT 시각화
- 노드 상태 표시 (SUCCESS, FAILURE, RUNNING)
- 트리 구조 렌더링
- 실시간 업데이트

### modules/ros_bridge.py
- ROS 2 클라이언트 초기화
- 토픽 서브스크라이버/퍼블리셔
- 서비스 클라이언트
- 액션 클라이언트

### modules/utils.py
- 설정 파일 로딩
- 유틸리티 함수
- 로깅 설정

---

## 🎬 시나리오

### scenarios/example_turtlesim/
TurtleSim을 이용한 기본 예제
- 원 그리기
- 사각형 이동
- 랜덤 이동

### scenarios/nav2_turtlebot3/
Nav2를 이용한 TurtleBot3 시나리오
- **MoveToGoal**: 목표 지점으로 이동
- **CaptureImage**: 목표 지점에서 이미지 캡처
- **Return**: 초기 위치로 복귀

### scenarios/test/
개발 및 테스트용 시나리오
- 커스텀 BT 노드 테스트
- 새로운 기능 검증

### 시나리오 구조

각 시나리오는 다음 구조를 가집니다:

```python
scenarios/
├── __init__.py
└── your_scenario/
    ├── __init__.py
    ├── scenario.py           # 시나리오 정의
    ├── custom_nodes.py       # 커스텀 BT 노드
    └── behavior_tree.xml     # BT 구조
```

### 새 시나리오 만들기

1. `scenarios/` 폴더에 새 폴더 생성
2. `scenario.py` 작성:

```python
from modules.agent import Agent
from modules.bt_constructor import BTConstructor

def create_scenario(config):
    agent = Agent(config['agent']['namespaces'])
    bt = BTConstructor.from_xml(
        config['agent']['behavior_tree_xml'],
        agent
    )
    return agent, bt
```

3. `config.yaml`에서 시나리오 지정:
```yaml
scenario: scenarios.your_scenario
```

---

## 🤖 YOLO 통합

### 지원 모델

프로젝트에 포함된 YOLO 모델들:

| 모델 파일 | 설명 | 크기 |
|----------|------|------|
| `best.pt` | 커스텀 학습 모델 | - |
| `yolov8n.pt` | YOLOv8 Nano | 경량 |
| `yolov8m.pt` | YOLOv8 Medium | 중간 |
| `yolov8m-seg.pt` | YOLOv8 Segmentation | 중간 |
| `yolov8s-world.pt` | YOLO-World Small | 오픈 어휘 |
| `yolov9s.pt` | YOLOv9 Small | 최신 |
| `yolov9c-seg.pt` | YOLOv9 Segmentation | 53.85 MB |
| `yolov10n.pt` | YOLOv10 Nano | 초경량 |
| `yolo12n.pt` | YOLOv12 Nano | 최신 |
| `yoloe-11s-seg-pf.pt` | YOLOE Segmentation | 특화 |

### YOLO 노드 사용법

#### 1. 기본 실행

```bash
python3 yolo_node.py
```

#### 2. 파라미터 변경

```bash
ros2 run yolo_ros yolo_node \
  --ros-args \
  -p model_path:=/path/to/model.pt \
  -p input_topic:=/camera/image_raw \
  -p conf_threshold:=0.5
```

#### 3. 감지 결과 구독

```python
import json
import rclpy
from std_msgs.msg import String

def detection_callback(msg):
    detections = json.loads(msg.data)
    for det in detections:
        print(f"Class: {det['class']}, Conf: {det['confidence']}")

# ... ROS 노드 설정
```

### YOLO ROS 패키지

#### yolo_bringup
런치 파일 모음:
- `yolov5.launch.py`
- `yolov8.launch.py`
- `yolov9.launch.py`
- `yolov10.launch.py`
- `yolov11.launch.py`

#### yolo_msgs
커스텀 메시지 정의:
- `Detection.msg` - 단일 감지 결과
- `DetectionArray.msg` - 감지 결과 배열
- `Point2D.msg` - 2D 좌표
- `BoundingBox.msg` - 바운딩 박스

#### yolo_ros
핵심 ROS 노드:
- 객체 감지
- 인스턴스 세그멘테이션
- 포즈 추정
- 3D 감지 (depth 이미지 사용)

---

## 🔧 트러블슈팅

### 1. 모듈을 찾을 수 없음

```bash
# PYTHONPATH 확인
export PYTHONPATH=$PYTHONPATH:~/limo_yolo_ws

# 또는 main.py에서 실행
cd ~/limo_yolo_ws
python3 main.py
```

### 2. ROS 2 토픽이 보이지 않음

```bash
# ROS 환경 소싱 확인
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# 토픽 확인
ros2 topic list
```

### 3. YOLO 모델 로딩 실패

```bash
# ultralytics 재설치
pip3 uninstall ultralytics
pip3 install ultralytics

# 모델 경로 확인
ls -lh *.pt
```

### 4. Pygame 창이 열리지 않음

```bash
# Pygame 재설치
pip3 install --upgrade pygame

# config.yaml에서 시각화 비활성화
bt_visualiser:
  enabled: False
```

### 5. colcon build 실패

```bash
# 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 클린 빌드
rm -rf build install log
colcon build --symlink-install
```

### 6. 대용량 파일 경고 (GitHub)

일부 YOLO 모델 파일이 50MB를 초과합니다:
- `yolov8m-seg.pt` (52.38 MB)
- `yolov9c-seg.pt` (53.85 MB)

**해결 방법:**
```bash
# Git LFS 사용
git lfs install
git lfs track "*.pt"
git add .gitattributes
```

### 7. Navigation 실패

```bash
# Nav2 상태 확인
ros2 topic echo /diagnostics

# 목표 지점이 맵 안에 있는지 확인
# 더 가까운 목표로 변경
```

### 8. 이미지 토픽 수신 안 됨

```bash
# 카메라 토픽 확인
ros2 topic list | grep camera
ros2 topic echo /camera/image_raw --once

# 토픽 이름 확인 후 config에서 수정
```

---

## 📝 키보드 단축키

BT 실행 중 사용 가능한 단축키:

- `SPACE` - 일시정지/재개
- `R` - BT 재시작
- `Q` / `ESC` - 종료
- `Ctrl+C` - 강제 종료

---

## 📚 참고 문서

### 공식 문서
- [py_trees 문서](https://py-trees.readthedocs.io/)
- [ROS 2 Humble 문서](https://docs.ros.org/en/humble/)
- [Ultralytics YOLO 문서](https://docs.ultralytics.com/)
- [Nav2 문서](https://navigation.ros.org/)

### 관련 저장소
- [yolo_ros 원본](https://github.com/mgonzs13/yolo_ros)

---

## 📄 라이선스

이 프로젝트는 다음 라이선스를 따릅니다:
- py_bt_ros: MIT License
- yolo_ros: GPL-3.0 License

---

## 🐛 버그 리포트 & 기능 제안

이슈는 GitHub Issues에 등록해주세요:
https://github.com/JaeyongCheon/limo_yolo_ws/issues

---

## 📧 연락처

문의사항이 있으시면 GitHub Issues를 이용해주세요.

---

---

패키지 사용 및 적용 시 도움을 받았습니다.

---

**마지막 업데이트:** 2025-12-17
