# 로봇 융합 교육 ROS

- OS : Ubuntu 22.04 (Windows VirtualBox를 이용)
- ROS 2 Humble
- 과제 노션 페이지
```
├─cosim : CoppeliaSim Pioneer3D (과제3)
├─shell : 환경 설치 셸 스크립트
└─src
    ├─topic : ROS topic 개념 이해 예제 (과제1)
    └─package_task : ROS package 실습 (과제2)
```

- [노션 페이지(서종원)](https://armaker.notion.site/Robot-f20fe1efd3cf43c0b603fa07151bf40d)

## 설치
- 환경 : Ubuntu 22.04 (가상 머신 등)
```
git clone https://github.com/euskate/ros-start.git
chmod u+x ./ros-start/shell/*.sh
./ros-start/shell/00_all.sh
```
- 전체 설치가 진행될 때까지 시간이 필요합니다.

## 환경 변수 설정

- ROS 2를 실행하기 위해 필요한 환경 변수를 설정합니다.

```bash
source /opt/ros/humble/setup.bash
```

- 자동화를 위해 `~/.bashrc`에 다음 줄을 추가:

```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

- 또는 alias 등록 후 `sb`, `humble` 실행

```
echo 'alias sb="source ~/.bashrc; echo \"bashrc is reloaded\""' >> ~/.bashrc
echo "alias humble=\"source /opt/ros/humble/setup.bash; echo \'ROS Humble activated\'\"" >> ~/.bashrc
```

## ROS2 토픽 실행 (과제1)
1. ROS2 환경 적용
    ```bash
    source /opt/ros/humble/setup.bash
    # 또는 alias
    humble
    ```

# 또는 .bashrc에 이전에 alias로 입력되어있는 humble 을 실행
2. 로봇 퍼블리셔 실행:
    
    ```bash
    cd ~/ros-start/topic
    python3 robot_state_publisher.py 1
    python3 robot_state_publisher.py 2
    ```
    
3. 중앙 모니터 실행:
    
    ```bash
    cd ~/ros-start/topic
    python3 robot_central_monitor.py
    ```

## ROS2 패키지 실습 (과제2)
1. 빌드
```
cd ~/ros-start
colcon build --symlink-install --packages-select package_task
```
2. 환경 변수 설정
```
source install/setup.bash
```
3. **turtlesim 실행**
    
    터미널 1:
    
    ```bash
    ros2 run turtlesim turtlesim_node
    
    ```
    
    → turtlesim 창이 나타나 거북이 시뮬레이터가 실행됩니다.
    
4. **거북이 생성 노드(turtle_spawn) 실행**
    
    터미널 2:
    
    ```bash
    ros2 run package_task turtle_spawn
    
    ```
    
    → `turtle2`, `turtle3`, `turtle4` 등이 지정된 위치에 생성됩니다.
    
5. **경주 노드(turtle_race) 실행**
    
    터미널 3:
    
    ```bash
    ros2 run package_task turtle_race
    
    ```
    
    → 거북이들이 무작위 이동 및 장애물 회피 동작(벽 감지 후 후진, 90도 회전, 전진)을 수행합니다.
    
6. **패턴 그리기 노드(turtle_patterns) 실행**
    
    터미널 4:
    
    ```bash
    ros2 run package_task turtle_patterns
    
    ```
    
    → 거북이들이 원, 사각형, 삼각형 등의 도형을 그리면서 이동합니다.
    
    ![2025-02-04 20 28 24.gif](./img/과제2_실행3.gif)

## 과제 3 : copelliasim 장애물 회피 알고리즘

```
# 디렉토리 진입
cd cosim
# 가상환경 생성
python -m venv cosim
# 가상환경 진입
.\cosim\Scripts\activate
# 라이브러리 설치
pip install pyzmq cbor2 coppeliasim-zmqremoteapi-client pynput numpy
```

실행
```
python Pioneer_03_task.py
```

- **시작 및 초기화**
    - 스크립트를 실행하면 `PioneerP3DXController` 클래스가 생성되어 CoppeliaSim과 연결됩니다.
    - 로봇의 좌측 모터, 우측 모터, 그리고 16개의 초음파 센서 객체를 초기화하여 사용 준비를 완료합니다.
- **모드 전환**
    - **기본 모드**: 프로그램은 기본적으로 수동 모드(`manual`)로 시작합니다.
    - **모드 전환**:
        - **`m` 키**: 수동 모드와 자율 모드를 전환합니다.
        - 모드 전환 시, 현재 모드에 따라 로봇의 제어 방식이 달라집니다.
- **수동 모드 제어**
    - **화살표 키**를 사용하여 로봇을 직접 조종합니다.
        - **Up/Down 화살표**: 전진 및 후진 속도를 조절합니다.
        - **Left/Right 화살표**: 좌회전 및 우회전 동작을 실행합니다.
    - 키를 뗄 경우, 해당 방향의 속도가 0으로 초기화되어 로봇이 정지합니다.