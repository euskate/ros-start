# 로봇 융합 교육 ROS


- OS : Ubuntu 22.04
- ROS 2 Humble

```
├─shell : 환경 설치 셸 스크립트
└─src
    └─topic : topic 개념 이해 예제
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

## 환경 변수 설정:

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

``
