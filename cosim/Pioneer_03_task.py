from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from pynput import keyboard
from pynput.keyboard import Key, Listener
import time
import math

class PioneerP3DXController:
    def __init__(self):
        # CoppeliaSim 연결 초기화
        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")
        self.run_flag = True

        try:
            # 로봇 객체 초기화: 모터와 16개의 초음파 센서를 가져옵니다.
            self.left_motor = self.sim.getObject("/PioneerP3DX/leftMotor")
            self.right_motor = self.sim.getObject("/PioneerP3DX/rightMotor")
            self.sensors = [
                self.sim.getObject(f"/PioneerP3DX/ultrasonicSensor[{i}]")
                for i in range(16)
            ]
        except Exception as e:
            print("객체 초기화 오류:", e)
            self.run_flag = False
            return

        # 제어 변수 초기화
        self.vel_X = 0.0           # 전진/후진 속도
        self.vel_Y = 0.0           # 회전 속도
        self.delta_velocity = 1.0  # 전진/후진 시 속도 값
        self.turn_velocity = 0.5   # 회전 시 속도 값

        # 모드 설정: "manual" (수동) 또는 "autonomous" (자율)
        self.mode = "manual"
        self.listener = None

    def set_velocity(self, vel_x, vel_y):
        """
        좌우 모터에 속도를 전달합니다.
        회전을 위해 왼쪽과 오른쪽 모터에 속도 차이를 적용합니다.
        """
        try:
            self.sim.setJointTargetVelocity(self.left_motor, vel_x - vel_y)
            self.sim.setJointTargetVelocity(self.right_motor, vel_x + vel_y)
        except Exception as e:
            print("제어 오류:", e)
            self.stop()

    def control_manual(self):
        """
        수동 모드에서는 키보드로 입력된 속도를 그대로 적용합니다.
        """
        self.set_velocity(self.vel_X, self.vel_Y)

    def autonomous_control(self):
        """
        자율 모드에서는 센서 데이터를 읽어 장애물을 회피하는 동작을 수행합니다.
        - 임계치(threshold) 이하의 거리를 감지하면 해당 방향을 피하도록 합니다.
        - 왼쪽(인덱스 0~3)과 오른쪽(인덱스 4~7) 전방 센서를 기준으로 판단합니다.
        """
        sensor_distances = []
        threshold = 0.5  # 임계치 (미터)
        try:
            # 모든 센서로부터 거리 값을 읽어옵니다.
            for sensor in self.sensors:
                ret, detectionState, detectedPoint, _, _ = self.sim.readProximitySensor(sensor)
                if ret and detectionState and detectedPoint:
                    # 3D 좌표를 이용해 유클리드 거리를 계산합니다.
                    distance = math.sqrt(
                        detectedPoint[0]**2 + detectedPoint[1]**2 + detectedPoint[2]**2
                    )
                else:
                    distance = float('inf')
                sensor_distances.append(distance)

            # 전방 센서(인덱스 0~7)를 기준으로 장애물 유무 판단
            left_obstacle = any(d < threshold for d in sensor_distances[0:4])
            right_obstacle = any(d < threshold for d in sensor_distances[4:8])
            
            if left_obstacle and not right_obstacle:
                # 왼쪽에 장애물이 감지되면 오른쪽으로 회전
                vel_x = 0.0
                vel_y = -self.turn_velocity
                print("왼쪽 장애물 감지, 오른쪽 회전")
            elif right_obstacle and not left_obstacle:
                # 오른쪽에 장애물이 감지되면 왼쪽으로 회전
                vel_x = 0.0
                vel_y = self.turn_velocity
                print("오른쪽 장애물 감지, 왼쪽 회전")
            elif left_obstacle and right_obstacle:
                # 양쪽 모두 장애물이 있으면 후진
                vel_x = -self.delta_velocity
                vel_y = 0.0
                print("양쪽 장애물 감지, 후진")
            else:
                # 장애물이 없으면 전진
                vel_x = self.delta_velocity
                vel_y = 0.0
                print("장애물 없음, 전진")
            
            # 선택적으로 모든 센서 값을 출력 (탐지된 값만)
            sensor_info = " | ".join(
                [f"센서 {i}: {d:.2f}m" for i, d in enumerate(sensor_distances) if d != float('inf')]
            )
            if sensor_info:
                print(sensor_info)
            return vel_x, vel_y
        except Exception as e:
            print("자율 제어 오류:", e)
            self.stop()
            return 0.0, 0.0

    def stop(self):
        """
        시뮬레이션 종료 및 모든 리소스를 정리합니다.
        """
        self.run_flag = False
        if self.listener:
            self.listener.stop()
        try:
            self.sim.stopSimulation()
        except Exception:
            pass
        print("시뮬레이션 종료 👋")

    def on_press(self, key):
        """
        키보드 입력 처리:
        - 화살표 키: 수동 모드일 때 로봇의 이동을 제어합니다.
        - 'm' 키: 수동 모드와 자율 모드 간 전환.
        - ESC: 시뮬레이션 종료.
        """
        if not self.run_flag:
            return False
        try:
            # 모드 전환: 알파벳 'm'을 눌렀을 때
            if hasattr(key, 'char') and key.char == 'm':
                self.mode = "autonomous" if self.mode == "manual" else "manual"
                print(f"모드 전환: {self.mode}")
            if self.mode == "manual":
                if key == Key.up:
                    self.vel_X = self.delta_velocity  # 전진
                elif key == Key.down:
                    self.vel_X = -self.delta_velocity  # 후진
                elif key == Key.left:
                    self.vel_Y = self.turn_velocity  # 좌회전
                elif key == Key.right:
                    self.vel_Y = -self.turn_velocity  # 우회전
            if key == Key.esc:
                self.stop()
                return False
        except Exception as e:
            print("키 입력 오류:", e)
            self.stop()
            return False

    def on_release(self, key):
        """
        키를 뗐을 때 수동 모드의 속도를 초기화합니다.
        자율 모드에서는 센서 데이터 기반 제어가 이루어집니다.
        """
        if not self.run_flag:
            return False
        try:
            if self.mode == "manual":
                if key in [Key.up, Key.down]:
                    self.vel_X = 0.0
                elif key in [Key.left, Key.right]:
                    self.vel_Y = 0.0
        except Exception as e:
            print("키 해제 오류:", e)
            self.stop()
            return False

    def run_simulation(self):
        """
        시뮬레이션을 시작하고, 키보드 이벤트와 로봇 제어 루프를 실행합니다.
        매 스텝마다 현재 모드에 따라 수동 제어나 자율 제어를 수행합니다.
        """
        try:
            self.sim.startSimulation()
            print("시뮬레이션 시작 🚀")
            with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
                self.listener = listener
                while self.run_flag:
                    if self.mode == "manual":
                        self.control_manual()
                    else:
                        auto_vel_x, auto_vel_y = self.autonomous_control()
                        self.set_velocity(auto_vel_x, auto_vel_y)
                    self.sim.step()  # 시뮬레이션 한 스텝 진행
                    time.sleep(0.1)
        except Exception as e:
            print("시뮬레이션 실행 오류:", e)
        finally:
            self.stop()

if __name__ == "__main__":
    controller = PioneerP3DXController()
    controller.run_simulation()
