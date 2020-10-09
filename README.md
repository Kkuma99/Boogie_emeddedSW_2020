# 📦Logi Boogie - 스마트 물류 시스템

# ➕ 참가정보
 - 2020년 임베디드 소프트웨어 경진대회 (smart things 부문 - 5024)
 - 팀명: Boogie
 - 작품명: Logi Boogie - 스마트 물류 시스템
 - 팀원: 강민지, 권미경
 <br>
 
 - 주제: Turtlebot3 Waffle Pi와 Open Manipulator를 이용한 스마트 물류 시스템 제어 로봇 제작
 - 개요: 물류센터에서 차량에 물건 적재 시 로봇에 탑재된 프로그램을 통해 크기와 배송지에 따라 최적의 적재 순서를 빠르게 계산하는 스마트 물류 시스템을 고안하였다. 이 시스템을 통해 물류 업무의 노동/시간적 효율을 높일 수 있으며, 차량의 빈 공간을 최소화하는 방향으로 계산하기 때문에 한 번에 더 많은 물류를 배송할 수 있다.
 - 개발 목표:
      - 영상처리를 이용한 바코드 스캔
      - 영상처리를 이용한 상자 크기 측정
      - ROS 통신을 이용한 두 PC 간의 연속적인 데이터 송수신
      - 로봇팔 제어를 통한 상자 분류
      - 저장된 상자 정보로 적재 순서를 계산하는 알고리즘 개발
      - 계산된 순서대로 상자의 정보를 표시하고 3D 시각화하는 알고리즘 개발

 - 진행 사항 참고: https://kkuma99.github.io/Boogie/
 
 ---
 # ➕ Needed (architecture)
 
 | `Development Environment` | `OS` | `ROS version` | `else` |
 | --- | --- | --- | --- |
 | Jetson TX2 | Jetpack 3.3.1 - Ubuntu 16.04 | ROS Kinetic | OpenCV 3.4.6, pyzbar, matplotlib, SSH |
 | Raspberry Pi 3 | Raspbian | ROS Kinetic | SSH |
 | HOST PC | Ubuntu 16.04 | ROS Kinetic | SSH, OpenCV 3.4.6, GCC compiler |
 
 다음과 같은 개발환경을 통하여 현재 프로젝트를 진행하였다.
 
 ---
 # ➕ System Process
 
 다양한 개발환경 및 보드를 같이 사용하기 때문에 그에 따라 코드를 한번에 실행하지 않고 여러 개의 통합 소스 파일들을 실행하게 된다.
 그에 따라 소스코드 실행 방법을 정리해두었다.
 
 <details>
<summary><span style="color:skyblue">✏️실행 명령어 (요약)</span></summary>

```
1. roscore : host pc에서 192.168.0.18로 실행

2. python3 smart_logi_system_jetson.py : Jetson TX2에서 실행

3. roslaunch turtlebot3_bringup turtlebot3_robot.launch: RPi에서 실행 ( bash에 마스터 선언 확인)

4. roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch : host pc에서 실행

5. roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch : host pc에서 실행

```
</details>
 
 ---
  # ➕ Source code
  
  ## 📝Loading Algorithm
  적재 알고리즘: [smart_logi_system_jetson.py](https://google.com, "google link")
  
  
  
  ## 📝Robot Arm Algorithm
  ### for raspberry pi
  [RPi launch](https://github.com/Kkuma99/Boogie_emeddedSW_2020/tree/master/Robot/SBC/turtlebot3_manipulation/turtlebot3_manipulation_bringup)
  
  ### main (host)
  [Host PC launch](https://github.com/Kkuma99/Boogie_emeddedSW_2020/tree/master/Robot/SBC/turtlebot3_manipulation/turtlebot3_manipulation_bringup)
 
---
 # ➕ 추후 활용방안
 
 - 택배 회사의 물류 상하차에 활용
 ```    
 이 시스템의 주 목적은 물류센터의 업무 효율을 높이는 것이다. 빠른 적재 순서 계산을 통하여 여러 번 상하차 진행을 할 필요없이 박스를 전달받은 대로 진행을 하면 되기 때문에 물품을 전달하는 과정에서도 찾는데 시간 절약이 가능하다.
 ```
 
 - 스마트 팩토리의 물건 분류와 재고 관리에 활용 가능
 ```
 스마트 팩토리에서 물건을 분류하고 출고 직전에 물품을 관리하는 데에 활용할 수 있다.
 ```
 
 - 대형 창고 및 물품 보관 서비스 업체에서 활용 가능
 ```
 많은 물품을 보관해야 하는 대형 창고나 물품 보관 서비스 업체에서는 제한되어있는 규모의 공간에 최대한 많은 물품을 보관하는 것이 가장 이득이다. 크기에 따라 배치를 결정해주는 시스템을 여기에 적용할 수 있다.
 ```

---
 이 후에도 다양한 개발 아이디어를 적용하여 작품을 발전시킬 계획입니다. 감사합니다.
