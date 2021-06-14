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
 - 개발영상:


<iframe width="640" height="360" src="https://youtu.be/anbjpytdbVc" frameborder="0" gesture="media" allowfullscreen=""></iframe>
 
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
  적재 알고리즘: [smart_logi_system_jetson.py](https://github.com/Kkuma99/Boogie_emeddedSW_2020/blob/master/Load_JetsonTX2/smart_logi_system_jetson.py)
  <br>
  
  `상자의 정보를 수집하고 적재 순서를 계산`
  
   <details>
<summary><span style="color:blue"> 알고리즘 설명 (요약)</span></summary>

```
① 먼저, 배송해야 하는 배송지의 개수와 트럭의 크기를 상수로 지정하고, 바코드 데이터를 담을 변수를 초기화한다.
② VideoCapture()를 통해 웹카메라의 화면을 받아온 뒤, 프레임의 너비와 높이, 프레임 속도를 조절한다.
③ set_window() 함수를 호출하여 카메라 화면을 display할 준비를 한다. window의 이름을 지정하고, 뒤에서 검출할 컨투어의 threshold를 쉽게 변경할 수 있도록 트랙바를 추가한다. 이 threshold는 작업을 수행하는 환경의 조도 상황에 따라 조정하여 사용한다.
④  while문에 진입하여 지속적인 영상처리를 시작한다. while문 내부에서는 상자 인식, 상자 정보 수집, 바코드 데이터 전송 등의 작업을 수행한다. 우선 카메라의 프레임을 읽어와 img_color 객체에 저장한다. box_detection() 함수를 호출하여 현재 프레임에서 상자를 인식한다. 
⑤ 다음으로 get_box_info() 함수를 호출하여 상자의 정보를 수집한다. 
⑥ send_data_to_host() 함수를 호출하여 바코드 데이터를 Host PC에 전송한다. 이 함수는 ROS 통신 중 단방향 통신인 메시지 통신을 수행한다. 데이터를 전송하는 publisher 역할이며, 문자열 데이터를 전송한다.
⑦ imshow()를 통해 컨투어, 바코드 등의 정보가 삽입된 이미지를 화면에 띄운다. waitKey()를 사용하여 키보드로 ESC 키가 입력되면 while문에서 탈출한다. ESC 키는 모든 상자의 입력이 끝났을 때 사용한다.
⑧ while문에서 탈출한 뒤에는 OpenCV, 즉 영상처리에 사용된 모든 메모리를 해제한다.
⑨ matplotlib를 이용하여 트럭의 내부 모습을 시각화하기 위해 plot의 형태를 3D로, 양상을 auto로 설정한다. colors는 각 배송지 별 상자의 색이다.
⑩ draw_truck() 함수를 호출하여 트럭의 전체 프레임을 생성한다. numpy의 meshgrid() 함수를 사용하여 격자를 생성하고, 검정색 선으로 표현한다.
⑪ calculate_loading_order() 함수를 호출하여 입력된 모든 상자들에 대해 최적의 적재 순서를 계산한다.
```
</details>
  
  ## 📝Robot Arm Algorithm
  ### for raspberry pi
  roslaunch turtlebot3_bringup turtlebot3_robot.launch: [RPi launch](https://github.com/Kkuma99/Boogie_emeddedSW_2020/tree/master/Robot/SBC/turtlebot3_manipulation/turtlebot3_manipulation_bringup)
  <br>
  
  `터틀봇 구동을 위한 오픈 소스 활용`
  
  참고: http://wiki.ros.org/turtlebot3_bringup
  
  ### main (host)
  roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch: [Host PC launch](https://github.com/Kkuma99/Boogie_emeddedSW_2020/tree/master/Robot/Master/turtlebot3_manipulation/turtlebot3_manipulation_gui)
   <br>
  
  `터틀봇 및 오픈 매니퓰레이터 구동을 위한 오픈 소스 활용`
  
  참고: https://github.com/ROBOTIS-GIT/turtlebot3_manipulation
  
   <details>
<summary><span style="color:blue"> 알고리즘 설명 (요약)</span></summary>

```
① ROS 메시지 통신을 이용하여 Jetson으로부터 바코드 데이터를 전송받는다. 
 
② 이때 데이터가 계속해서 들어오는 경우 중복을 방지하기 위해서 중복 검사를 한다. 주소지의 형태로 예를 들어 A00이면 ‘A’,‘0’,‘0’ 이런 식으로 string 데이터가 들어오게 되는데 각각의 문자열을 원래 가지고 있던 배열과 비교를 한다. 만약 각각의 문자열 중 하나라도 다르게 된다면 다른 주소지이므로 그 데이터를 배열에 저장하고 Queue에 해당 데이터를 push한다. 즉 Queue에서는 반복되는 데이터 없이 주소지를 저장을 하여 추후 매니퓰레이터 제어에 사용되게 된다.

 ③ 기존에 GUI를 통하여 데이터를 전달받던 알고리즘을 변경하여 지정된 위치로 Open Manipulator를 제어할 수 있도록 함수를 사용한다. 원본 함수의 경우에는 GUI 창이 틀어지면서 home pose / init pose / gripper open / gripper close 외에 나머지는 xyz의 데이터 혹은 각각의 조인트의 값을 숫자로 입력하여 그 값을 전달해주는 형식으로 로봇팔을 조종한다. 현재 알고리즘에서는 거기서 사용되는 함수를 변경을 하여 이를 gui 입력없이 직접 사용하는 형식으로 변경하였다. 각 배송지 별로 분류하기 위해 로봇팔이 이동해야 하는 위치를 확인하여 이를 리스트로 저장을 하고 그 값을 조건에 따라 전달하여 함수가 실행되도록 한다. float a[4] = {0.550, 0.500, -0.100, 0.500}; 이러한 식으로 저장을 하여 후에 함수에서각각의 조인트 값을 저 리스트의 값으로 실행을 하게 된다.

 ④ 퍼블리셔 서브스크라이버 함수를 통하여 주소지 데이터가 계속적으로 전달이 되고 반복되지 않는 값이 Queue에 쌓이는 동안 로봇팔은 산발적으로 움직이는 것은 아니다.상자가 컨베이어 벨트 초반부분에서 적재 알고리즘을 통해 데이터가 전달이 되지만 로봇팔이 이동을하는 컨베이어 벨트 끝부분까지 도착한 것은 아니다. 상자가 컨베이어의 끝부분에 도달하였는지 확인하기 위해서 OpenCV를 이용하여 카메라로상자를 인식한다. 특정 위치에서 상자가 인식되면 플래그를 바꾸고 로봇팔 제어 함수를 호출한다. 

```
</details>

---
 # ➕ 추후 활용방안
 
 - 택배 회사의 물류 상하차에 활용
 ```    
 이 시스템의 주 목적은 물류센터의 업무 효율을 높이는 것이다. 
 
 빠른 적재 순서 계산을 통하여 여러 번 상하차 진행을 할 필요없이 박스를 전달받은 대로 진행을 하면 되기 때문에 물품을 전달하는 과정에서도 찾는데 시간 절약이 가능하다.
 ```
 
 - 스마트 팩토리의 물건 분류와 재고 관리에 활용 가능
 ```
 스마트 팩토리에서 물건을 분류하고 출고 직전에 물품을 관리하는 데에 활용할 수 있다.
 ```
 
 - 대형 창고 및 물품 보관 서비스 업체에서 활용 가능
 ```
 많은 물품을 보관해야 하는 대형 창고나 물품 보관 서비스 업체에서는 제한되어있는 규모의 공간에 최대한 많은 물품을 보관하는 것이 가장 이득이다. 
 
 크기에 따라 배치를 결정해주는 시스템을 여기에 적용할 수 있다.
 ```

---
 이 후에도 다양한 개발 아이디어를 적용하여 작품을 발전시킬 계획입니다. 감사합니다😄
