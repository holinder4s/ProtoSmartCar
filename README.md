# ProtoSmartCar
STM32보드를 이용한 Prototype SmartCar제작

## 1. Development Environment
### 1.1 Equipment
* D-Stream
* ARM STM32 board(stm32f10x)
### 1.2 Compiler & IDE
* DS-5 Eclipse
### 1.3 Program Language
* C

## 2. Function
### 2.1 지문 등록 및 매치
    최초 1회 관리자 지문을 3회에 걸쳐 등록한다. 이후 버튼1을 누르면 관리자 지문을 인식한 후 매치 성공 시 모든 등록된 지문을 초기화한 후 관리자와 추가 1인의 지문을 등록받는다. 또한 버튼2를 누르면 차의 문이 열린다.
### 2.2 조도센서 활용 라이트 제어
    조도센서를 활용하여 주변의 밝기를 수집한 후 어두운 경우(밤인 경우) 라이트를 켜고, 밝은 경우(낮인 경우) 라이트를 끈다.
### 2.3 빗물감지센서를 활용한 와이퍼 제어
    빗물감지센서를 활용하여 비가 적게 올 경우 와이퍼를 1초 간격으로 작동시키며, 많이 올 경우 0.5초 간격으로 작동시킨다.
### 2.4 크래시 로그
    정지 시 또는 주행 시 차 범퍼의 압력 감지센서에 충돌이 감지되면, LCD에 사고 당시의 속력과 충격량을 표시한다.
### 2.5 거리 센서를 활용한 차량 정지
    LCD의 5번째 표시되는 값이 거리센서의 수치이며, 5cm이내에 장애물이 탐지되면, 차량을 멈춘다.
### 2.6 음성인식센서를 활용한 명령 수행
    0) "Hi cell"이라는 명령으로 명령어 인식상태로 활성화 시킨다.
    1) "Turn on the TV"라는 명령에 의해 차량의 시동을 켠다.(LCD on & 기타 장치 동작)
    2) "Turn off the TV"라는 명령에 의해 차량의 시동을 끈다.
    3) "Open the door"라는 명령에 의해 차량의 문을 연다.
    4) "Close the door"라는 명령에 의해 차량의 문을 닫는다.
    5) "Turn on the Light", "Turn off the Light"라는 명령으로 라이트를 켜고 끈다.
### 2.7 블루투스모듈을 활용한 수동 차량 제어(안드로이드 어플)
    안드로이드 어플리케이션을 이용하여 수동으로 차량을 제어(전진/후진/좌회전/우회전, 라이트 ON/OFF, 시동 ON/OFF)한다.

## 3. Demo
- https://youtu.be/JSvViEZcYjA

## 4. Author
- Name : Woojin, Lee
- Nickname : holinder4s
- blog : holinder4s.tistory.com

## 5. Contact
- e-mail : holinder4s@gmail.com

## test
