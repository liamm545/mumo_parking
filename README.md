# se2_navigation 사용법

## 해야할 것

~~##### 1. 장애물 충돌 관련 처리
장애물을 만들 순 있는데, 지금 충돌이 안되어서 관련 처리를 진행하고 있습니다.
관련 패키지 -> se2_grid_map_generator, se2_planning (특히 GridMapStateValidator.cpp 파일)~~

##### 2. 실차 적용 관련
10.9일에 실차에서 테스트해 본 결과, 조향은 잘 들어가는데 속도가 좀 문제가 있습니다.
erp에 들어가는 속도 제어값이 너무 적어서 거의 안 굴러가요
그런데 속도를 크게 하면 rviz상에서 경로를 제대로 추종하지 못합니다.
!!따라서!!
여기서 구현되어있는 자동차의 setting을 erp 용으로 맞추는 작업을 해야할 거 같습니다. (휠베이스, 조향각 등등)
erp42 ROS 패키지에, erp42_vehicle 패키지가 있는데 여기를 보고 적용하면 될 것 같습니다.

## 실행법

##### 1. car_demo 실행

    roslaunch car_demo demo_autonomous.launch

##### 2. grid map generator 실행

    roslaunch se2_grid_map_generator se2_grid_map_generator.launch

이후 [ INFO] [1728476251.814679445, 13.837000000]: GridMap received successfully 가 뜨면 됩니다.

##### 3. 장애물 넣어보기
```
rosservice call /se2_grid_map_generator_node/addPolygonObstacle "obstacle:
  polygon:
    vertices:
      - x: {data: 2.0}
        y: {data: -4.0}
      - x: {data: 3.0}
        y: {data: -4.0}
      - x: {data: 3.0}
        y: {data: -5.0}
      - x: {data: 2.0}
        y: {data: -5.0}
  layers:
    - data: 'obstacle'
  values:
    - data: 1.0"
```
좌표는 알아서 설정, layer는 그대로 두기

만약 맵을 초기화 시키고 싶으면
```
rosservice call /se2_grid_map_generator_node/resetMap
```

##### 4. 경로 Planning 및 Tracking

Start 위치와 Goal 위치를 적절히 조정한 뒤, Planning Panel의 "Request Plan"을 누르면 경로 Planning 진행

그 후 "Start Tracking"을 누르면 트래킹 진행

## 참고

##### 1. 빌드가 안돼요
ompl 관련 오류일 경우 -> slack의 주차 채널의 제 메시지 확인
erp42_driver 관련 오류일 경우 -> car_demo 패키지의 PriusControllerRos.cpp 에서 translateCommands, publishControl, initRos 세 함수를 주석 처리하고, 바로 위에 주석 처리되어있는 코드로 바꿉니다.
그런데 함수를 바꿀 경우 erp42용 drive랑 mode 토픽이 안 나오긴 합니다.

