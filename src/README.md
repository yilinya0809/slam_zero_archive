# K-city에서 해야할 것

## 1. IMU calibration
K-city에 도착하면 IMU calibration을 하자.
### Calibration
우분투에서는 버전 문제로 잘 안 되어서 Windows 사용   
https://www.xsens.com/software-downloads 에서 Download MT Software Suite 클릭하고 윈도우용 다운   
다운받은 exe파일을 실행하여 common과 magnetic field mapper 설치   
imu를 연결하고 magnetic field mapper 실행   
지시사항대로 한 다음   
(새 데이터 사용)Use Motion Tracker 클릭하고 Scan   
스캔된 imu 클릭하고 다음 누르다가 측정 시작   
차를 다양한 방향으로 돌림   
측정 끝 누름   
결과가 나오고 write 누르면 기록 됐다는 창이 뜸   

## 2. slam_para.yaml 변경
calibration을 마쳤다면, 코스트맵을 보고 angle_offset = 0 으로 할지, 45 혹은 -45을 넣을 것인지 결정하자.
is_kcity : true 인지도 확인하자. 코드에 임의로 is_kcity 변수가 할당되어 있는지 살피자.

## 3. 통합 launch 파일 사용
모든 파일을 한번에 launch 하고 싶은 경우, **slam_all.launch** 를 사용하자.

sensor_driver : imu_driver, gps_driver, decoder 관련 코드 실행
obstacle_costmap : obstacle_costmap 관련 코드 실행
pp_publisher : cost_map, global_path, map_layer 관련 코드 실행
heading_correction : angle_offset과 관련 코드 실행

## 4. 기타 우려 사항
+ /angle_1 을 echo 했을 때 제대로 나오지 않는다면, image_publisher의 문제이다. /match1.png의 경로를 확인하고 제대로 들어가있는지 확인하자.
+ /angle_2 를 echo 했을 때 제대로 나오지 않는다면, lane_sector 혹은 lane_costmap_publisher의 문제이다.
lane_costmap_publisher가 잘 작동하는지 확인하려면 rviz를 켜보거나, asdlfkjal.png를 확인해보면 된다.(경로 변경해야됨.) lane_costmap_publisher는 /filtered_data가 알맞게 들어오지 않으면 제대로 작동하지 않으니, 센서들을 다시 살펴보자. 포트에 권한을 줬는지도 확인하자. 
```
sudo chmod 777 /dev/포트이름
```
+ /lane_state를 echo 했을 때 0이면 직선 구간, 1이면 직선 구간이 아닌 것이다. (Kcity_color_map.png에서 검은색인 부분을 직선구간으로 판단하였다.) lane_state가 0이 아닌 경우, lane_costmap_publisher가 작동하지 않고 nan을 뱉어내게 되는데, 이것이 문제가 될 경우를 생각해서 후진 시 혹은 직선 구간이 아닐 경우 lane_costmap이 검은색 바탕에 하얀색 선(세로줄)이 그려진 이미지가 되도록 하여 angle_diff가 0이 되도록 하는 코드를 추가하였다. (lane_costmap_publisher.cpp, line 127, else ...)
이것이 문제가 되면 주석 처리를 하자. (안돌려봐서 잘되는지 모름 ㅎㅎ ㅈㅅ;;)

