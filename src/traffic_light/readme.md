# traffic_light



**작성자 : 선준호 (s.junho@snu.ac.kr)**

**신호등 ROS Publisher**

```topic``` 이름 : ```/traffic_light```

```node``` 이름 : ```traffic_light_node```



## 파일 종류

**DEPRECATED** ```customDetectROS.py``` : ~~신호등 인식 및 ROS Publisher~~

**DEPRECATED** ```customDetectVisual.py``` : ~~신호등 인식 및 시각화~~

**Will Be Deprecated Soon** ```customDetectVisualROS.py``` : 신호등 인식 및 시각화, ROS Publisher

```traffic_light_publisher``` : ```customDetectVisualROS.py```와 동일



## 사용 방법

1. Anaconda 가상환경 생성 (```conda create -n NAME python=3.9```)

2. 가상환경에서 ```pip install -r requirments.txt``` 실행하여 패키지 설치

3. 가상환경에서 ```pip install rospkg``` 실행하여 ```rospkg``` 설치

4. 파일 상단의 인터프리터 경로를 가상환경의 인터프리터 경로로 변경

5. 파일의 ```WEIGHTS```를 모델의 weight 파일의 경로로 변경

6. 파일의 ```FPS```를 사용하는 프레임레이트로 변경

   

## Class 설명

| Class ID |     신호등      |           설명           |
| :------: | :-------------: | :----------------------: |
|    -1    |    None (X)     |      신호등이 없음       |
|    0     |     Red (R)     |  직진 및 좌회전 불가능   |
|    1     |  Red Left (RL)  | 직진 불가능, 좌회전 가능 |
|    2     |    Green (G)    | 직진 가능, 좌회전 불가능 |
|    3     | Green Left (GL) |   직진 및 좌회전 가능    |



## 기타 사항

1. 카메라 설정 개선 필요

   - 연결 안정화

   - 자동 노출, 밝기, 대비 등
2. 본 프로그램은 ```yolov5```의 출력 값을 후처리하여 publish 하고 있음
3. 주간 데이터로 학습된 모델의 경우 야간 성능을 보장하지 못함 (새로운 모델 학습 예정)
4. 학습된 model 파일(```.pt``` 파일)은 구글 드라이브 ```/국제대학생대회/2021/VISION/traffic_light_experiments/*/weights/*.pt``` 참고

