# Заметки по разработке передатчика координат из Raspberry Pi 5 на Android смартфон

Настройка IMU ноды:  
git clone https://github.com/kimsniper/ros2_mpu6050.git  
cd ros2_mpu6050  
colcon build --packages-select ros2_mpu6050  
source install/setup.bash  
ros2 run ros2_mpu6050 ros2_mpu6050_calibrate  
Скопировать полученные в выводе значения в config/params.yaml  
colcon build --packages-select ros2_mpu6050  
ros2 launch ros2_mpu6050 ros2_mpu6050.launch.py  
Для проверки топика: ros2 topic echo /imu/mpu6050  


Разработка и тестирование 08.12.2024:
- Создал директорию map, cd map
- Запустил ORB_SLAM3:
```Shell
ros2 run orbslam3_pose mono ~/ORB_SLAM3/Vocabulary/ORBvoc.txt ~/ORB_SLAM3/Examples/Monocular/TUM1.yaml --ros-args -r __ns:=/ -r /anafi/camera/image:=/camera
```
- Запустил ноду камеры:
```Shell
ros2 run opencv_tools image_publisher
```
- Для тестов инициализировал текущие начальные значения latitude и longitude в gps_simulator.c по текущим из gps телефона.
- После запуска ORB_SLAM3 и ноды камеры просмотрел топики ros2 topic list. В результате подходят /odom, /pose_orb1 и /pose_orb2.
- ros2 topic echo /odom --once, подвинул камеру, ros2 topic echo / odom --once. Данные обновились, топик /odom подходит. Остальные топики имеют аналогичные значения. У /odom есть также данные ковариации, поворота (twist) и ещё несколько, но они занулены.
- Проверка частоты публикации:
```Shell
ros2 topic hz /odom
```
В пределах 20 Гц.
- Структура сообщений:
```Shell
ros2 interface show geometry_msgs/msg/PoseStamped
# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
Pose pose
	Point position
		float64 x
		float64 y
		float64 z
	Quaternion orientation
		float64 x 0
		float64 y 0
		float64 z 0
		float64 w 1
```
