# PX4-Gazebo Aruco Challenge

[![PX4-Gazebo Aruco Challenge](https://img.youtube.com/vi/25MkysUniwA/0.jpg)](https://www.youtube.com/watch?v=25MkysUniwA "Video Demonstration")

## Подготовка

1. Установить ROS2 по [инструкции](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
2. Установить Gazebo по [инструкции](https://gazebosim.org/docs/harmonic/install_ubuntu)
3. Установить [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)
4. Установить PX4 из [репозитория](https://github.com/Bogdanov-am/PX4-Autopilot) по [инструкции](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html) (в первом шаге заменить репозиторий на https://github.com/Bogdanov-am/PX4-Autopilot). После установки предварительно собрать модель
```
make px4_sitl gz_x500_mono_cam_baylands
```
5. Установить micro-xrce-dds-agent
```
sudo snap install micro-xrce-dds-agent --edge
```
6. Создать workspace, установить и собрать зависимости
```
mkdir aruco_ws
cd aruco_ws
mkdir src build install
cd src
git clone https://github.com/Bogdanov-am/px4-challenge
git clone https://github.com/Bogdanov-am/ros2_aruco
git clone https://github.com/PX4/px4_msgs
cd ..
colcon build

```
## Запуск
1. Запустить QGroundControl
2. Запустить симуляцию с автопилотом и подождать полной загрузки 
```
PX4_GZ_MODEL_POSE="136 36 -0.25 0 0 0" make px4_sitl gz_x500_mono_cam_baylands
```
3. Запустить micro-xrce-dds-agent
```
micro-xrce-dds-agent udp4 -p 8888
```
4. Запустить слежение за aruco меткой
```
ros2 launch px4_challenge aruco_follower.launch.py
```
5. Поднять дрон в воздух и перевести в OFFBOARD режим как на [видео](https://www.youtube.com/watch?v=25MkysUniwA)
6. **IT'S MAGICK**