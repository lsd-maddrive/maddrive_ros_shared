# Общие заметки по всем проектам

## Запуск докера из под прокси
```bash
sudo mkdir -p /etc/systemd/system/docker.service.d
sudo nano /etc/systemd/system/docker.service.d/http-proxy.conf
```
В файл запишите как минимум следующее:
```Systemd
[Service]
Environment="HTTP_PROXY=http://10.128.0.90:8080"
Environment="HTTPS_PROXY=http://10.128.0.90:8080"
Environment="NO_PROXY=localhost,127.0.0.1"
```
После выполните 
```bash
sudo systemctl daemon-reload
sudo systemctl restart docker
```
Проверка:
```bash
sudo systemctl show --property=Environment docker
```

terminate called after throwing an instance of 'c10::CUDAError'
  what():  CUDA error: no kernel image is available for execution on the device
CUDA kernel errors might be asynchronously reported at some other API call,so the stacktrace below might be incorrect.
For debugging consider passing CUDA_LAUNCH_BLOCKING=1.

[Как проверить, не работает ли сетевое соединение или удаленный ROS Master?](https://answers.ros.org/question/301135/how-to-check-if-network-connection-or-remote-ros-master-is-down/)
[Запуск rtabmap из докера](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## add user to tty / DIALOUT
добавить пользователея в группу tty
```bash
sudo usermod -a -G dialout $USER
```
logout or reboot
Проверить есть ли пользователь в groups
```bash
groups
```
## UDEV для камер по пути
```/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1.1:1.0-video-index0``` [удев для камер по path](https://askubuntu.com/questions/715333/assign-webcam-to-a-specific-dev-video)

## Калибровка джойстика
https://askubuntu.com/questions/421822/jstest-gtk-not-saving-profile
Для сохрания текущей конфигурации jscal-store
При подключении вызывается jscal-restore
Удалить конфиг можно из папки /var/lib/joystick

## install python requirements
```bash
pip install -r /path/to/requirements.txt
```

## shutdowm over ssh
 В то время как вы подключены и терминал явно указывает, что текущая эвм -- Бортовой компьютер
```bash
sudo shutdown now
```
[Отключение по SSH](https://askubuntu.com/questions/251148/shutdown-over-ssh)

## Systemd: Service File Examples
[Systemd example](https://www.shellhacks.com/systemd-service-file-example/)
[Systemd HOWTO](https://linuxhandbook.com/create-systemd-services/)

## Robot upstart guide
[Дополнительная информация о настройке автозапуска](https://roboticsbackend.com/make-ros-launch-start-on-boot-with-robot_upstart/)


```bash
rosrun robot_upstart install nuke_software/launch/full_robot_start.launch --job nuke_robot_ros --symlink --logdir /home/alakey/upstart_logs
sudo systemctl start nuke_robot_ros # запустить вручную
sudo systemctl stop nuke_robot_ros
rosrun robot_upstart uninstall nuke_robot_ros
sudo systemctl disable my_robot_ros.service # выключить автозапуск
sudo systemctl enable my_robot_ros.service
```
Для запуска на нескольких машинах переходим в /usr/sbin/nuke_robot_ros-start и меняем ROS_HOSTNAME.
Папка для записи логов должна находится в папке HOME. Для уточнения папки можно открыть файл /usr/sbin/nuke_robot_ros-start и посмотреть переменную log_path в 12 строке. После смены папки выполнить `sudo systemctl daemon-reload`

Очередь запуска: clearpathrobotics/robot_upstart#71 [Автозапуск по подключению USB или вроде того](https://superuser.com/questions/1240367/how-to-wait-for-a-usb-flash-drive-to-be-mounted-present-before-running-services)

# Launch Guide

1. Connect **this** PC and **onboard** one to the same network,
    1. Set up a network to **onboard** device to have ```192.168.50.154``` Static IP addres,
    1. Set up a network to **this** device to have ```192.168.50.XXX``` Static IP addres,
3. SSH to **onboard** PC with alias ```connect-robot```,
2. Execute ```rosatom-pc``` alias (leads to full_robot_start.launch),
4. On **this** PC execute ```rosatom``` to launch gui.


# TEST ***(ONLY USE THIS INCTRUCTIONS IF U ANDERSTAND WHAT U DO)***
instructions to test partly/full

## Disable 2nd PC connection to run ROS localy
DO 
```bash 
sudo nano ~/.zshrc
``` 
at the very botom find ROS related export and coment out
shoud' be:
```bash
export ROS_HOSTNAME=192.168.50.155
export ROS_MASTER_URI=http://192.168.50.155:11311
```

After that you'll be able to run test purpose launch file
```bash 
roslaunch nuke_software full_robot_start.launch
```
localy with out second PC and UI

## CAN problems

If can node reports error 3 (No device or smth) try:

1. CHK if device connected
2. connect and restart PC
3. IF 2 is not an option:
    1. execute 
    ```bash
    sudo systemctl start I7CAN
    ```
    2. chk it with comand below, u should see SUCSESS word with reacent date
    ```bash 
    systemctl status I7CAN
    ``` 
    3. chk if new /dev/ttyUSB* device arived
        
## joy emul
rostopic pub /joy/f710 sensor_msgs/Joy '{ header: {seq: 10, stamp: {secs: 1431222430, nsecs: 345678}, frame_id: "3"}, axes: [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]}'

## UI problems

## UI deps
```shell
python3 -m pip install PyQt5
```

### v4l dev list
```shell
v4l2-ctl --list-devices
```

also 
```shell 
ll /dev/v4l/by-path | grep index0
```

## wifi cli 
https://ubuntu.com/core/docs/networkmanager/configure-wifi-connections
```shell
nmcli d wifi list
sudo nmcli d wifi rescan
nmcli d wifi connect RDMTSG
nmcli d wifi connect ASUS_DEMO_NET
```
## PyQT Seg Fault
https://stackoverflow.com/questions/21857935/pyqt-segmentation-fault-sometimes

## Create desktop 
https://www.cyberciti.biz/howto/how-to-install-and-edit-desktop-files-on-linux-desktop-entries/

## Iris Xe problems on ubuntu 20.04
https://askubuntu.com/questions/1299067/ubuntu-20-04-no-driver-loaded-for-intel-iris-xe-graphics
https://askubuntu.com/questions/1521777/graphics-drivers-with-ubuntu-20-04
