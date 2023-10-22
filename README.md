# maddrive_ros_shared

Здесь хранятся общие пакеты и инструменты, используемые для сборки роботов lsd-maddrive. В папке [scripts](scripts) находятся скрипты для загрузки пакетов. Скрипты сборки самих роботов находятся непосредственно в репозитории робота

## Инструкция по работе с контейнером

* сборка изображения

```bash
docker image build -t lsd-maddrive-ros:noetic-desktop-full .
```

* запуск контейнера в хост-системе Linux

```bash
docker container run -it \
    --name=TYPE_PROJECT_NAME \
    --user=user1122 \
    --network=host \
    --ipc=host \
    --volume=$HOME/catkin_ws/src:/home/user1122/catkin_ws/src \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
    --env=DISPLAY \
    --device=/dev/input/js0 \
    lsd-maddrive-ros:noetic-desktop-full
```

* с помощью флага `--device` передайте путь до файла подключаемого устройства (можно узнать с помощью команды `dmesg`)
пример (передадим в контейнер файлы лидара и джойстика):

```bash
--device=/dev/ydlidar \
--device=/dev/input/js0
```

* расширяем конфигурацию catkin параметрами из `/opt/ros/noetic`

```bash
sudo catkin config --extend /opt/ros/noetic
```

* если не появляется GUI, выполняем в хост-системе команду `xhost +`
