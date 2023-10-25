# основной образ
FROM osrf/ros:noetic-desktop-full

# добавляем пользователя по умолчанию
ARG USERNAME=user1122
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# копируем .bashrc
COPY bashrc /home/${USERNAME}/.bashrc

# необходимые apt-пакеты
RUN apt-get update && apt-get install -y \
    sudo \
    git \
    wget \
    xxd \
    && rm -rf /var/lib/apt/lists/*

# утилита catkin
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# RUN wget https://github.com/IntelRealSense/librealsense/raw/master/scripts/libuvc_installation.sh
# RUN chmod +x ./libuvc_installation.sh
# RUN ./libuvc_installation.sh

# необходимые библиотеки для ROS-пакетов
COPY scripts/install_packages.sh /home/${USERNAME}/
RUN chmod +x /home/${USERNAME}/install_packages.sh
RUN ./home/${USERNAME}/install_packages.sh
RUN rm -rf /var/lib/apt/lists/*

COPY scripts/install_hardware_moduls.sh /home/${USERNAME}/
RUN chmod +x /home/${USERNAME}/install_hardware_moduls.sh
RUN ./home/${USERNAME}/install_hardware_moduls.sh