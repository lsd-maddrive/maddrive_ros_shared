# основной образ
FROM osrf/ros:noetic-desktop-full

# необходимые apt-пакеты
RUN apt-get update && apt-get install -y \
    curl \
    zsh \
    git \
    sudo \
    dos2unix \
    python3-catkin-tools

RUN sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

# zsh вместо bash
RUN echo "zsh" >> /root/.bashrc

# копируем .zshrc
COPY zshrc /root/.zshrc
RUN dos2unix /root/.zshrc