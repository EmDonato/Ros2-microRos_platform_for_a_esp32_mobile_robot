# Usa un'immagine di Ubuntu come base
FROM ubuntu:22.04

# Impostazioni per l'installazione di ROS2
RUN apt-get update && apt-get install -y \
    curl \
    lsb-release \
    gnupg \
    sudo \
    bash \
    python3-pip \
    python3-argparse \
    python3-rosinstall-generator \
    && rm -rf /var/lib/apt/lists/*

# Aggiungi il repository ROS2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos > ros2.repos && \
    vcs import src < ros2.repos

# Aggiungi la chiave GPG per i pacchetti ROS
RUN curl -sSL https://github.com/ros/rosdistro/raw/master/ros.asc | tee /etc/apt/trusted.gpg.d/ros.asc

# Impostazioni per l'installazione di pacchetti ROS2
RUN echo "deb [signed-by=/etc/apt/trusted.gpg.d/ros.asc] http://repos.ros.org/ros2/ubuntu/ $(lsb_release -c | awk '{print $2}') main" | tee /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-humble-desktop && rm -rf /var/lib/apt/lists/*

# Installazione micro-ros-agent
RUN apt-get install -y micro-ros-agent


# Esegui rosdep per installare le dipendenze
RUN rosdep update && rosdep install --from-paths /workspace --ignore-src -r -y


# Installa le dipendenze Python di micro-ROS
RUN pip3 install -U setuptools

# Esegui il comando di build
RUN . /opt/ros/humble/setup.sh && colcon build

# Sorgenti dell'ambiente ROS 2 e avvio di micro-ROS agent
CMD ["bash", "-c"]


