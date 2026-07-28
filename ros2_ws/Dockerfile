FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-c"]

COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /usr/local/bin/

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-serial \
        ros-dev-tools \
        ros-jazzy-slam-toolbox \
        ros-jazzy-nav2-bringup \
        ros-jazzy-navigation2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY motor_control_system/py ./motor_control_system/py
RUN uv pip install --system --break-system-packages ./motor_control_system/py

COPY ros2_ws/src ./ros2_ws/src
# upstream bug: pthread.h include is commented out, only worked by accident
# on older toolchains that transitively leaked pthread_mutex_t via <chrono>/<string>
RUN sed -i 's|//#include <pthread.h>|#include <pthread.h>|' \
    ./ros2_ws/src/ldlidar_sl_ros2/ldlidar_driver/include/log_module.h
RUN rosdep init 2>/dev/null || true \
    && rosdep update \
    && rosdep install --from-paths ./ros2_ws/src --ignore-src -y

WORKDIR /workspace/ros2_ws
RUN . /opt/ros/jazzy/setup.bash && colcon build

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "source /workspace/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
