FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip python3-venv

WORKDIR /ros2_ws

# --- Install Optitrack driver ---
COPY ./optitrack/ /ros2_ws/src/

# Create one venv only
RUN python3 -m venv --system-site-packages /ros2_ws/venv \
 && touch /ros2_ws/venv/COLCON_IGNORE

RUN bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    source /ros2_ws/venv/bin/activate && \
    pip install --upgrade pip msgpack eclipse-zenoh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    PYTHON_EXECUTABLE=/ros2_ws/venv/bin/python colcon build --symlink-install \
"

# --- Install your main project ---
COPY ./src/ /ros2_ws/src/

RUN bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    source install/setup.bash && \
    colcon build --symlink-install --packages-select zenoh_ros2_bridge \
"

COPY ./ros_entrypoint.sh /ros_entrypoint.sh

ARG robot_0="" \
    robot_1="" \
    robot_2="" \
    robot_3="192.168.1.161:11811" \
    robot_4="" \
    robot_5="192.168.1.132:11811" \
    robot_6="" \
    robot_7="" \
    robot_8="" \
    robot_9=""

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_SUPER_CLIENT=True \
    ROS_DOMAIN_ID=0 \
    ROS_DISCOVERY_SERVER=${robot_0};${robot_1};${robot_2};${robot_3};${robot_4};${robot_5};${robot_6};${robot_7};${robot_8};${robot_9};

CMD ["ros2", "launch", "zenoh_ros2_bridge", "zenoh_ros2_bridge.launch.py"]
