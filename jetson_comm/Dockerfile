FROM osrf/ros:humble-desktop

SHELL [ "/bin/bash", "-c" ]

WORKDIR /app

COPY jetson_comm ros2_ws/src/

RUN cd ros2_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build

COPY jetson_comm/ros_entrypoint.sh /

ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD [ "bash" ]