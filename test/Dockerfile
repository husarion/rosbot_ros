FROM ros:melodic

WORKDIR /ros_test_ws

SHELL ["/bin/bash", "-c"]

COPY src src

RUN sudo apt-get update -y && \
    sudo apt-get install --no-install-recommends -y \
    ros-melodic-move-base-msgs &&\
    source /opt/ros/melodic/setup.bash &&\
    catkin_make

COPY ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

COPY wait4master.sh /

RUN chmod +x /wait4master.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
