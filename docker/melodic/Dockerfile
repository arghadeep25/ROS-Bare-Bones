FROM ros:melodic

RUN apt-get update && apt-get install -y --no-install-recommends \
  && apt-get install -y --no-install-recommends \
  wget nano build-essential libomp-dev clang lld \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace'

COPY . /root/catkin_ws/src/rosbarebones/

WORKDIR /root/catkin_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'
RUN sed -i "6i source \"/root/catkin_ws/devel/setup.bash\"" /ros_entrypoint.sh
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /
ENTRYPOINT ["/ros_entrypoint.sh"]
#CMD ["bash"]
CMD ["roslaunch", "ros_bare_bones", "ros_bare_bones.launch"]