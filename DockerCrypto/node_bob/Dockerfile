FROM ros:melodic-ros-base
ENV ROS_PACKAGE_PATH=/opt/ros/melodic/share:/opt/ros/melodic/stacks
ENV LD_LIBRARY_PATH=/opt/ros/melodic/lib
ENV SHLVL=0
ENV ROS_ETC_DIR=/opt/ros/melodic/etc/ros
ENV PKG_CONFIG_PATH=/opt/ros/melodic/lib/pkgconfig
ENV PATH=/opt/ros/melodic/bin:${PATH}
ENV ROS_ROOT=/opt/ros/melodic/share/ros
ENV ROSLISP_PACKAGE_DIRECTORIES=
ENV PYTHONPATH=/opt/ros/melodic/lib/python2.7/dist-packages
ENV CPATH=/opt/ros/melodic/include
ENV CMAKE_PREFIX_PATH=/opt/ros/melodic

WORKDIR /usr/src/node_bob/src
RUN apt-get update && apt-get install -y build-essential  && catkin_init_workspace 

WORKDIR /usr/src
ADD ./cmake-3.13.4-Linux-x86_64.sh /usr/src
RUN mkdir /opt/cmake
RUN sh /usr/src/cmake-3.13.4-Linux-x86_64.sh --prefix=/opt/cmake --skip-license
RUN ln -s /opt/cmake/bin/cmake /usr/local/bin/cmake
RUN cmake --version

WORKDIR /usr/src/node_bob
RUN catkin_make

WORKDIR /usr/src/node_bob/src
RUN catkin_create_pkg node_bob std_msgs roscpp message_generation message_runtime

WORKDIR /usr/src/node_bob/src/node_bob
RUN rm CMakeLists.txt
ADD ./CMakeLists.txt /usr/src/node_bob/src/node_bob
ADD ./include/*.hpp /usr/src/node_bob/src/node_bob/include/node_bob/

WORKDIR /usr/src/node_bob/src/node_bob/msg
ADD ./cipher.msg /usr/src/node_bob/src/node_bob/msg

WORKDIR /usr/src/node_bob/src/node_bob/srv
ADD ./key_generator.srv /usr/src/node_bob/src/node_bob/srv

WORKDIR /usr/src/node_bob/src/node_bob/src
ADD ./NodeBob.cpp /usr/src/node_bob/src/node_bob/src

WORKDIR /usr/src/node_bob
ADD ./entrypoint.sh /usr/src/node_bob
RUN chmod u+x /usr/src/node_bob/entrypoint.sh && mv /usr/src/node_bob/entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]