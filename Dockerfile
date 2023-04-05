FROM ros:noetic-ros-core

LABEL maintainer Bassam Pervez

# Trick to get apt-get to not prompt for timezone in tzdata
ENV DEBIAN_FRONTEND=noninteractive

ARG STARTDELAY=5
ENV STARTDELAY=$STARTDELAY

RUN rm -rf /var/lib/apt/lists

# Install gstreamer dependencies 
RUN sudo apt-get update 

RUN sudo apt-get install -y python3  ros-noetic-rospy ros-noetic-cv-bridge ros-noetic-tf ros-noetic-mavros ros-noetic-mavros-msgs



COPY controller.py /
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT /entrypoint.sh 


