# Need run the command "xhost +" in bash to be able to display GUI applications in docker and maybe "export DISPLAY=:0.0"
# BUILD: sudo docker build -t carla_script .
# RUN: sudo docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --net=host carla_script

FROM python:3.8-slim-buster

WORKDIR /projeto_informatico

COPY . /projeto_informatico/

RUN apt-get update && apt-get install -y libgl1-mesa-glx libgomp1 libgtk2.0-dev pkg-config mesa-utils

# To use the shared library
RUN apt install -y cmake libpcl-dev
WORKDIR /projeto_informatico/utils/ground_truth/build
RUN cmake .. && make

WORKDIR /projeto_informatico

RUN pip install carla==0.9.15
RUN pip install numpy
RUN pip install open3d-cpu
RUN pip install opencv-python
RUN pip install --no-cache-dir pcl
RUN pip install pygame
RUN pip install Pillow

CMD /bin/bash