# TNP-base-image
# This docker image is equipped with ros-robot, git, eclipse, gazeboo, sublimetext3, cuda and cudann and zsh terminal.
#
# Version:  2017.07.31
# Authors:  Members of the Team NAIST-Panasonic at the Amazon Robotics Challenge 2017
#           Gustavo A. Garcia R. garcia-g at is.naist.jp (Captain), 
#           Lotfi El Hafi, Felix von Drigalski, Wataru Yamazaki, Viktor Hoerig, Arnaud Delmotte, 
#           Akishige Yuguchi, Marcus Gall, Chika Shiogama, Kenta Toyoshima, Pedro Uriguen, 
#           Rodrigo Elizalde, Masaki Yamamoto, Yasunao Okazaki, Kazuo Inoue, Katsuhiko Asai, 
#           Ryutaro Futakuchi, Seigo Okada, Yusuke Kato, and Pin-Chu Yang
# *******************
# Copyright 2017 Team NAIST-Panasonic 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at 
#     http://www.apache.org/licenses/LICENSE-2.0 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# *******************

# kinetic-robot bases on the ubuntu-xenial 16.04 image
FROM ros:kinetic-robot

ENV HOME /root
ENV LC_ALL en_US.UTF-8
ENV TERM xterm-color
ENV DEBIAN_FRONTEND noninteractive
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}
LABEL com.nvidia.volumes.needed="nvidia_driver"

# Change to bash
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# install system tools
RUN echo 'deb-src http://us.archive.ubuntu.com/ubuntu/ xenial main restricted' >> /etc/apt/sources.list && \
	echo 'deb-src http://us.archive.ubuntu.com/ubuntu/ xenial-updates main restricted' >> /etc/apt/sources.list &&\
	echo 'deb-src http://us.archive.ubuntu.com/ubuntu/ xenial-backports main restricted universe multiverse' >> /etc/apt/sources.list && \
	echo 'deb-src http://security.ubuntu.com/ubuntu xenial-security main restricted' >> /etc/apt/sources.list && \
	apt-get update && \
	apt-get -y upgrade && \
	apt-get install -y \
	apt-src \
	apt-utils \
	build-essential \
	language-pack-en \
	software-properties-common && \
	locale-gen en_US.UTF-8 && \
	dpkg-reconfigure locales && \
	apt-get clean && \
	rm -rf /var/lib/apt/lists/* 

# Install support software packages
RUN add-apt-repository ppa:webupd8team/java -y && \
	add-apt-repository ppa:webupd8team/sublime-text-3 -y && \
	echo oracle-java8-installer shared/accepted-oracle-license-v1-1 select true | /usr/bin/debconf-set-selections && \
	apt-get update && \
	apt-get -y upgrade && \
	apt-get install -y \
	bash-completion \
	ccache \
	curl \
	dnsutils \
	git \
	gdb \
	inetutils-tools \
	inetutils-ping \
	inetutils-traceroute \
	language-pack-en \
	less \
	libcanberra-gtk-module \
	libgtk2.0-0 \
	libxext-dev \
	libxrender-dev \
	libxtst-dev \
	locales \
	mlocate \
	man-db \
	nano \
	oracle-java8-installer \
	sublime-text-installer \
	sudo \
	tmux \
	unzip \
	usbutils \
	vim \
	wget \
	zsh \
	&& apt-get clean \
	&& rm -rf /var/lib/apt/lists/* 

# install ssh
RUN apt-get -yq update && \
	apt-get -yqq install ssh && \
	rm -rf /var/lib/apt/lists/*

# install ROS components
RUN	echo 'deb-src http://packages.ros.org/ros/ubuntu xenial main' >> /etc/apt/sources.list.d/ros-latest.list && \
	echo 'deb http://packages.osrfoundation.org/gazebo/ubuntu xenial main' >> /etc/apt/sources.list.d/gazebo-latest.list && \
	apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743 && \
	apt-get update && \
	apt-get install -y \
	arduino-core \
	libgtest-dev \
	python-catkin-tools \
	python-serial \
	ros-kinetic-effort-controllers \
	ros-kinetic-forward-command-controller \
	ros-kinetic-joint-state-controller \
	ros-kinetic-position-controllers \
	ros-kinetic-joint-trajectory-controller \
    ros-kinetic-gazebo-ros-control \
	ros-kinetic-moveit \
	ros-kinetic-rqt \
	ros-kinetic-rqt-common-plugins \
	ros-kinetic-realsense-camera \
	ros-kinetic-rqt-robot-plugins \
	ros-kinetic-ros-control \
	ros-kinetic-ros-tutorials \
	ros-kinetic-velocity-controllers \
	ros-kinetic-vision-visp && \
	apt-get clean && \
	rm -rf /var/lib/apt/lists/*

# filling missing gazebo models
RUN mkdir -p /root/.gazebo/models && \
	wget -q https://bitbucket.org/osrf/gazebo_models/get/5c4223ac1ea2.zip -P /tmp && \
	unzip /tmp/5c42* -d /tmp && \
	mv /tmp/osrf-gazebo*/* /root/.gazebo/models && \
	rm -r /tmp/*5c42*

## install CUDA
RUN wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/cuda-repo-ubuntu1604_8.0.61-1_amd64.deb -P /tmp \
	&& sudo dpkg -i /tmp/cuda-repo-ubuntu1604_8.0.61-1_amd64.deb \
	&& apt-get update \
	&& apt-get install -y cuda \
	&& rm /tmp/cuda*

## install cudnn8
COPY cudnn-8.0-linux-x64-v5.1.tgz /tmp/
RUN tar xzvf /tmp/cudnn-8.0-linux-x64-v5.1.tgz -C /tmp \
	&& cp -a /tmp/cuda/lib64/* /usr/local/cuda/lib64/ \
	&& mkdir -p /usr/local/cuda/include \
	&& cp -a /tmp/cuda/include/* /usr/local/cuda/include/ \
	&& ldconfig \
	&& echo -e "\n## CUDA and cuDNN paths" >> ~/.bashrc \
	&& echo 'export PATH=/usr/local/cuda/bin:${PATH}' >> ~/.bashrc \
	&& echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}' >> ~/.bashrc \
	&& rm -R /tmp/cud*

# install ML components
RUN apt-get update  \
	&& apt-get install -y \
		python3 \
		python3-pip \
	&& wget https://bootstrap.pypa.io/get-pip.py \
	&& python get-pip.py \
	&& pip install tensorflow-gpu \
	&& pip install ipython \
	&& pip install cython \
	&& rm get-pip.py \
	&& apt-get clean \
	&& rm -rf /var/lib/apt/lists/*

# install graphic components
RUN apt-get update \
	&& apt-get install -y \
		libglfw3-dev \
		libepoxy-dev \
		libgoogle-glog-dev \
		libatlas-base-dev \
		libsuitesparse-dev \
		libglm-dev \
	&& apt-get clean \
	&& rm -rf /var/lib/apt/lists/*

RUN wget http://ceres-solver.org/ceres-solver-1.12.0.tar.gz -P /tmp \
	&& mkdir -p /opt/ceres-solver-1.12.0/build \
	&& tar zxf /tmp/ceres-solver-1.12.0.tar.gz -C /opt/ \
	&& cd /opt/ceres-solver-1.12.0/build \
	&& cmake .. \
	&& make -j6 \
	&& make install \
	&& rm -R /tmp/ceres* \
	&& git clone https://github.com/sdmiller/cpu_tsdf.git /opt/cpu_tsdf \
	&& mkdir -p /opt/cpu_tsdf/build \
	&& cd /opt/cpu_tsdf/build \
	&& cmake .. \
	&& make

## Photoneo scanner 
USER root
ADD PhotoneoPhoXiControlInstaller-1.1.5-ubuntu16-64.run /tmp/PhotoneoPhoXiControlInstaller-1.1.5-ubuntu16-64.run

RUN cd /tmp \
    && chmod a+x PhotoneoPhoXiControlInstaller-1.1.5-ubuntu16-64.run
RUN cd /tmp && ./PhotoneoPhoXiControlInstaller-1.1.5-ubuntu16-64.run \
    && rm PhotoneoPhoXiControlInstaller-1.1.5-ubuntu16-64.run

ENV PHOXI_CONTROL_PATH="/opt/PhotoneoPhoXiControl"

# compile gtest
RUN cd /usr/src/gtest \
	&& cmake CMakeLists.txt \
	&& make \
	&& cp *.a /usr/lib 

# install eclipse
RUN wget http://ftp.jaist.ac.jp/pub/eclipse/technology/epp/downloads/release/neon/2/eclipse-cpp-neon-2-linux-gtk-x86_64.tar.gz -P /tmp && \
	cd /opt && \
	tar -xzvf /tmp/eclipse-cpp-neon-2-* && \
	rm -r /tmp/eclipse-cpp-neon-2-* && \
	ln -s /opt/eclipse/eclipse /usr/local/bin && \
	echo "-Dswt.enable.themedScrollBar=true" >> /opt/eclipse/eclipse.ini

# Activate code completion
RUN echo "if ! shopt -oq posix; then" >> ~/.bashrc && \
 echo "    if [ -f /usr/share/bash-completion/bash_completion ]; then" >> ~/.bashrc && \
 echo "     . /usr/share/bash-completion/bash_completion" >> ~/.bashrc && \
 echo "    elif [ -f /etc/bash_completion ]; then" >> ~/.bashrc && \
 echo "     . /etc/bash_completion" >> ~/.bashrc && \
 echo " fi" >> ~/.bashrc && \
 echo "fi" >> ~/.bashrc

# Install and configure zsh-terminal
WORKDIR /root
RUN git clone git://github.com/robbyrussell/oh-my-zsh.git ~/.oh-my-zsh && \
	  mkdir -p ${ZSH_CUSTOM}/plugins/git-auto-status && \
	  git clone https://gist.github.com/475ee7768efc03727f21.git ${ZSH_CUSTOM}/plugins/git-auto-status && \
      cp ~/.oh-my-zsh/templates/zshrc.zsh-template ~/.zshrc && \
      sed -i -E "s/^plugins=\((.*)\)$/plugins=(\1 git git-auto-status gitfast tmux)/" ~/.zshrc

# setup dotfiles
RUN git clone https://bitbucket.org/scpeters/unix-stuff \
 && cp $HOME/unix-stuff/.gdbinit $HOME \
 && echo 'export PATH=/usr/lib/ccache:$HOME/bin:$PATH' >> $HOME/.bashrc \
 && rm -r $HOME/unix-stuff

# gazebo
EXPOSE 11345
# roscore
EXPOSE 11311

CMD ["/bin/bash"]
