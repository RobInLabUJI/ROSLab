import os, sys

versions = ['kinetic-ros-core', 'kinetic-ros-base', 'kinetic-robot', 'kinetic-perception', 
            'melodic-ros-core', 'melodic-ros-base', 'melodic-robot', 'melodic-perception']

DOCKER_CORE_CONTENTS = """
###################################### ROS #####################################

# install packages
RUN apt-get update && apt-get install -q -y \\
    dirmngr \\
    gnupg2 \\
    lsb-release \\
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \\
    python-rosdep \\
    python-rosinstall \\
    python-vcstools \\
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init \\
    && rosdep update

# install ros packages
ENV ROS_DISTRO %s
RUN apt-get update && apt-get install -y \\
    %s \\
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
"""

DOCKER_PKG_CONTENTS = """
RUN apt-get update && apt-get install -y \\
    %s \\
    && rm -rf /var/lib/apt/lists/*
"""

ROS_CORE_PACKAGE = {'kinetic': 'ros-kinetic-ros-core=1.3.2-0*',
                    'melodic': 'ros-melodic-ros-core=1.4.1-0*'}

ROS_BASE_PACKAGE = {'kinetic': 'ros-kinetic-ros-base=1.3.2-0*',
                    'melodic': 'ros-melodic-ros-base=1.4.1-0*'}

ROS_ROBOT_PACKAGE = {'kinetic': 'ros-kinetic-robot=1.3.2-0*',
                     'melodic': 'ros-melodic-robot=1.4.1-0*'}

ROS_PERCEPTION_PACKAGE = {'kinetic': 'ros-kinetic-perception=1.3.2-0*',
                          'melodic': 'ros-melodic-perception=1.4.1-0*'}

def write(DOCKER_FILE, version):
    if version in versions:
        distro = version.split('-')[0]
        config = '-'.join(version.split('-')[1:])
        with open(DOCKER_FILE, "a") as dockerfile:
            dockerfile.write(DOCKER_CORE_CONTENTS % (distro, ROS_CORE_PACKAGE[distro]))
            if not config=='ros-core':
                dockerfile.write(DOCKER_PKG_CONTENTS % ROS_BASE_PACKAGE[distro])
            if config=='robot':
                dockerfile.write(DOCKER_PKG_CONTENTS % ROS_ROBOT_PACKAGE[distro])
            if config=='perception':
                dockerfile.write(DOCKER_PKG_CONTENTS % ROS_PERCEPTION_PACKAGE[distro])
        return
    else:
        print("ros: version %s not supported. Options: %s" % (version, versions))
        sys.exit(1)

ROS_ENTRYPOINT_CONTENTS = """#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"
"""

def entry_point():
    with open('ros_entrypoint.sh', 'w') as output_file:
        output_file.write(ROS_ENTRYPOINT_CONTENTS)
    os.chmod('ros_entrypoint.sh', 0o755)
    return


