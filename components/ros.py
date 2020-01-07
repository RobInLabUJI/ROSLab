import os, sys

versions = ['kinetic-ros-core',   'kinetic-ros-base', 'kinetic-robot', 
            'kinetic-perception', 'kinetic-desktop',  'kinetic-desktop-full',
            'melodic-ros-core',   'melodic-ros-base', 'melodic-robot',
            'melodic-perception', 'melodic-desktop',  'melodic-desktop-full']

DOCKER_CORE_CONTENTS = """
###################################### ROS #####################################

# install packages
RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install -q -y \\
    dirmngr \\
    gnupg2 \\
    lsb-release \\
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install --no-install-recommends -y \\
    python-rosdep \\
    python-rosinstall \\
    python-vcstools \\
    python-catkin-tools \\
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init \\
    && rosdep update

# install ros packages
ENV ROS_DISTRO %s
RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install -y \\
    %s \\
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

RUN mkdir -p /home/jovyan/.ros
RUN chown jovyan.jovyan /home/jovyan/.ros
"""

PACKAGE_VERSION = {'kinetic': '1.3.2-0*',
                   'melodic': '1.4.1-0*'}

def write(DOCKER_FILE, version):
    if version in versions:
        distro = version.split('-')[0]
        config = '-'.join(version.split('-')[1:])
        with open(DOCKER_FILE, "a") as dockerfile:
            dockerfile.write(DOCKER_CORE_CONTENTS % (distro, 'ros-' + version + '=' + PACKAGE_VERSION[distro]))
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


