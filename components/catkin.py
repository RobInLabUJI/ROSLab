DOCKER_CONTENTS = """
#################################### CATKIN ####################################

RUN mkdir -p ${HOME}/catkin_ws/src && ln -s ${HOME}/%s ${HOME}/catkin_ws/src/.

RUN cd ${HOME}/catkin_ws \\
 && apt-get -o Acquire::ForceIPv4=true update \\
 && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep update && rosdep install --as-root apt:false --from-paths src --ignore-src -r -y" \\
 && apt-get clean \\
 && rm -rf /var/lib/apt/lists/* \\
 && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && %s"

RUN echo "source ~/catkin_ws/devel/setup.bash" >> ${HOME}/.bashrc
"""

build_string = {'catkin_make': 'catkin_make', 'catkin_build': 'catkin build'}

def write(DOCKER_FILE, name, build_method):
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS % (name, build_string[build_method]))
    return

