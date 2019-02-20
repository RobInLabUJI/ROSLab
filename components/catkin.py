DOCKER_CONTENTS = """
#################################### CATKIN ####################################

RUN mkdir -p ${HOME}/catkin_ws/src/%s

COPY . ${HOME}/catkin_ws/src/%s/.

RUN cd ${HOME}/catkin_ws \\
 && apt-get update \\
 && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep update && rosdep install --as-root apt:false --from-paths src --ignore-src -r -y" \\
 && apt-get clean \\
 && rm -rf /var/lib/apt/lists/* \\
 && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && %s"

RUN echo "source ~/catkin_ws/devel/setup.bash" >> ${HOME}/.bashrc
"""

build_string = {'catkin_make': 'catkin_make', 'catkin_build': 'catkin build'}

def getDirectoryList(path):
    directoryList = []

    #return nothing if path is a file
    if os.path.isfile(path):
        return []

    #add dir to directorylist if it contains a package.xml
    if len([f for f in os.listdir(path) if f=='package.xml'])>0:
        directoryList.append(path)

    for d in os.listdir(path):
        new_path = os.path.join(path, d)
        if os.path.isdir(new_path):
            directoryList += getDirectoryList(new_path)

    return directoryList


def write(DOCKER_FILE, name, build_method):
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS % (name, name, build_string[build_method]))
    return

