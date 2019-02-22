DOCKER_SOURCE_HEADER = """
################################### SOURCE #####################################
"""

DEPENDENCIES = """
RUN apt-get update \\
 && apt-get install -yq --no-install-recommends \\
%s && apt-get clean \\
 && rm -rf /var/lib/apt/lists/*
"""

CLONE = """
RUN git clone %s /%s \\
 && cd /%s \\"""

BUILD_CMAKE = """
 && mkdir build \\
 && cd build \\
 && cmake ../ \\
 && make -j4 install \\"""

BUILD_CATKIN_MAKE = """
 && mkdir -p ${HOME}/catkin_ws/src \\
 && cp -R /%s ${HOME}/catkin_ws/src/. \\
 && cd ${HOME}/catkin_ws \\
 && apt-get update \\
 && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep update && rosdep install --as-root apt:false --from-paths src --ignore-src -r -y" \\
 && apt-get clean \\
 && rm -rf /var/lib/apt/lists/* \\
 && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make" \\"""

BUILD_CATKIN_BUILD = """
 && mkdir -p ${HOME}/catkin_ws/src \\
 && cp -R /%s ${HOME}/catkin_ws/src/. \\
 && cd ${HOME}/catkin_ws \\
 && apt-get update \\
 && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep update && rosdep install --as-root apt:false --from-paths src --ignore-src -r -y" \\
 && apt-get clean \\
 && rm -rf /var/lib/apt/lists/* \\
 && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build" \\"""

DELETE = """
 && rm -fr /%s
"""

def write(DOCKER_FILE, package_list):
    s = DOCKER_SOURCE_HEADER
    for p in package_list:
        if 'depends' in p.keys() and p['depends']:
            pstr = ''
            for p in p['depends']:
                pstr += '    ' + p + ' \\\n'
            s += DEPENDENCIES % pstr
        s += CLONE % (p['repo'], p['name'], p['name'])
        if p['build'] == 'cmake':
            s += BUILD_CMAKE
        elif p['build'] == 'catkin_build':
            s += BUILD_CATKIN_BUILD % p['name']
        elif p['build'] == 'catkin_make':
            s += BUILD_CATKIN_MAKE % p['name']
        else:
            print("Warning: build method '%s' not defined for package '%s'" % (p['build'], p['name']))
        s += DELETE % p['name']

    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(s)
    return

