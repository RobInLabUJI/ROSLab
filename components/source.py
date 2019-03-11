import sys

DOCKER_SOURCE_HEADER = """
################################### SOURCE #####################################
"""

DEPENDENCIES = """
RUN apt-get -o Acquire::ForceIPv4=true update \\
 && apt-get -o Acquire::ForceIPv4=true install -yq --no-install-recommends \\
%s && apt-get clean \\
 && rm -rf /var/lib/apt/lists/*
"""

CLONE = """
RUN git clone %s /%s \\
 && cd /%s \\"""

UNZIP = """
RUN mkdir /%s && cd /%s \\
 && wget %s \\
 && unzip %s && rm %s \\
 && cd %s-%s \\"""

BUILD_CMAKE = """
 && mkdir build \\
 && cd build \\
 && cmake %s %s \\
 && %s \\"""

BUILD_CATKIN_MAKE = """
 && mkdir -p ${HOME}/catkin_ws/src \\
 && cp -R /%s ${HOME}/catkin_ws/src/. \\
 && cd ${HOME}/catkin_ws \\
 && apt-get -o Acquire::ForceIPv4=true update \\
 && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep update && rosdep install --as-root apt:false --from-paths src --ignore-src -r -y" \\
 && apt-get clean \\
 && rm -rf /var/lib/apt/lists/* \\
 && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin_make" \\"""

BUILD_CATKIN_BUILD = """
 && mkdir -p ${HOME}/catkin_ws/src \\
 && cp -R /%s ${HOME}/catkin_ws/src/. \\
 && cd ${HOME}/catkin_ws \\
 && apt-get -o Acquire::ForceIPv4=true update \\
 && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && rosdep update && rosdep install --as-root apt:false --from-paths src --ignore-src -r -y" \\
 && apt-get clean \\
 && rm -rf /var/lib/apt/lists/* \\
 && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build" \\"""

KEEP_SOURCE = """
 && cp -R /%s ${HOME}/. \\"""

DELETE = """
 && rm -fr /%s
"""

def write(DOCKER_FILE, package_list):
    s = DOCKER_SOURCE_HEADER
    for p in package_list:
        if 'depends' in p.keys() and p['depends']:
            pstr = ''
            for apt_pack in p['depends']:
                pstr += '    ' + apt_pack + ' \\\n'
            s += DEPENDENCIES % pstr
        if 'repo' in p.keys():
            s += CLONE % (p['repo'], p['name'], p['name'])
        elif 'zip' in p.keys():
            url = p['zip']
            archive = url.split('/')[-1]
            version = '.'.join(archive.split('.')[:-1])
            s += UNZIP % (p['name'], p['name'], url, archive, archive, p['name'], version)
        else:
            print('source: cannot get source for package %s' % p['name'])
            sys.exit(1)
        if p['build'] == 'cmake':
            if 'cmake_options' in p.keys():
                cmake_options = p['cmake_options']
            else:
                cmake_options = ''
            if 'cmake_folder' in p.keys():
                cmake_folder = p['cmake_folder']
            else:
                cmake_folder = '../'
            if 'make_command' in p.keys():
                make_command = p['make_command']
            else:
                make_command = 'make -j4 install'
            s += BUILD_CMAKE % (cmake_options, cmake_folder, make_command)
            if 'keep_source' in p.keys() and p['keep_source'] == True:
                s += KEEP_SOURCE % p['name']
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

