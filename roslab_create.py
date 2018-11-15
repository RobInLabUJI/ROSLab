#!/usr/bin/env python
import os, subprocess, sys, yaml
import os.path
    
distros = ['kinetic', 'lunar', 'melodic']

build_methods = {'catkin_make': 'catkin_make', 'catkin_build': 'catkin build'}

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

def source_commands(list_pack, distro):
    s = ''
    for p in list_pack:
        if 'depends' in p.keys() and p['depends']:
            s += """\nRUN apt-get update \\
  && apt-get install -yq --no-install-recommends \\"""
            for d in p['depends']:
                s += '\n    ' + d + ' \\'
            s += """
 && apt-get clean \\
 && rm -rf /var/lib/apt/lists/*
"""
        s += "\nRUN git clone " + p['repo'] + " /" + p['name'] + " \\\n && cd /" + p['name'] + " \\\n"
        if p['build'] == 'cmake':
            s += """ && mkdir build \\
 && cd build \\
 && cmake ../ \\
 && make -j4 install \\
"""
        elif p['build'] == 'catkin_build':
            s += " && mkdir -p ${HOME}/catkin_ws/src \\\n && cp -R /" + p['name'] + " ${HOME}/catkin_ws/src/. \\" + """
 && cd ${HOME}/catkin_ws \\
 && apt-get update \\
 && /bin/bash -c "source /opt/ros/DISTRO/setup.bash && rosdep update && rosdep install --as-root apt:false --from-paths src --ignore-src -r -y" \\
 && apt-get clean \\
 && rm -rf /var/lib/apt/lists/* \\
 && /bin/bash -c "source /opt/ros/DISTRO/setup.bash && catkin build" \\
"""

        elif p['build'] == 'catkin_make':
            s += " && mkdir -p ${HOME}/catkin_ws/src \\\n && cp -R /" + p['name'] + " ${HOME}/catkin_ws/src/. \\" + """
 && cd ${HOME}/catkin_ws \\
 && apt-get update \\
 && /bin/bash -c "source /opt/ros/DISTRO/setup.bash && rosdep update && rosdep install --as-root apt:false --from-paths src --ignore-src -r -y" \\
 && apt-get clean \\
 && rm -rf /var/lib/apt/lists/* \\
 && /bin/bash -c "source /opt/ros/DISTRO/setup.bash && catkin_make" \\
"""

        else:
            print("Warning: build method '%s' not defined for package '%s'" % (p['build'], p['name']))
        s += " && rm -fr /" + p['name'] + "\n"
        s = s.replace("DISTRO", distro)
    return s
    
def main():
    if len(sys.argv) != 2:
        print("Usage: %s <folder>" % sys.argv[0])
        sys.exit(1)
    
    os.chdir(sys.argv[1])
    try:
        with open(sys.argv[1] + '/roslab.yaml', 'r') as stream:
            try:
                yl = yaml.load(stream)
            except yaml.YAMLError as e:
                print(e)
                sys.exit(1)
    except FileNotFoundError:
        print("File 'roslab.yaml' not found in folder '%s'" % sys.argv[1])
        sys.exit(1)
    distro = yl['distro']
    if not distro in distros:
        print('Distro %s not supported.' % distro)
        sys.exit(1)
    
    base_image = distro
    if 'runtime' in yl.keys():
        runtime = yl['runtime']
        if runtime == 'nvidia':
            base_image += '-' + runtime
        else:
            print("Warning: unknown runtime '%s', ignoring." % runtime)
            runtime = None
    else:
        runtime = None

    global_head = "FROM roslab/roslab:" + base_image + """

USER root\n
"""
    apt_head = """RUN apt-get update \\
 && apt-get install -yq --no-install-recommends \\
"""

    apt_tail = """ && apt-get clean \\
 && rm -rf /var/lib/apt/lists/*
"""
    #if os.path.isfile("package.xml"):
    if getDirectoryList("."):
        mid_section = "\nRUN mkdir -p ${HOME}/catkin_ws/src/" + yl['name'] + \
            "\nCOPY . ${HOME}/catkin_ws/src/" + yl['name'] + "/." + """
RUN cd ${HOME}/catkin_ws \\
 && mv src/""" + yl['name'] + """/README.ipynb .. \\
 && apt-get update \\
 && /bin/bash -c "source /opt/ros/DISTRO/setup.bash && rosdep update && rosdep install --as-root apt:false --from-paths src --ignore-src -r -y" \\
 && apt-get clean \\
 && rm -rf /var/lib/apt/lists/* \\
 && /bin/bash -c "source /opt/ros/DISTRO/setup.bash && BUILD"

RUN echo "source ~/catkin_ws/devel/setup.bash" >> ${HOME}/.bashrc\n
"""
        mid_section = mid_section.replace('DISTRO', distro)
        mid_section = mid_section.replace('BUILD', build_methods[yl['build']])
    else:
        mid_section = """
COPY . ${HOME}\n
"""

    global_tail = """RUN chown -R ${NB_UID} ${HOME}

USER ${NB_USER}
WORKDIR ${HOME}
"""

    with open("Dockerfile", "w") as dockerfile:
        dockerfile.write(global_head)
        if 'packages' in yl.keys():
            dockerfile.write(apt_head)
            for p in yl['packages']:
                dockerfile.write("    %s \\\n" % p)
            dockerfile.write(apt_tail)
        if 'source' in yl.keys():
            dockerfile.write(source_commands(yl['source'], distro))
        dockerfile.write(mid_section)
        dockerfile.write(global_tail)
    
    with open("docker_build.sh", "w") as scriptfile:
        scriptfile.write("#!/bin/sh\ndocker build -t %s -f Dockerfile ." % yl['name'])
    os.chmod("docker_build.sh", 0o755)
    
    vol_string = ""
    if 'volume' in yl.keys():
        for v in yl['volume']:
            vol_string += '--volume="%s:%s:%s" ' % (v['host_path'], v['container_path'], v['options'])

    with open("docker_run.sh", "w") as scriptfile:
        if not runtime:
            scriptfile.write("#!/bin/sh\ndocker run --rm -p 8888:8888 " + vol_string + " %s" % yl['name'])
        elif runtime=='nvidia':
            scriptfile.write("""#!/bin/sh
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker run --rm \\
    --env="DISPLAY" \\
    --env="QT_X11_NO_MITSHM=1" \\
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \\
    -env="XAUTHORITY=$XAUTH" \\
    --volume="$XAUTH:$XAUTH" \\
    --runtime=nvidia \\
""")
            scriptfile.write("    -p 8888:8888 \\\n    " + vol_string + "\\\n    %s" % yl['name'])
    os.chmod("docker_run.sh", 0o755)
    
    with open(".dockerignore", "w") as ignorefile:
        ignorefile.write(".dockerignore\nDockerfile\nroslab.yaml\ndocker_build.sh\ndocker_run.sh\n")

    if os.path.isfile("README.md"):
        os.system("notedown README.md > README.ipynb")
    
if __name__ == "__main__":
    main()
