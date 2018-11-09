#!/usr/bin/env python
import os, subprocess, sys, yaml
import os.path
    
distros = ['kinetic', 'lunar', 'melodic']

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

def source_commands(list_pack):
    s = ''
    for p in list_pack:
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
            s += " && rm -fr /" + p['name'] + "\n"
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
    
    global_head = "FROM roslab/roslab:" + distro + """

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
 && apt-get update \\
 && /bin/bash -c "source /opt/ros/DISTRO/setup.bash && rosdep update && rosdep install --as-root apt:false --from-paths src --ignore-src -r -y" \\
 && apt-get clean \\
 && rm -rf /var/lib/apt/lists/* \\
 && /bin/bash -c "source /opt/ros/DISTRO/setup.bash && catkin build"

RUN echo "source ~/catkin_ws/devel/setup.bash" >> ${HOME}/.bashrc\n
"""
        mid_section = mid_section.replace('DISTRO', distro)
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
            dockerfile.write(source_commands(yl['source']))
        dockerfile.write(mid_section)
        dockerfile.write(global_tail)
    
    with open("docker_build.sh", "w") as scriptfile:
        scriptfile.write("#!/bin/sh\ndocker build -t %s -f Dockerfile ." % yl['name'])
    os.chmod("docker_build.sh", 0o755)
    
    with open("docker_run.sh", "w") as scriptfile:
        scriptfile.write("#!/bin/sh\ndocker run --rm -p 8888:8888 %s" % yl['name'])
    os.chmod("docker_run.sh", 0o755)
    
    if os.path.isfile("README.md"):
        os.system("notedown README.md > README.ipynb")
    
if __name__ == "__main__":
    main()