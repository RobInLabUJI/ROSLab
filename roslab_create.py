#!/usr/bin/env python
import os, sys, yaml
import os.path

PROJECT_DIR  = '/project'
PROJECT_FILE = 'roslab.yaml'
DOCKER_FILE  = 'roslab.dockerfile'
BUILD_FILE   = 'roslab_build.sh'
RUN_FILE     = 'roslab_run.sh'

def main():
    os.chdir(PROJECT_DIR)
    yaml_file = read_yaml_file()
    write_docker_file(yaml_file)
    write_build_script(yaml_file)
    write_run_script(yaml_file)
    write_docker_ignore()
    write_readme_notebook()

def read_yaml_file():
    try:
        with open(PROJECT_FILE, 'r') as stream:
            try:
                yaml_file = yaml.load(stream)
            except yaml.YAMLError as e:
                print(e)
                sys.exit(1)
    except FileNotFoundError:
        print("File '%s' not found in folder '%s'" % (PROJECT_FILE, PROJECT_DIR))
        sys.exit(1)
    return yaml_file

def write_docker_file(yaml_file):
    try:
        base = yaml_file['base']
        if 'ubuntu' in base.keys():
            import components.ubuntu
            version = base['ubuntu']
            components.ubuntu.write(DOCKER_FILE, version)
        import components.jupyterlab
        components.jupyterlab.write(DOCKER_FILE)
        if 'ros' in base.keys():
            import components.ros
            version = base['ros']
            components.ros.write(DOCKER_FILE, version)
            components.ros.entry_point()
        import components.copy
        components.copy.write(DOCKER_FILE)
        import components.tail
        components.tail.write(DOCKER_FILE)
    except KeyError as e:
        print("Key %s not found in file %s" % (e, PROJECT_FILE) )
        sys.exit(1)

BUILD_SCRIPT = """#!/bin/sh
docker build -f %s -t %s ."""

def write_build_script(yaml_file):
    with open(BUILD_FILE, "w") as scriptfile:
        scriptfile.write(BUILD_SCRIPT % (DOCKER_FILE, yaml_file['name']))
    os.chmod(BUILD_FILE, 0o755)

RUN_SCRIPT   = """#!/bin/sh
docker run --rm -p 8888:8888 %s"""

def write_run_script(yaml_file):
    with open(RUN_FILE, "w") as scriptfile:
        scriptfile.write(RUN_SCRIPT % (yaml_file['name']))
    os.chmod(RUN_FILE, 0o755)

DOCKER_IGNORE_FILE = ".dockerignore"

DOCKER_IGNORE_CONTENTS = """README.md
%s
%s
%s
%s
%s
""" % (DOCKER_IGNORE_FILE, PROJECT_FILE, DOCKER_FILE, BUILD_FILE, RUN_FILE)

def write_docker_ignore():
    with open(DOCKER_IGNORE_FILE, "w") as ignorefile:
        ignorefile.write(DOCKER_IGNORE_CONTENTS)

NOTEBOOK_TAIL = """
 "metadata": {
  "kernelspec": {
   "display_name": "Bash",
   "language": "bash",
   "name": "bash"
  },
  "language_info": {
   "codemirror_mode": "shell",
   "file_extension": ".sh",
   "mimetype": "text/x-sh",
   "name": "bash"
  }
 },
 "nbformat": 4,
 "nbformat_minor": 2
}
"""

def write_readme_notebook():
    if os.path.isfile("README.md"):
        os.system("notedown README.md | head -n -4 > README.ipynb")
        with open("README.ipynb", "a") as myfile:
            myfile.write(NOTEBOOK_TAIL)


################################################################################

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
    
def old_main():
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
        if 'repos' in yl.keys():
            dockerfile.write("""RUN apt-get update \\
 && apt-get install -yq --no-install-recommends \\
    software-properties-common \\
 && apt-get clean \\
 && rm -rf /var/lib/apt/lists/*
""")
            for r in yl['repos']:
                dockerfile.write("RUN apt-add-repository %s\n" % r)
        if 'packages' in yl.keys():
            dockerfile.write(apt_head)
            for p in yl['packages']:
                dockerfile.write("    %s \\\n" % p)
            dockerfile.write(apt_tail)
        if 'python-packages' in yl.keys():
            pip_string = "RUN pip install "
            for p in yl['python-packages']:
                pip_string += p + " "
            pip_string += "\n"
            dockerfile.write(pip_string)
        if 'source' in yl.keys():
            dockerfile.write(source_commands(yl['source'], distro))
        dockerfile.write(mid_section)
        dockerfile.write(global_tail)
        if 'custom' in yl.keys():
            for c in yl['custom']:
                run_string = "RUN " + c + "\n"
                dockerfile.write(run_string)
    
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
    --env="XAUTHORITY=$XAUTH" \\
    --volume="$XAUTH:$XAUTH" \\
    --runtime=nvidia \\
""")
            scriptfile.write("    -p 8888:8888 \\\n    " + vol_string + "\\\n    %s" % yl['name'])
    os.chmod("docker_run.sh", 0o755)
    
    with open(".dockerignore", "w") as ignorefile:
        ignorefile.write(".dockerignore\nDockerfile\nroslab.yaml\ndocker_build.sh\ndocker_run.sh\n")

    if os.path.isfile("README.md"):
        os.system("notedown README.md | head -n -4 > README.ipynb")
        with open("README.ipynb", "a") as myfile:
            myfile.write(""" "metadata": {
  "kernelspec": {
   "display_name": "Bash",
   "language": "bash",
   "name": "bash"
  },
  "language_info": {
   "codemirror_mode": "shell",
   "file_extension": ".sh",
   "mimetype": "text/x-sh",
   "name": "bash"
  }
 },
 "nbformat": 4,
 "nbformat_minor": 2
}
""")
    
if __name__ == "__main__":
    main()
