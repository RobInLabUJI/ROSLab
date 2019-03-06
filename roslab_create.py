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
        name = yaml_file['name']

        if 'ubuntu' in base.keys():
            import components.ubuntu
            ubuntu = components.ubuntu.canonic_version(base['ubuntu'])
            if 'opengl' in base.keys():
                opengl = base['opengl']
            else:
                opengl = None
            components.ubuntu.write(DOCKER_FILE, ubuntu, opengl)
            #components.ubuntu.select_fastest_mirror(DOCKER_FILE)
            #components.ubuntu.select_country_mirror(DOCKER_FILE, "es")

        import components.jupyterlab
        components.jupyterlab.write(DOCKER_FILE)

        if 'cuda' in base.keys():
            import components.cuda
            version = base['cuda']
            components.cuda.write(DOCKER_FILE, version, ubuntu)

        if 'cudnn' in  base.keys():
            if not 'cuda' in  base.keys():
                print('cudnn: cuda must be specified for using cudnn')
                sys.exit(1)
            import components.cudnn
            version = base['cudnn']
            components.cudnn.write(DOCKER_FILE, version, base['cuda'], ubuntu)

        if 'ros' in base.keys():
            import components.ros
            version = base['ros']
            components.ros.write(DOCKER_FILE, version)
            components.ros.entry_point()

        if 'cmake_update' in base.keys():
            import components.cmake_update
            version = base['cmake_update']
            components.cmake_update.write(DOCKER_FILE, version)

        if 'repo' in yaml_file.keys():
            import components.repo
            repo_list = yaml_file['repo']
            components.repo.write(DOCKER_FILE, repo_list)

        if 'apt' in yaml_file.keys():
            import components.apt
            package_list = yaml_file['apt']
            components.apt.write(DOCKER_FILE, package_list)

        if 'pip3' in yaml_file.keys():
            import components.pip3
            package_list = yaml_file['pip3']
            components.pip3.write(DOCKER_FILE, package_list)

        if 'pip' in yaml_file.keys():
            import components.pip
            package_list = yaml_file['pip']
            components.pip.write(DOCKER_FILE, package_list)

        if 'source' in yaml_file.keys():
            import components.source
            package_list = yaml_file['source']
            components.source.write(DOCKER_FILE, package_list)

        import components.copy
        components.copy.write(DOCKER_FILE, name)

        if 'build' in yaml_file.keys():
            build_method = yaml_file['build']
            if build_method == 'catkin_make' or build_method == 'catkin_build':
                import components.catkin
                components.catkin.write(DOCKER_FILE, name, build_method)
            elif build_method == 'cmake':
                if 'cmake_options' in yaml_file.keys():
                    cmake_options = yaml_file['cmake_options']
                else:
                    cmake_options = ''
                import components.cmake
                components.cmake.write(DOCKER_FILE, name, cmake_options)
            else:
                print('Unknown build method: %s, supported methods: %s' % (build_method, ['catkin_make', 'catkin_build', 'cmake']))

        if 'custom' in yaml_file.keys():
            import components.custom
            custom_commands = yaml_file['custom']
            components.custom.write(DOCKER_FILE, custom_commands)

        import components.tail
        components.tail.write(DOCKER_FILE, name)

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
docker run --rm %s -p 8888:8888 %s"""

RUN_SCRIPT_NVIDIA = """#!/bin/sh
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
    %s -p 8888:8888 %s"""

def write_run_script(yaml_file):
    vol_string = ""
    if 'volume' in yaml_file.keys():
        for v in yaml_file['volume']:
            vol_string += '--volume="%s:%s:%s" ' % (v['host_path'], v['container_path'], v['options'])
    with open(RUN_FILE, "w") as scriptfile:
        if 'opengl' in yaml_file['base'].keys() or 'cuda' in yaml_file['base'].keys():
            scriptfile.write(RUN_SCRIPT_NVIDIA % (vol_string, yaml_file['name']))
        else:
            scriptfile.write(RUN_SCRIPT % (vol_string, yaml_file['name']))
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

    
if __name__ == "__main__":
    main()
