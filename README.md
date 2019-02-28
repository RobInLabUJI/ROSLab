# ROSLab

## *RO*botics *S*oftware with Jupyter*Lab*

ROSLab is a [JupyterLab](https://jupyterlab.readthedocs.io/en/stable/)
environment for a source code repository with robotics software. 
It automatically creates a dockerfile
from a configuration YAML file (`roslab.yaml`) with the following information:

- name: the name of the docker image
- base:
  - ubuntu: `16.04` | `18.04`
  - opengl: `runtime` | `devel`
  - cuda: `8.0-runtime` | `8.0-devel` | `9.0-runtime` | `9.0-devel` | `10.0-runtime` | `10.0-devel`
  - cudnn: `5-runtime` | `5-devel` | `6-runtime` | `6-devel` | `7-runtime` | `7-devel`
  - ros: `kinetic-ros-core` | `kinetic-ros-base` | `kinetic-robot` | `kinetic-perception` | `kinetic-desktop` | `kinetic-desktop-full` | `melodic-ros-core` | `melodic-ros-base` | `melodic-robot` | `melodic-perception` | `melodic-desktop` | `melodic-desktop-full`
  
- build: `cmake` | `catkin_make` | `catkin_build`
- apt: a list of apt packages to install
- pip: a list of Python2 packages to install
- pip3: a list of Python3 packages to install
- source: a list of source code repositories to download, compile, and install
    - name: name of the repository
    - repo: address of the git repository
    - depends (optional): list of apt packages with dependencies for this repository
    - build: `cmake` | `catkin_make` | `catkin_build`
- volume: list of directories to be mounted in the docker image
    - host_path: full path in the host
    - container_path: full path in the container
    - options: `ro` | `rw`
 - custom: list of additional shell commands
    
## Prerequisites

[Docker](https://www.docker.com/)

[nvidia-docker 2.0](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0)) For running an image with `OpenGL` and/or `CUDA`.

## Usage

```
$ docker run --rm -v <REPOSITORY_FOLDER>:/project:rw roslab/create
```

The command will read the file `roslab.yaml` from your repository folder,
create the `roslab.dockerfile` in the same folder,
and convert the markdown file `README.md` into a notebook file named
`README.ipynb`.

It will also create two script files `roslab_build.sh` and `roslab_run.sh`
for building and running the docker image respectively.

After running the image, connect to JupyterLab by opening this URL 
in your browser: `http://localhost:8888/lab/tree/README.ipynb`

## Examples

#### [Minimalistic example](https://github.com/ICRA-2018/raspimouse_ros_2/blob/master/roslab.yaml)
```
name: raspimouse-ros
base: 
  ubuntu: 16.04
  ros: kinetic-ros-base
build: catkin_make
```

#### [Typical example with Debian packages](https://github.com/ICRA-2018/nanomap_ros/blob/master/roslab.yaml)
```
name: nanomap_ros
base:
  ubuntu: 16.04
  ros: kinetic-ros-core
build: catkin_make
apt:
  - libeigen3-dev
  - ros-kinetic-cv-bridge
  - ros-kinetic-image-transport
  - liborocos-kdl-dev
  - ros-kinetic-tf2-sensor-msgs
```

#### [Example with volume and source package](https://github.com/ICRA-2018/VINS-Mono/blob/master/roslab.yaml)
```
name: vins-mono
base:
  ubuntu: 16.04
  ros: kinetic-desktop-full
  opengl: runtime
build: catkin_make

volume:
  - host_path: /DATASETS/EuRoC_MAV_Dataset
    container_path: /EuRoC_MAV_Dataset
    options: ro

apt:
  - ros-kinetic-cv-bridge
  - ros-kinetic-tf
  - ros-kinetic-message-filters
  - ros-kinetic-image-transport

source:
  - name: ceres-solver
    repo: https://github.com/ceres-solver/ceres-solver.git
    depends:
      - libgoogle-glog-dev
      - libatlas-base-dev
      - libeigen3-dev
      - libsuitesparse-dev
    build: cmake
```

## Try ROSLab

You can try the JupyterLab environment in a fresh ROS install:

1. Install docker

2. Run a ROSLab image with:
```
$ docker run --rm -p 8888:8888 roslab/roslab:melodic
```

3. Open http://localhost:8888 in your browser (Windows users: you may need to replace `localhost` with `192.168.99.100` or any IP address assigned to your docker machine)

Or try it in Binder:

[![Binder](https://mybinder.org/badge.svg)](https://mybinder.org/v2/gh/RobInLabUJI/ROSLab-demo/master?urlpath=lab)
