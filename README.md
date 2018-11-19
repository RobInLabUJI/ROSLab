# ROSLab

## JupyterLab for ROS

ROSLab is a [JupyterLab](https://jupyterlab.readthedocs.io/en/stable/)
environment for your [ROS](http://www.ros.org/) repository. 
It automatically creates a dockerfile
from a configuration YAML file (`roslab.yaml`) with the following information:

- name: the name of the docker image
- distro: `kinetic` | `lunar` | `melodic`
- build: `catkin_make` | `catkin_build`
- runtime (optional): `nvidia` (for 3D acceleration)
- packages (optional): a list of debian packages to be installed in the docker image
- python-packages (optional): a list of pip packages to be installed in the docker image
- source (optional): a list of source packages to be downloaded, compiled, and installed in the docker image
    - name: name of the package
    - repo: git repository
    - depends (optional): list of debian packages with dependencies for this package
    - build: `cmake` | `catkin_make` 
- volume (optional): list of directories to be mounted in the docker image
    - host_path: full path in the host
    - container_path: full path in the container
    - options: `ro` | `rw`
    
## Prerequisites

[Docker](https://www.docker.com/)

## Usage

```
$ docker run --rm -v <REPOSITORY_FOLDER>:/project:rw roslab/create
```

The command will read the file `roslab.yaml` from your repository folder,
create the `Dockerfile` in the same folder,
and convert the markdown file `README.md` into a notebook file named
`README.ipynb`.

It will also create two script files `docker_build.sh` and `docker_run.sh`
for building and running the docker image respectively.

After running the image, connect to JupyterLab by opening this URL 
in your browser: `http://localhost:8888/lab/tree/README.ipynb`

## Examples

#### [Minimalistic example](https://github.com/ICRA-2018/raspimouse_ros_2/blob/master/roslab.yaml)
```
name: raspimouse-ros
distro: kinetic
build: catkin_make
```

#### [Typical example with Debian packages](https://github.com/ICRA-2018/nanomap_ros/blob/master/roslab.yaml)
```
name: nanomap_ros
distro: kinetic
build: catkin_make

packages:
  - libeigen3-dev
  - ros-kinetic-cv-bridge
  - ros-kinetic-image-transport
  - liborocos-kdl-dev
  - ros-kinetic-tf2-sensor-msgs
```

#### [Example with volume and source package](https://github.com/ICRA-2018/VINS-Mono/blob/master/roslab.yaml)
```
name: vins-mono
distro: kinetic
build: catkin_make
runtime: nvidia

volume:
  - host_path: /Data/EuRoC_MAV_Dataset
    container_path: /EuRoC_MAV_Dataset
    options: ro

packages:
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
