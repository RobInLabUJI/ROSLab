# ROSLab

## JupyterLab for ROS

ROSLab is a [JupyterLab](https://jupyterlab.readthedocs.io/en/stable/)
environment for your [ROS](http://www.ros.org/) repository. 
It automatically creates a dockerfile
from a configuration YAML file with the following information:

- name: the name of the docker image
- repo: the URL address of the repository
- distro: the ROS distribution
- packages: a list of debian packages to be installed in the docker image

## Prerequisites

[Docker](https://www.docker.com/)

## Usage

```
$ docker run --rm -v <REPOSITORY_FOLDER>:/project:rw roslab/create
```

The command will create the `Dockerfile` in your repository folder,
and convert the markdown file `README.md` into a notebook file named
`README.ipynb`.

It will also create two script files `docker_build.sh` and `docker_run.sh`
for building and running the docker image respectively.

After running the image, connect to JupyterLab by opening this URL 
in your browser: `http://localhost:8888`

## Try ROSLab

You can try the JupyterLab environment in a fresh ROS install:

1. Install docker

2. Run a ROSLab image with:
```
$ docker run --rm -p 8888:8888 roslab/roslab:melodic
```

3. Open http://localhost:8888 in your browser

Or try it in Binder:

[![Binder](https://mybinder.org/badge.svg)](https://mybinder.org/v2/gh/RobInLabUJI/ROSLab-demo/master?urlpath=lab)
