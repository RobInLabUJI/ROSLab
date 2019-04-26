# ROSLab
[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/RobInLabUJI/ROSLab/web?filepath=CreateDockerFile.ipynb)

## *RO*botics *S*oftware with Jupyter*Lab*

ROSLab is a [JupyterLab](https://jupyterlab.readthedocs.io/en/stable/)
environment for a source code repository with robotics software. 
It automatically creates a Dockerfile
from the configuration data entered by the user in a simple web form:

![UI form](ui_form.png "UI form")

First, you must specify a name for your Docker image, then select an Ubuntu version (16.04 is the default).

A number of options can be selected for the platform:
* X11
* OpenGL
* CUDA
* cuDNN
* ROS
* Build method

In the middle column of the form, you can specify the package dependencies of your software:
* APT packages
* Python 2 packages
* Python 3 packages

You can also add any custom commands that will be executed at the end of the process, as well as the path
to a local MATLAB install (for this to work, the guest and host OS must be compatible).

In the rightmost column, you can specify other dependencies on source packages; for each one, you must indicate:
* a name
* the URL of the repository
* the dependencies
* the building method

When the form is complete, you must press the `Proceed` button and the Dockerfile will be generated, which can be downloaded to your local folder. The commands for building and running the Docker image will be displayed too.

## Prerequisites

[Docker](https://www.docker.com/)

[nvidia-docker 2.0](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0)) For running an image with `OpenGL` and/or `CUDA`.

## Usage

Use it in [Binder](https://mybinder.org/v2/gh/RobInLabUJI/ROSLab/web?filepath=CreateDockerFile.ipynb), or in your local host with Docker:
1. Run in the command line:
```
docker run --rm -p 8888:8888 roslab/web:latest
```
2. Copy and paste the displayed link in your browser:
```
http://127.0.0.1:888?token=...
```
3. Open the notebook `CreateDockerFile.ipynb`
