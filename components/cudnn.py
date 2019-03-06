import os, sys

versions = ['5-runtime', '5-devel', '6-runtime', '6-devel', '7-runtime', '7-devel']

DOCKER_CUDNN_HEADER = """
###################################### CUDNN ###################################
"""

DOCKER_RUNTIME_CONTENTS = {}
DOCKER_RUNTIME_CONTENTS['18.04'] = {}
DOCKER_RUNTIME_CONTENTS['16.04'] = {}
DOCKER_RUNTIME_CONTENTS['18.04']['10.0'] = {}
DOCKER_RUNTIME_CONTENTS['18.04']['9.2'] = {}
DOCKER_RUNTIME_CONTENTS['16.04']['10.0'] = {}
DOCKER_RUNTIME_CONTENTS['16.04']['9.0'] = {}
DOCKER_RUNTIME_CONTENTS['16.04']['8.0'] = {}

DOCKER_RUNTIME_CONTENTS['18.04']['10.0']['7'] = """
ENV CUDNN_VERSION 7.4.2.24

LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \\
            libcudnn7=$CUDNN_VERSION-1+cuda10.0 && \\
    apt-mark hold libcudnn7 && \\
    rm -rf /var/lib/apt/lists/*
"""

DOCKER_RUNTIME_CONTENTS['18.04']['9.2']['7'] = """
ENV CUDNN_VERSION 7.4.1.5
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \\
            libcudnn7=$CUDNN_VERSION-1+cuda9.2 && \\
    apt-mark hold libcudnn7 && \\
    rm -rf /var/lib/apt/lists/*
"""

DOCKER_RUNTIME_CONTENTS['16.04']['10.0']['7'] = """
ENV CUDNN_VERSION 7.4.2.24
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \\
            libcudnn7=$CUDNN_VERSION-1+cuda10.0 && \\
    apt-mark hold libcudnn7 && \\
    rm -rf /var/lib/apt/lists/*
"""

DOCKER_RUNTIME_CONTENTS['16.04']['9.0']['7'] = """
RUN apt-get update && apt-get install -y --no-install-recommends \\
            libcudnn7=$CUDNN_VERSION-1+cuda9.0 && \\
    apt-mark hold libcudnn7 && \\
    rm -rf /var/lib/apt/lists/*
"""

DOCKER_RUNTIME_CONTENTS['16.04']['8.0']['7'] = """
RUN echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list

ENV CUDNN_VERSION 7.2.1.38
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \\
            libcudnn7=$CUDNN_VERSION-1+cuda8.0 && \\
    apt-mark hold libcudnn7 && \\
    rm -rf /var/lib/apt/lists/*
"""

DOCKER_RUNTIME_CONTENTS['16.04']['8.0']['6'] = """
RUN echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list

ENV CUDNN_VERSION 6.0.21
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \
            libcudnn6=$CUDNN_VERSION-1+cuda8.0 && \
    rm -rf /var/lib/apt/lists/*
"""

DOCKER_RUNTIME_CONTENTS['16.04']['8.0']['5'] = """
RUN echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list

ENV CUDNN_VERSION 5.1.10
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \\
            libcudnn5=$CUDNN_VERSION-1+cuda8.0 && \\
    rm -rf /var/lib/apt/lists/*
"""

DOCKER_DEVEL_CONTENTS = {}
DOCKER_DEVEL_CONTENTS['18.04'] = {}
DOCKER_DEVEL_CONTENTS['16.04'] = {}
DOCKER_DEVEL_CONTENTS['18.04']['10.0'] = {}
DOCKER_DEVEL_CONTENTS['18.04']['9.2'] = {}
DOCKER_DEVEL_CONTENTS['16.04']['10.0'] = {}
DOCKER_DEVEL_CONTENTS['16.04']['9.0'] = {}
DOCKER_DEVEL_CONTENTS['16.04']['8.0'] = {}

DOCKER_DEVEL_CONTENTS['18.04']['10.0']['7'] = """
ENV CUDNN_VERSION 7.4.2.24

LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \\
            libcudnn7=$CUDNN_VERSION-1+cuda10.0 \\
            libcudnn7-dev=$CUDNN_VERSION-1+cuda10.0 && \\
    apt-mark hold libcudnn7 && \\
    rm -rf /var/lib/apt/lists/*
"""

DOCKER_DEVEL_CONTENTS['18.04']['9.2']['7'] = """
ENV CUDNN_VERSION 7.4.1.5
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \\
            libcudnn7=$CUDNN_VERSION-1+cuda9.2 \\
            libcudnn7-dev=$CUDNN_VERSION-1+cuda9.2 && \\
    apt-mark hold libcudnn7 && \\
    rm -rf /var/lib/apt/lists/*
"""

DOCKER_DEVEL_CONTENTS['16.04']['10.0']['7'] = """
ENV CUDNN_VERSION 7.4.1.5
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \\
            libcudnn7=$CUDNN_VERSION-1+cuda10.0 \\
            libcudnn7-dev=$CUDNN_VERSION-1+cuda10.0 && \\
    apt-mark hold libcudnn7 && \\
    rm -rf /var/lib/apt/lists/*
"""

DOCKER_DEVEL_CONTENTS['16.04']['9.0']['7'] = """
ENV CUDNN_VERSION 7.4.2.24
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \\
            libcudnn7=$CUDNN_VERSION-1+cuda9.0 \\
            libcudnn7-dev=$CUDNN_VERSION-1+cuda9.0 && \\
    apt-mark hold libcudnn7 && \\
    rm -rf /var/lib/apt/lists/*
"""

DOCKER_DEVEL_CONTENTS['16.04']['8.0']['7'] = """
RUN echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list

ENV CUDNN_VERSION 7.2.1.38
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \\
            libcudnn7=$CUDNN_VERSION-1+cuda8.0 \\
            libcudnn7-dev=$CUDNN_VERSION-1+cuda8.0 && \\
    apt-mark hold libcudnn7 && \\
    rm -rf /var/lib/apt/lists/*
"""

DOCKER_DEVEL_CONTENTS['16.04']['8.0']['6'] = """
RUN echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list

ENV CUDNN_VERSION 6.0.21
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \
            libcudnn6=$CUDNN_VERSION-1+cuda8.0 \
            libcudnn6-dev=$CUDNN_VERSION-1+cuda8.0 && \
    rm -rf /var/lib/apt/lists/*
"""

DOCKER_DEVEL_CONTENTS['16.04']['8.0']['5'] = """
RUN echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list

ENV CUDNN_VERSION 5.1.10
LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \\
            libcudnn5=$CUDNN_VERSION-1+cuda8.0 \\
            libcudnn5-dev=$CUDNN_VERSION-1+cuda8.0 && \\
    rm -rf /var/lib/apt/lists/*
"""

def write(DOCKER_FILE, version, cuda, ubuntu):
    if version in versions:
        with open(DOCKER_FILE, "a") as dockerfile:
            dockerfile.write(DOCKER_CUDNN_HEADER)
            cuda = cuda.split('-')[0]
            cudnn = version.split('-')[0]
            try:
                if 'runtime' in version:
                    dockerfile.write(DOCKER_RUNTIME_CONTENTS[ubuntu][cuda][cudnn])
                elif 'devel' in version:
                    dockerfile.write(DOCKER_DEVEL_CONTENTS[ubuntu][cuda][cudnn])
            except KeyError as e:
                print("CUDNN version %s not supported in Ubuntu %s with CUDA %s" % (cudnn, ubuntu, cuda) )
                sys.exit(1)
        return
    else:
        print("cudnn: version %s not supported. Options: %s" % (version, versions))
        sys.exit(1)

