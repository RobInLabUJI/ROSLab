import os, sys

versions = ['8.0-runtime', '8.0-devel', '9.0-runtime', '9.0-devel', '10.0-runtime', '10.0-devel']

DOCKER_CUDA_HEADER = """
###################################### CUDA ####################################
"""

DOCKER_RUNTIME_CONTENTS = {}
DOCKER_RUNTIME_CONTENTS['18.04'] = {}
DOCKER_RUNTIME_CONTENTS['16.04'] = {}

DOCKER_RUNTIME_CONTENTS['18.04']['10.0'] = """
RUN apt-get update && apt-get install -y --no-install-recommends gnupg2 curl ca-certificates && \\
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub | apt-key add - && \\
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \\
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list && \\
    apt-get purge --autoremove -y curl && \\
    rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 10.0.130

ENV CUDA_PKG_VERSION 10-0=$CUDA_VERSION-1
# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \\
        cuda-cudart-$CUDA_PKG_VERSION \\
        cuda-compat-10-0=410.48-1 && \\
    ln -s cuda-10.0 /usr/local/cuda && \\
    rm -rf /var/lib/apt/lists/*

ENV PATH /usr/local/cuda/bin:${PATH}

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=10.0 brand=tesla,driver>=384,driver<385"

ENV NCCL_VERSION 2.4.2

RUN apt-get update && apt-get install -y --no-install-recommends \\
        cuda-libraries-$CUDA_PKG_VERSION \\
        cuda-nvtx-$CUDA_PKG_VERSION \\
        libnccl2=$NCCL_VERSION-1+cuda10.0 && \\
    apt-mark hold libnccl2 && \\
    rm -rf /var/lib/apt/lists/*
"""

DOCKER_RUNTIME_CONTENTS['16.04']['10.0'] = """
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates apt-transport-https gnupg-curl && \\
    rm -rf /var/lib/apt/lists/* && \\
    NVIDIA_GPGKEY_SUM=d1be581509378368edeec8c1eb2958702feedf3bc3d17011adbf24efacce4ab5 && \\
    NVIDIA_GPGKEY_FPR=ae09fe4bbd223a84b2ccfce3f60f4b3d7fa2af80 && \\
    apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub && \\
    apt-key adv --export --no-emit-version -a $NVIDIA_GPGKEY_FPR | tail -n +5 > cudasign.pub && \\
    echo "$NVIDIA_GPGKEY_SUM  cudasign.pub" | sha256sum -c --strict - && rm cudasign.pub && \\
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \\
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list

ENV CUDA_VERSION 10.0.130

ENV CUDA_PKG_VERSION 10-0=$CUDA_VERSION-1
# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \\
        cuda-cudart-$CUDA_PKG_VERSION \\
        cuda-compat-10-0=410.48-1 && \\
    ln -s cuda-10.0 /usr/local/cuda && \\
    rm -rf /var/lib/apt/lists/*

ENV PATH /usr/local/cuda/bin:${PATH}

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=10.0 brand=tesla,driver>=384,driver<385"

ENV NCCL_VERSION 2.4.2

RUN apt-get update && apt-get install -y --no-install-recommends \\
        cuda-libraries-$CUDA_PKG_VERSION \\
        cuda-nvtx-$CUDA_PKG_VERSION \\
        libnccl2=$NCCL_VERSION-1+cuda10.0 && \\
    apt-mark hold libnccl2 && \\
    rm -rf /var/lib/apt/lists/*
"""

DOCKER_RUNTIME_CONTENTS['16.04']['9.0'] = """
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates apt-transport-https gnupg-curl && \\
    rm -rf /var/lib/apt/lists/* && \\
    NVIDIA_GPGKEY_SUM=d1be581509378368edeec8c1eb2958702feedf3bc3d17011adbf24efacce4ab5 && \\
    NVIDIA_GPGKEY_FPR=ae09fe4bbd223a84b2ccfce3f60f4b3d7fa2af80 && \\
    apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub && \\
    apt-key adv --export --no-emit-version -a $NVIDIA_GPGKEY_FPR | tail -n +5 > cudasign.pub && \\
    echo "$NVIDIA_GPGKEY_SUM  cudasign.pub" | sha256sum -c --strict - && rm cudasign.pub && \\
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \\
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list

ENV CUDA_VERSION 9.0.176

ENV CUDA_PKG_VERSION 9-0=$CUDA_VERSION-1
RUN apt-get update && apt-get install -y --no-install-recommends \\
        cuda-cudart-$CUDA_PKG_VERSION && \\
    ln -s cuda-9.0 /usr/local/cuda && \\
    rm -rf /var/lib/apt/lists/*

# nvidia-docker 1.0
LABEL com.nvidia.volumes.needed="nvidia_driver"
LABEL com.nvidia.cuda.version="${CUDA_VERSION}"

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \\
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=9.0"

ENV NCCL_VERSION 2.4.2

RUN apt-get update && apt-get install -y --no-install-recommends \\
        cuda-libraries-$CUDA_PKG_VERSION \\
        cuda-cublas-9-0=9.0.176.4-1 \\
        libnccl2=$NCCL_VERSION-1+cuda9.0 && \\
    apt-mark hold libnccl2 && \\
    rm -rf /var/lib/apt/lists/*
"""

DOCKER_RUNTIME_CONTENTS['16.04']['8.0'] = """
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates apt-transport-https gnupg-curl && \\
    rm -rf /var/lib/apt/lists/* && \\
    NVIDIA_GPGKEY_SUM=d1be581509378368edeec8c1eb2958702feedf3bc3d17011adbf24efacce4ab5 && \\
    NVIDIA_GPGKEY_FPR=ae09fe4bbd223a84b2ccfce3f60f4b3d7fa2af80 && \\
    apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub && \\
    apt-key adv --export --no-emit-version -a $NVIDIA_GPGKEY_FPR | tail -n +5 > cudasign.pub && \\
    echo "$NVIDIA_GPGKEY_SUM  cudasign.pub" | sha256sum -c --strict - && rm cudasign.pub && \\
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/cuda.list

ENV CUDA_VERSION 8.0.61

ENV CUDA_PKG_VERSION 8-0=$CUDA_VERSION-1
RUN apt-get update && apt-get install -y --no-install-recommends \\
        cuda-nvrtc-$CUDA_PKG_VERSION \\
        cuda-nvgraph-$CUDA_PKG_VERSION \\
        cuda-cusolver-$CUDA_PKG_VERSION \\
        cuda-cublas-8-0=8.0.61.2-1 \\
        cuda-cufft-$CUDA_PKG_VERSION \\
        cuda-curand-$CUDA_PKG_VERSION \\
        cuda-cusparse-$CUDA_PKG_VERSION \\
        cuda-npp-$CUDA_PKG_VERSION \\
        cuda-cudart-$CUDA_PKG_VERSION && \\
    ln -s cuda-8.0 /usr/local/cuda && \\
    rm -rf /var/lib/apt/lists/*

# nvidia-docker 1.0
LABEL com.nvidia.volumes.needed="nvidia_driver"
LABEL com.nvidia.cuda.version="${CUDA_VERSION}"

RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \\
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=8.0"
"""

DOCKER_DEVEL_CONTENTS = {}
DOCKER_DEVEL_CONTENTS['18.04'] = {}
DOCKER_DEVEL_CONTENTS['16.04'] = {}

DOCKER_DEVEL_CONTENTS['18.04']['10.0'] = """
RUN apt-get update && apt-get install -y --no-install-recommends \\
        cuda-libraries-dev-$CUDA_PKG_VERSION \\
        cuda-nvml-dev-$CUDA_PKG_VERSION \\
        cuda-minimal-build-$CUDA_PKG_VERSION \\
        cuda-command-line-tools-$CUDA_PKG_VERSION \\
        libnccl-dev=$NCCL_VERSION-1+cuda10.0 && \\
    rm -rf /var/lib/apt/lists/*

ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs
"""

DOCKER_DEVEL_CONTENTS['16.04']['10.0'] = """
RUN apt-get update && apt-get install -y --no-install-recommends \\
        cuda-libraries-dev-$CUDA_PKG_VERSION \\
        cuda-nvml-dev-$CUDA_PKG_VERSION \\
        cuda-minimal-build-$CUDA_PKG_VERSION \\
        cuda-command-line-tools-$CUDA_PKG_VERSION \\
        libnccl-dev=$NCCL_VERSION-1+cuda10.0 && \\
    rm -rf /var/lib/apt/lists/*

ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs
"""

DOCKER_DEVEL_CONTENTS['16.04']['9.0'] = """
RUN apt-get update && apt-get install -y --no-install-recommends \\
        cuda-libraries-dev-$CUDA_PKG_VERSION \\
        cuda-nvml-dev-$CUDA_PKG_VERSION \\
        cuda-minimal-build-$CUDA_PKG_VERSION \\
        cuda-command-line-tools-$CUDA_PKG_VERSION \\
        cuda-core-9-0=9.0.176.3-1 \\
        cuda-cublas-dev-9-0=9.0.176.4-1 \\
        libnccl-dev=$NCCL_VERSION-1+cuda9.0 && \\
    rm -rf /var/lib/apt/lists/*

ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs
"""

DOCKER_DEVEL_CONTENTS['16.04']['8.0'] = """
RUN apt-get update && apt-get install -y --no-install-recommends \\
        cuda-core-$CUDA_PKG_VERSION \\
        cuda-misc-headers-$CUDA_PKG_VERSION \\
        cuda-command-line-tools-$CUDA_PKG_VERSION \\
        cuda-nvrtc-dev-$CUDA_PKG_VERSION \\
        cuda-nvml-dev-$CUDA_PKG_VERSION \\
        cuda-nvgraph-dev-$CUDA_PKG_VERSION \\
        cuda-cusolver-dev-$CUDA_PKG_VERSION \\
        cuda-cublas-dev-8-0=8.0.61.2-1 \\
        cuda-cufft-dev-$CUDA_PKG_VERSION \\
        cuda-curand-dev-$CUDA_PKG_VERSION \\
        cuda-cusparse-dev-$CUDA_PKG_VERSION \\
        cuda-npp-dev-$CUDA_PKG_VERSION \\
        cuda-cudart-dev-$CUDA_PKG_VERSION \\
        cuda-driver-dev-$CUDA_PKG_VERSION && \\
    rm -rf /var/lib/apt/lists/*

ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs
"""

def write(DOCKER_FILE, version, ubuntu):
    if version in versions:
        with open(DOCKER_FILE, "a") as dockerfile:
            dockerfile.write(DOCKER_CUDA_HEADER)
            cuda = version.split('-')[0]
            try:
                dockerfile.write(DOCKER_RUNTIME_CONTENTS[ubuntu][cuda])
                if 'devel' in version:
                    dockerfile.write(DOCKER_DEVEL_CONTENTS[ubuntu][cuda])
            except KeyError as e:
                print("CUDA version %s not supported in Ubuntu %s" % (cuda, ubuntu) )
                sys.exit(1)
        return
    else:
        print("cuda: version %s not supported. Options: %s" % (version, versions))
        sys.exit(1)

