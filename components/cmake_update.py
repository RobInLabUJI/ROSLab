import os, sys

#versions = ['3.7.2','3.13.4']

DOCKER_CORE_CONTENTS = """
################################# CMAKE_UPDATE #################################

RUN apt remove -y --purge --auto-remove cmake

RUN apt-get update \\
 && apt-get install -yq --no-install-recommends wget libcurl4-openssl-dev zlib1g-dev\\
 && apt-get clean \\
 && rm -rf /var/lib/apt/lists/*

RUN mkdir /temp_cmake && cd /temp_cmake \\
 && wget https://cmake.org/files/v%s/cmake-%s.tar.gz \\
 && tar -xzvf cmake-%s.tar.gz \\
 && cd cmake-%s \\
 && ./bootstrap --system-curl && make -j4 && make install \\
 && rm -fr /temp_cmake
"""

def write(DOCKER_FILE, version):
    if True: #version in versions:
        major = '.'.join(version.split('.')[:-1])
        with open(DOCKER_FILE, "a") as dockerfile:
            dockerfile.write(DOCKER_CORE_CONTENTS % (major, version, version, version))
        return
    else:
        print("cmake_update: version %s not supported. Options: %s" % (version, versions))
        sys.exit(1)


