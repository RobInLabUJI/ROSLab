import sys

versions = ['binary']

DOCKER_CORE_CONTENTS = """
##################################### ICUB #####################################

# install packages
RUN apt-get -o Acquire::ForceIPv4=true update && apt-get -o Acquire::ForceIPv4=true install -q -y \\
    dirmngr \\
    gnupg2 \\
    lsb-release \\
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 57A5ACB6110576A6

# setup sources.list
RUN echo "deb http://www.icub.org/ubuntu `lsb_release -sc` contrib/science" > /etc/apt/sources.list.d/icub.list

RUN apt-get -o Acquire::ForceIPv4=true update\\
 && apt-get -o Acquire::ForceIPv4=true install --no-install-recommends -y \\
    icub \\
    && rm -rf /var/lib/apt/lists/*
"""

def write(DOCKER_FILE, version):
    if version in versions:
        with open(DOCKER_FILE, "a") as dockerfile:
            dockerfile.write(DOCKER_CORE_CONTENTS)
        return
    else:
        print("icub: version %s not supported. Options: %s" % (version, versions))
        sys.exit(1)

