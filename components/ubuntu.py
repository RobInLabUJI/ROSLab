import sys

versions = [16.04, '16.04', 'xenial', 
            18.04, '18.04', 'melodic']

DOCKER_CONTENTS = "FROM ubuntu:%s\n"

DOCKER_CONTENTS_OPENGL_RUNTIME = "FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu%s\n"

DOCKER_CONTENTS_OPENGL_DEVEL = "FROM nvidia/opengl:1.0-glvnd-devel-ubuntu%s\n"

def write(DOCKER_FILE, version, opengl):
    if version in versions:
        if opengl is None:
            with open(DOCKER_FILE, "w") as dockerfile:
                dockerfile.write(DOCKER_CONTENTS % version)
        else:
            if version == 'xenial':
                version = '16.04'
            elif version == 'melodic':
                version = '18.04'

            if opengl == 'runtime':
                with open(DOCKER_FILE, "w") as dockerfile:
                    dockerfile.write(DOCKER_CONTENTS_OPENGL_RUNTIME % version)
            elif opengl == 'devel':
                with open(DOCKER_FILE, "w") as dockerfile:
                    dockerfile.write(DOCKER_CONTENTS_OPENGL_DEVEL % version)
            else:
                print("opengl: option %s not supported. Options: %s" % (opengl, ['runtime', 'devel']))
                sys.exit(1)
        return
    else:
        print("ubuntu: version %s not supported. Options: %s" % (version, versions))
        sys.exit(1)

def canonic_version(version):
    if version == 'xenial' or version == 16.04:
        version = '16.04'
    elif version == 'melodic' or version == 18.04:
        version = '18.04'
    return version

FASTEST_MIRROR = """
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y \\
        wget \\
    && rm -rf /var/lib/apt/lists/*

RUN wget -q -nv -O- http://ftp.debian.org/debian/pool/main/n/netselect/netselect_0.3.ds1-26_amd64.deb > /tmp/netselect_0.3.ds1-26_amd64.deb

RUN dpkg -i /tmp/netselect_0.3.ds1-26_amd64.deb

RUN netselect -s1 -t20 `wget -q -nv -O- https://launchpad.net/ubuntu/+archivemirrors | grep -P -B8 "statusUP|statusSIX" | grep -o -P "(f|ht)tp.*\\"" | tr '"\\n' '  '` 2>/dev/null | awk '{ print $2 }' > /fastest-mirror

RUN MIRROR=`cat /fastest-mirror` && sed -i "s#http://archive.ubuntu.com/ubuntu/#$MIRROR#g" /etc/apt/sources.list
"""

def select_fastest_mirror(DOCKER_FILE):
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(FASTEST_MIRROR)

COUNTRY_MIRROR = """
RUN sed --in-place --regexp-extended "s/(\/\/)(archive\.ubuntu)/\\1%s.\\2/" /etc/apt/sources.list && \
	apt-get update && apt-get upgrade --yes
"""

def select_country_mirror(DOCKER_FILE, country_code):
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(COUNTRY_MIRROR % country_code)

