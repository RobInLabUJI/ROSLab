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

