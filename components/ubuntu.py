import sys

versions = [16.04, '16.04', 'xenial', 
            18.04, '18.04', 'melodic']

DOCKER_CONTENTS = "FROM ubuntu:%s\n"

DOCKER_CONTENTS_OPENGL_RUNTIME = "FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04\n"

DOCKER_CONTENTS_OPENGL_DEVEL = "FROM nvidia/opengl:1.0-glvnd-devel-ubuntu16.04\n"

def write(DOCKER_FILE, version, opengl):
    if version in versions:
        if opengl is None:
            with open(DOCKER_FILE, "w") as dockerfile:
                dockerfile.write(DOCKER_CONTENTS % version)
        elif opengl == 'runtime':
            with open(DOCKER_FILE, "w") as dockerfile:
                dockerfile.write(DOCKER_CONTENTS_OPENGL_RUNTIME)
        elif opengl == 'devel':
            with open(DOCKER_FILE, "w") as dockerfile:
                dockerfile.write(DOCKER_CONTENTS_OPENGL_DEVEL)
        return
    else:
        print("ubuntu: version %s not supported. Options: %s" % (version, versions))
        sys.exit(1)

