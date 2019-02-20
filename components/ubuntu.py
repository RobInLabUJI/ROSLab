import sys

versions = [16.04, '16.04', 'xenial', 
            18.04, '18.04', 'melodic']

DOCKER_CONTENTS = "FROM ubuntu:%s\n"

def write(DOCKER_FILE, version):
    if version in versions:
        with open(DOCKER_FILE, "w") as dockerfile:
            dockerfile.write(DOCKER_CONTENTS % version)
        return
    else:
        print("ubuntu: version %s not supported. Options: %s" % (version, versions))
        sys.exit(1)

