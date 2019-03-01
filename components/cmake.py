DOCKER_CONTENTS = """
#################################### CMAKE #####################################

RUN mkdir ${HOME}/%s/build \\
 && cd ${HOME}/%s/build \\
 && cmake .. \\
 && make -j2
"""

def write(DOCKER_FILE, name):
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS % (name, name))
    return

