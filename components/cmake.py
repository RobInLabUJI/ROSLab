DOCKER_CONTENTS = """
#################################### CMAKE #####################################

RUN mkdir build \\
 && cd build \\
 && cmake .. \\
 && make -j2
"""

def write(DOCKER_FILE):
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS)
    return

