DOCKER_CONTENTS = """
#################################### CMAKE #####################################

RUN mkdir %s/build \\
 && cd %s/build \\
 && cmake .. \\
 && make -j2
"""

def write(DOCKER_FILE, name):
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS % (name, name))
    return

