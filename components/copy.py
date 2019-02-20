DOCKER_CONTENTS = """
##################################### COPY #####################################

COPY . ${HOME}
"""

def write(DOCKER_FILE):
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS)
    return

