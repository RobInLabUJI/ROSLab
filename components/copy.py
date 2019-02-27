DOCKER_CONTENTS = """
##################################### COPY #####################################

RUN mkdir ${HOME}/%s

COPY . ${HOME}/%s
"""

def write(DOCKER_FILE, name):
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS % (name, name))
    return

