DOCKER_CONTENTS = """
##################################### TAIL #####################################


RUN chown -R ${NB_UID} ${HOME}

USER ${NB_USER}

WORKDIR ${HOME}
"""

def write(DOCKER_FILE):
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS)
    return

