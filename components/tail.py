DOCKER_CONTENTS = """
##################################### TAIL #####################################

RUN chown -R ${NB_UID} ${HOME}

USER ${NB_USER}

WORKDIR ${HOME}/%s
"""

def write(DOCKER_FILE, name):
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS % name)
    return

