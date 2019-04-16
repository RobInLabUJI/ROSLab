DOCKER_CONTENTS = """
##################################### TAIL #####################################

RUN chown ${NB_UID} ${HOME}/%s
 
USER ${NB_USER}

WORKDIR ${HOME}/%s
"""

def write(DOCKER_FILE, name):
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS % (name, name))
    return

