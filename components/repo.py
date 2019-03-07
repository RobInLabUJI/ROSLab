DOCKER_REPO_HEADER = """
##################################### REPO #####################################

RUN apt-get update \\
 && apt-get install -yq --no-install-recommends \\
    software-properties-common \\
 && apt-get clean \\
 && rm -rf /var/lib/apt/lists/*
"""
DOCKER_REPO_ADD = """
RUN apt-add-repository %s
"""

def write(DOCKER_FILE, repo_list):
    pstr = DOCKER_REPO_HEADER
    for p in repo_list:
        pstr += DOCKER_REPO_ADD % p
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(pstr)
    return

