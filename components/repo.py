DOCKER_CONTENTS = """
##################################### REPO #####################################
"""
repo_commands = {}

repo_commands['name'] = """

"""

def write(DOCKER_FILE, repo_list):
    pstr = DOCKER_CONTENTS
    for p in repo_list:
        pstr += repo_commands[p]
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(pstr)
    return

