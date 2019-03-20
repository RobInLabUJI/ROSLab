DOCKER_CONTENTS = """
##################################### PIP ######################################

RUN pip2 install --upgrade pip

RUN pip2 install %s
"""

def write(DOCKER_FILE, package_list):
    pstr = ''
    for p in package_list:
        pstr += ' \\\n    ' + p
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS % pstr)
    return

