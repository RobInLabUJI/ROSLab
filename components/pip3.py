DOCKER_CONTENTS = """
##################################### PIP3 #####################################

RUN pip3 install --upgrade pip

RUN pip3 install %s
"""

def write(DOCKER_FILE, package_list):
    pstr = ''
    for p in package_list:
        pstr += ' \\\n    ' + p
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS % pstr)
    return

