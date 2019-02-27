DOCKER_CONTENTS = """
##################################### APT ######################################

RUN apt-get -o Acquire::ForceIPv4=true update \\
 && apt-get -o Acquire::ForceIPv4=true install -yq --no-install-recommends \\
%s && apt-get clean \\
 && rm -rf /var/lib/apt/lists/*
"""

def write(DOCKER_FILE, package_list):
    pstr = ''
    for p in package_list:
        pstr += '    ' + p + ' \\\n'
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS % pstr)
    return

