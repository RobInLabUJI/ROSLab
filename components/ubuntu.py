versions = ['16.04', 'xenial', '18.04', 'melodic']

DOCKER_CONTENTS = """
FROM ubuntu:%s
"""

def write(DOCKER_FILE, version):
    with open(DOCKER_FILE, "w") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS % version)


