DOCKER_CONTENTS = """
################################### CUSTOM #####################################

RUN %s
"""

def write(DOCKER_FILE, command_list):
    cstr = command_list[0]
    for c in command_list[1:]:
        cstr += ' \\\n && ' + c
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS % cstr)
    return

