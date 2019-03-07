DOCKER_CONTENTS = """
#################################### MATLAB ####################################

RUN apt-get update \\
 && apt-get install -yq --no-install-recommends \\
    libpng12-dev libfreetype6-dev \\
    libblas-dev liblapack-dev gfortran build-essential xorg \\
 && apt-get clean \\
 && rm -rf /var/lib/apt/lists/*

ENV PATH="/usr/local/MATLAB/from-host/bin:${PATH}"
"""

def write(DOCKER_FILE):
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS)
    return

