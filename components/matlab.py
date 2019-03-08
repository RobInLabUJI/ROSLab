DOCKER_CONTENTS = """
#################################### MATLAB ####################################

RUN apt-get update \\
 && apt-get install -yq --no-install-recommends \\
    libpng12-dev libfreetype6-dev \\
    libblas-dev liblapack-dev gfortran build-essential xorg \\
 && apt-get clean \\
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install matlab_kernel

ENV PATH="%s/bin:${PATH}"
"""

def write(DOCKER_FILE, host_path):
    with open(DOCKER_FILE, "a") as dockerfile:
        dockerfile.write(DOCKER_CONTENTS % host_path)
    return

