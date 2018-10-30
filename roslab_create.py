#!/usr/bin/env python
import os, subprocess, sys, yaml

distros = ['kinetic', 'lunar', 'melodic']

def main():
    if len(sys.argv) != 2:
        print("Usage: %s <folder>" % sys.argv[0])
        sys.exit(1)
    
    os.chdir(sys.argv[1])
    try:
        with open(sys.argv[1] + '/roslab.yaml', 'r') as stream:
            try:
                yl = yaml.load(stream)
            except yaml.YAMLError as e:
                print(e)
                sys.exit(1)
    except FileNotFoundError:
        print("File 'roslab.yaml' not found in folder '%s'" % sys.argv[1])
        sys.exit(1)
    distro = yl['distro']
    if not distro in distros:
        print('Distro %s not supported.' % distro)
        sys.exit(1)
    
    head = "FROM roslab/roslab:" + distro + """

USER root

RUN apt-get update \\
 && apt-get install -yq --no-install-recommends \\
"""

    tail = """ && apt-get clean \\
 && rm -rf /var/lib/apt/lists/*

COPY . ${HOME}
RUN chown -R ${NB_UID} ${HOME}

USER ${NB_USER}
WORKDIR ${HOME}
"""
    with open("Dockerfile", "w") as dockerfile:
        dockerfile.write(head)
        for p in yl['packages']:
            dockerfile.write("    %s \\\n" % p)
        dockerfile.write(tail)
    
    with open("docker_build.sh", "w") as scriptfile:
        scriptfile.write("#!/bin/sh\ndocker build -t %s -f Dockerfile ." % yl['name'])
    os.chmod("docker_build.sh", 0o755)
    
    with open("docker_run.sh", "w") as scriptfile:
        scriptfile.write("#!/bin/sh\ndocker run --rm -p 8888:8888 %s" % yl['name'])
    os.chmod("docker_run.sh", 0o755)
        
if __name__ == "__main__":
    main()