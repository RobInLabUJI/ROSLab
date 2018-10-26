#!/usr/bin/env python
import os, subprocess, sys, yaml

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
        
    head = """FROM robinlab/roslab

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
    
if __name__ == "__main__":
    main()