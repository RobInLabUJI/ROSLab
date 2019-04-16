FROM jupyter/base-notebook:latest

RUN conda install pyyaml

RUN conda install -c conda-forge ipywidgets

ADD roslab_create.py /home/jovyan/

COPY components /home/jovyan/components

COPY --chown=1000:1000 CreateDockerFile.ipynb /home/jovyan/

CMD ["jupyter", "notebook", "CreateDockerFile.ipynb"]
