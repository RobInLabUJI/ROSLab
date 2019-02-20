FROM python:3-alpine

RUN pip install pyyaml notedown

RUN mkdir /project

ADD roslab_create.py /

COPY components /components

CMD [ "python", "roslab_create.py", "/project"]
