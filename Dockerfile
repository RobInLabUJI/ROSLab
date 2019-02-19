FROM python:3

RUN pip install pyyaml notedown

RUN mkdir /project

ADD components roslab_create.py /

CMD [ "python", "roslab_create.py", "/project"]
