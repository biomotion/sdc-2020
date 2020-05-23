FROM ubuntu:18.04

RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime

RUN apt-get update && \
    apt-get install -y \
        python3-pip \
        python3-tk \
        python-opencv \
        wget

WORKDIR /tmp

RUN wget https://github.com/eddyhkchiu/mahalanobis_3d_multi_object_tracking/raw/master/requirements.txt -O 3dmot-requirements.txt && \
    wget https://github.com/nutonomy/nuscenes-devkit/raw/master/setup/requirements.txt -O nuscenese-requirements.txt && \
    pip3 install -r /tmp/3dmot-requirements.txt && \
    pip3 install -r /tmp/nuscenese-requirements.txt

RUN pip3 install \
        jupyter \
        numpy \
        scipy \
        matplotlib \
        nuscenes-devkit

RUN jupyter notebook --generate-config \
    && echo "c.NotebookApp.allow_root = True" >> /root/.jupyter/jupyter_notebook_config.py \
    && echo "c.NotebookApp.ip = '0.0.0.0'" >> /root/.jupyter/jupyter_notebook_config.py \
    && echo "c.NotebookApp.password = 'sha1:b22dd1ea32ec:8d7e23c36485866b1f6d254be4eceb5fc389d0d1'" >> /root/.jupyter/jupyter_notebook_config.py \
    && echo "c.NotebookApp.open_browser = False" >> /root/.jupyter/jupyter_notebook_config.py