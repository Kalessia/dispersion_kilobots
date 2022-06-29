FROM alpine as intermediate
MAINTAINER leo.cazenille@gmail.com

# Install git and ssh
RUN apk --no-cache --update-cache add git openssh-client

# Add credentials on build
ARG SSH_PRIVATE_KEY
RUN \
    mkdir /root/.ssh/ && \
    echo "${SSH_PRIVATE_KEY}" > /root/.ssh/id_rsa && \
    chmod 0600 /root/.ssh/id_rsa

# Make sure your domain is accepted
RUN \
    touch /root/.ssh/known_hosts && \
    ssh-keyscan bitbucket.org >> /root/.ssh/known_hosts

# Download repositories
RUN \
    git clone git@bitbucket.org:AubertKato/limmsswarm.git && \
    cd limmsswarm && \
    git checkout origin/triple_mse


########################
#FROM lcazenille/ubuntupython3andjava
#FROM lcazenille/bioneatandpython:SSCI2019
#FROM lcazenille/bioneatandpython
FROM python:3.7.10

ENV DEBIAN_FRONTEND noninteractive

# Update system and install relevant packages
RUN \
    apt-get update -y && \
    #apt-get install -yq openssh-server openssh-client rsync gosu && \
apt-get install -yq rsync gosu build-essential git gcc-avr gdb-avr binutils-avr avr-libc avrdude libsdl1.2-dev libjansson-dev libsubunit-dev cmake check xserver-xorg-video-dummy xserver-xorg-input-void x11-apps && \
#    apt-get install -yq rsync gosu build-essential git gcc-avr gdb-avr binutils-avr avr-libc avrdude libsdl1.2-dev libjansson-dev libsubunit-dev cmake check xserver-xorg-video-dummy  x11-apps && \
    rm -rf /var/lib/apt/lists/*


RUN mkdir -p /home/user

## Create ssh key
#RUN \
#    mkdir /home/user/.ssh && \
#    ssh-keygen -q -t rsa -N '' -f /home/user/.ssh/id_rsa && \
#    cat /home/user/.ssh/id_rsa.pub > /home/user/.ssh/authorized_keys

# Copy repositories
COPY --from=intermediate /limmsswarm /home/user/limmsswarm
# Remove git directories (we can not use them because we would need private credentials)
RUN rm -fr /home/user/limmsswarm/.git*

#VOLUME /tmp/.X11-unix

# Install kilombo
RUN \
    cp /home/user/limmsswarm/kilo/xorg.conf /etc/X11/xorg.conf && \
    git clone https://github.com/JIC-CSB/kilombo.git && \
    cd kilombo && mkdir build && cd build && \
    cmake .. && make -j 10 && make install

RUN \
    pip3 install numpy scipy pyaml matplotlib seaborn scikit-learn

# Compile limmsswarm
RUN \
    cd /home/user/limmsswarm/kilo && \
    make -j 20

# Prepare for entrypoint execution
#CMD ["bash"]
ENTRYPOINT ["/home/user/limmsswarm/kilo/entrypoint.sh"]
#CMD ["1000", "normal"]

# MODELINE	"{{{1
# vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
# vim:foldmethod=marker
