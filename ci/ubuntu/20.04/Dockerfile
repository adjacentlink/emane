FROM ubuntu:20.04

RUN mkdir -p /opt/built

# prevent failures due to interactive apt transactions
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get -y update
RUN DEBIAN_FRONTEND=noninteractive apt-get -y upgrade --no-install-recommends

RUN DEBIAN_FRONTEND=noninteractive \
apt-get -y install git gcc g++ autoconf automake libtool dh-python  debhelper python3-setuptools \
                   libxml2-dev libprotobuf-dev libpcap-dev libpcre3-dev uuid-dev pkg-config \
                   python3-protobuf protobuf-compiler

WORKDIR /opt
RUN git clone https://github.com/adjacentlink/emane -b develop

WORKDIR emane
RUN ./autogen.sh
RUN ./configure
RUN make deb
RUN cp $(find .debbuild -name "*\.deb") /opt/built
RUN dpkg -i /opt/built/python3*\.deb /opt/built/emane*\.deb; apt-get -y install -f

RUN echo 'complete'

