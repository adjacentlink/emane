FROM fedora:35

RUN mkdir -p /opt/built

RUN dnf -y update
RUN dnf clean all


RUN dnf -y install git gcc-c++ make autoconf automake libtool rpm-build python3-setuptools \
                   libxml2-devel libpcap-devel pcre-devel libuuid-devel python3-devel \
                   python3-protobuf protobuf-devel

WORKDIR /opt
RUN git clone https://github.com/adjacentlink/emane -b develop

WORKDIR emane
RUN ./autogen.sh
RUN ./configure
RUN make rpm
RUN cp $(find .rpmbuild/RPMS -name "*\.rpm") /opt/built
RUN dnf -y install /opt/built/*

RUN echo 'complete'

