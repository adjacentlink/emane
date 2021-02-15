FROM fedora:33

RUN dnf -y update


RUN dnf -y install git gcc-c++ make autoconf automake libtool rpm-build python3-setuptools \
                   libxml2-devel libpcap-devel pcre-devel libuuid-devel python3-devel \
                   python3-protobuf protobuf-devel

WORKDIR /opt
RUN git clone https://github.com/adjacentlink/emane -b develop

WORKDIR emane
RUN ./autogen.sh
RUN ./configure
RUN make rpm
RUN dnf -y install $(find .rpmbuild/RPMS -name "*\.rpm")

RUN echo 'complete'

