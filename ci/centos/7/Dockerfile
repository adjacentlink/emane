FROM centos:centos7

RUN yum -y update

RUN yum -y install epel-release
RUN yum -y install git gcc-c++ make autoconf automake libtool rpm-build python-setuptools \
                   libxml2-devel libpcap-devel pcre-devel libuuid-devel python-devel \
                   protobuf protobuf-devel protobuf-compiler protobuf-python

WORKDIR /opt
RUN git clone https://github.com/adjacentlink/emane -b develop

WORKDIR emane
RUN ./autogen.sh
RUN ./configure
RUN make rpm
RUN yum -y install $(find .rpmbuild/RPMS -name "*\.rpm")

RUN echo 'complete'

