FROM centos:centos7

RUN mkdir -p /opt/built

RUN yum -y update
RUN yum clean all

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
RUN cp $(find .rpmbuild/RPMS -name "*\.rpm") /opt/built
RUN yum -y install /opt/built/*

RUN echo 'complete'

