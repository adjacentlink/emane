FROM centos:centos7

RUN mkdir -p /opt/built

RUN yum -y update
RUN yum clean all

RUN yum -y install epel-release
RUN yum -y install git gcc-c++ make autoconf automake libtool rpm-build python-setuptools \
                   libxml2-devel libpcap-devel pcre-devel libuuid-devel python-devel \
                   protobuf protobuf-devel protobuf-compiler protobuf-python

# Install devtoolset-9 to support c++17 compatibility
RUN yum -y install centos-release-scl
RUN yum -y install devtoolset-9

WORKDIR /opt
RUN git clone https://github.com/adjacentlink/emane -b develop

WORKDIR emane

# Using devtoolset-9
RUN scl enable devtoolset-9 "./autogen.sh && ./configure && make rpm"
RUN cp $(find .rpmbuild/RPMS -name "*\.rpm") /opt/built
RUN yum -y install /opt/built/*

RUN echo 'complete'

