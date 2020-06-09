FROM centos:7

# install EMANE build dependencies and tools
RUN yum install -y autoconf automake git libtool libxml2-devel libpcap-devel pcre-devel libuuid-devel python-devel python-setuptools rpm-build make gcc-c++ 

# enable the EPEL package repository and install Google Protocol Buffer dependencies
RUN yum install -y epel-release
RUN yum install -y protobuf protobuf-devel protobuf-compiler protobuf-python

# clone the emane repo  (?)
RUN git clone https://github.com/adjacentlink/emane.git

# build RPMs
# RUN cd emane
# RUN ls
RUN cd emane && ls && ./autogen.sh && ./configure && make rpm

# install RPMs
RUN cd emane && cd .rpmbuild && yum install -y $(ls RPMS/x86_64/*.rpm RPMS/noarch/*.rpm | grep -v debuginfo)
# RUN yum install -y $(ls RPMS/x86_64/*.rpm RPMS/noarch/*.rpm | grep -v debuginfo)