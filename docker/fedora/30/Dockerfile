FROM fedora:30

# install EMANE build dependencies
RUN dnf install gcc-c++ autoconf automake libtool libxml2-devel protobuf-devel python2-protobuf python3-protobuf libpcap-devel pcre-devel libuuid-devel rpm-build python2-setuptools python3-setuptools  python2-devel python3-devel git make -y

# clone the emane repo  (?)
RUN git clone https://github.com/adjacentlink/emane.git

# build RPMs
RUN cd emane && ./autogen.sh && ./configure && make rpm

# install RPMs
RUN cd emane/.rpmbuild && dnf install $(ls RPMS/x86_64/*.rpm RPMS/noarch/*.rpm | grep -v debuginfo) -y