AC_COPYRIGHT([
 Copyright (c) 2013-2019 - Adjacent Link LLC, Bridgewater, New Jersey
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:
 
 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
 * Neither the name of Adjacent Link LLC nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 See toplevel COPYING for more information.
])

AC_INIT([emane],1.2.5,[emane at adjacent link dot com])

AC_CONFIG_SRCDIR([src])

AC_CONFIG_MACRO_DIR([m4])

AM_INIT_AUTOMAKE([])

: ${CXXFLAGS="-g -O3 -Wall -Wextra"}

AC_PROG_CXX

AC_PROG_CXXCPP

AX_CXX_COMPILE_STDCXX_11([noext],[mandatory])

AC_PROG_INSTALL

AC_PROG_LIBTOOL

AC_LANG([C++])

AC_ARG_WITH(profile,
           [  --with-profile          add profiling support (off)])

AC_ARG_WITH(debug,
           [  --with-debug            add debug support (off)])

AC_ARG_WITH(dot,
           [  --with-dot              Use dot with doxygen (off)])

AC_ARG_WITH([verbose-logging],
           [  --without-verbose-logging  disable verbose logging support (on)],
           [],
           [with_verbose_logging=yes])


AC_ARG_WITH([testcases],
           [  --with-testcases        add testcases to build (off)],
           [AM_CONDITIONAL(WITH_TESTCASES,true)],
           [AM_CONDITIONAL(WITH_TESTCASES,false)])


AC_ARG_VAR([pcap_CFLAGS],[C compiler flags for libpcap])
AC_ARG_VAR([pcap_LIBS], [linker flags for libpcap])

PKG_CHECK_MODULES([libxml2], libxml-2.0)
PKG_CHECK_MODULES([protobuf], protobuf)
PKG_CHECK_MODULES([libpcre], libpcre)
PKG_CHECK_MODULES([libuuid], uuid)

AC_CHECK_HEADER(sys/timerfd.h,
 [],
 [AC_MSG_ERROR("Missing sys/timerfd.h header")])


LDFLAGS="$LDFLAGS -lpthread -ldl"

_cppflags=${CPPFLAGS}
_ldflags=${LDFLAGS}

# Checks for libraries.
if test "${pcap_CFLAGS}" != ""; then
   CPPFLAGS="${pcap_CFLAGS} ${CPPFLAGS}"
fi

if test "${pcap_LIBS}" != ""; then
   LDFLAGS="${pcap_LIBS} ${LDFLAGS}"
fi

AC_CHECK_HEADER(pcap.h,
 [AC_CHECK_LIB([pcap],pcap_open_live,[pcap_LIBS+=" -lpcap"],
 [AC_MSG_ERROR("pcap not installed")])],
 [AC_MSG_ERROR("Missing pcap development headers")])


CPPFLAGS=${_cppflags}
LDFLAGS=${_ldflags}

AC_SUBST(pcap_CFLAGS)
AC_SUBST(pcap_LIBS)

AC_C_BIGENDIAN(AC_DEFINE([__BYTE_ORDER],__BIG_ENDIAN,[Big Endian]),
AC_DEFINE([__BYTE_ORDER],__LITTLE_ENDIAN,[Little Endian]))
AC_DEFINE([__LITTLE_ENDIAN],1234,[for the places where it is not defined])
AC_DEFINE([__BIG_ENDIAN],4321,[for the places where it is not defined])

# options for use with gprof
if test "$with_profile" = "yes"
then
CXXFLAGS="$CXXFLAGS -g -pg"
LDFLAGS="$LDFLAGS -g -pg"
fi

# options for use with debug
if test "$with_debug" = "yes"
then
AX_CHECK_COMPILE_FLAG([-Og],
 [CXXFLAGS="$CXXFLAGS -g -Og"],
 [CXXFLAGS="$CXXFLAGS -g -O0"])
AX_CHECK_LINK_FLAG([-Og],
 [LDFLAGS="$LDFLAGS -g -Og"],
 [LDFLAGS="$LDFLAGS -g -O0"])
fi

# options for use with verbose logging
if test "$with_verbose_logging" = "yes"
then
CPPFLAGS="$CPPFLAGS -DVERBOSE_LOGGING"
fi

AC_SUBST(libemane_CFLAGS,
["${libxml2_CFLAGS} ${protobuf_CFLAGS} ${libpcre_CFLAGS} ${libuuid_CFLAGS}"])

LANG=C
AC_SUBST(LANG)

AC_CHECK_HEADER([pty.h],
[AM_CONDITIONAL(EMANE_HAS_PTY,true)],
[AC_CHECK_HEADER([util.h],
 [AM_CONDITIONAL(EMANE_HAS_PTY,true)],
 [AM_CONDITIONAL(EMANE_HAS_PTY,false)])])

# check for doxygen
AC_CHECK_PROG(HAVE_DOXYGEN, doxygen, true, false)
AM_CONDITIONAL(HAVE_DOXYGEN,$HAVE_DOXYGEN)

if test "$with_dot" = "yes"
then
# check for dot
AC_CHECK_PROG(HAVE_DOT, dot, true, false)
AM_CONDITIONAL(HAVE_DOT,$HAVE_DOT)
else
AM_CONDITIONAL(HAVE_DOT,false)
fi

AM_PATH_PYTHON([2.6])

# check for rpmbuild
AC_CHECK_PROG(HAVE_RPMBUILD, rpmbuild, true, false)
AM_CONDITIONAL(HAVE_RPMBUILD,$HAVE_RPMBUILD)

# check for deb
AC_CHECK_PROG(HAVE_DEB, dh_clean, true, false)
AM_CONDITIONAL(HAVE_DEB,$HAVE_DEB)

AC_CHECK_FILE(/etc/lsb-release,[. /etc/lsb-release],)
AC_SUBST(DISTRIB_CODENAME)

AC_SUBST(RELDATE,`date -I`)

AC_SUBST(DATE_RFC2822,`date -R`)

AC_SUBST(PACKAGE_VERSION_CODE,
 [`echo $PACKAGE_VERSION | awk '/^([[0-9]]+)\.([[0-9]]+)\.([[0-9]]+)$/ { split($0, m, "."); printf("%d", (m[[1]]*(2^16)) + (m[[2]]*(2^8)) + m[[3]]);}'`])

AC_OUTPUT(
 Makefile 
 include/emane/Makefile
 include/emane/utils/Makefile
 include/emane/controls/Makefile
 include/emane/application/Makefile
 include/emane/generators/Makefile
 include/emane/generators/eel/Makefile
 include/emane/events/Makefile
 include/emane/models/Makefile
 include/emane/models/tdma/Makefile
 include/emane/version.h
 src/Makefile
 src/libemanexmlparser/Makefile 
 src/libemane/Makefile
 src/libemane/libemane.pc
 src/applications/Makefile
 src/applications/emane/Makefile
 src/applications/emaneinfo/Makefile
 src/applications/emanetransportd/Makefile
 src/applications/emaneeventd/Makefile
 src/applications/emaneeventservice/Makefile
 src/python/Makefile
 src/python/emane/info/__init__.py
 src/transports/Makefile
 src/transports/common/Makefile
 src/transports/virtual/Makefile
 src/transports/raw/Makefile
 src/models/Makefile
 src/models/mac/Makefile
 src/models/mac/bypass/Makefile
 src/models/phy/Makefile
 src/models/phy/bypass/Makefile
 src/models/mac/rfpipe/Makefile
 src/models/mac/ieee80211abg/Makefile
 src/models/mac/tdma/Makefile
 src/models/mac/tdma/eventscheduler/Makefile
 src/models/shim/Makefile
 src/models/shim/timinganalysis/Makefile
 src/models/shim/phyapitest/Makefile
 src/models/shim/commeffect/Makefile
 src/agents/Makefile
 src/agents/gpsdlocation/Makefile
 src/generators/Makefile
 src/generators/eel/Makefile
 src/generators/eel/loaders/Makefile
 src/generators/eel/loaders/location/Makefile
 src/generators/eel/loaders/pathloss/Makefile
 src/generators/eel/loaders/antennaprofile/Makefile
 src/generators/eel/loaders/commeffect/Makefile
 src/generators/eel/loaders/fadingselection/Makefile
 test/Makefile
 test/harness/Makefile
 test/harness/gainscenario/Makefile
 test/harness/noisescenario/Makefile
 test/harness/profilescenario/Makefile
 test/harness/propagationscenario/Makefile
 test/harness/phyupstreamscenario/Makefile
 test/testcases/Makefile
 test/testcases/gainscenario001/Makefile
 test/testcases/noisescenario001/Makefile
 test/testcases/noisescenario002/Makefile
 test/testcases/profilescenario001/Makefile
 test/testcases/propagationscenario001/Makefile
 test/testcases/phyupstreamscenario001/Makefile
 test/testcases/phyupstreamscenario002/Makefile
 test/testcases/phyupstreamscenario003/Makefile
 test/testcases/phyupstreamscenario004/Makefile
 test/testcases/phyupstreamscenario005/Makefile
 scripts/Makefile
 scripts/doc/Makefile
 dtd/Makefile
 schema/Makefile
 doc/Makefile
)
