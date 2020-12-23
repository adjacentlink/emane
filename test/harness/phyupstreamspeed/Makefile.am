noinst_PROGRAMS = phyupstreamspeed

phyupstreamspeed_CPPFLAGS =        \
 -I@top_srcdir@/include            \
 -I@top_srcdir@/src/libemane       \
 $(AM_CPPFLAGS)                    \
 $(libemane_CFLAGS)

phyupstreamspeed_LDADD =           \
 $(libuuid_LIBS)                   \
 $(libxml2_LIBS)                   \
 @top_srcdir@/src/libemane/.libs/libemane.la

phyupstreamspeed_SOURCES =         \
 main.cc                           \
 harnessupstreamtransport.h        \
 utils.h

EXTRA_DIST=                        \
 antenna30dsector.xml              \
 antennaomni.xml                   \
 antennaprofiles.xml               \
 blockageaft.xml                   \
 generate-graphs                   \
 run-it.sh



