noinst_PROGRAMS = phydownstreamspeed

phydownstreamspeed_CPPFLAGS =         \
 -I@top_srcdir@/include               \
 -I@top_srcdir@/src/libemane          \
 $(AM_CPPFLAGS)                       \
 $(libemane_CFLAGS)

phydownstreamspeed_LDADD =            \
 $(libuuid_LIBS)                      \
 $(libxml2_LIBS)                      \
 @top_srcdir@/src/libemane/.libs/libemane.la

phydownstreamspeed_SOURCES =         \
 main.cc                             \
 harnessdownstreamtransport.h        \
 utils.h

EXTRA_DIST=                          \
 generate-graphs                     \
 run-it.sh

