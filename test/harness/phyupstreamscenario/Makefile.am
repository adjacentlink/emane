noinst_PROGRAMS = phyupstreamscenario

phyupstreamscenario_CPPFLAGS =        \
 -I@top_srcdir@/include               \
 -I@top_srcdir@/src/libemane          \
 $(AM_CPPFLAGS)                       \
 $(libemane_CFLAGS)

phyupstreamscenario_LDADD =                  \
 $(libuuid_LIBS)                             \
 $(libxml2_LIBS)                             \
 @top_srcdir@/src/libemane/.libs/libemane.la

phyupstreamscenario_SOURCES =         \
 main.cc            

EXTRA_DIST=                           \
 phyupstreamscenario.xsd

