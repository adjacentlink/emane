noinst_PROGRAMS = propagationscenario

propagationscenario_CPPFLAGS =        \
 -I@top_srcdir@/include               \
 -I@top_srcdir@/src/libemane          \
 $(AM_CPPFLAGS)                       \
 $(libemane_CFLAGS)

propagationscenario_LDADD =                  \
 $(libxml2_LIBS)                             \
 @top_srcdir@/src/libemane/.libs/libemane.la

propagationscenario_SOURCES =         \
 main.cc            

EXTRA_DIST=                           \
 propagationscenario.xsd
