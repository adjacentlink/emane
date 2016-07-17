noinst_PROGRAMS = profilescenario

profilescenario_CPPFLAGS =    \
 -I@top_srcdir@/include       \
 -I@top_srcdir@/src/libemane  \
 $(AM_CPPFLAGS)               \
 $(libemane_CFLAGS)

profilescenario_LDADD =                      \
 $(libxml2_LIBS)                             \
 @top_srcdir@/src/libemane/.libs/libemane.la

profilescenario_SOURCES =  \
 main.cc            

EXTRA_DIST=   \
 profilescenario.xsd
