noinst_PROGRAMS = noisescenario

noisescenario_CPPFLAGS =      \
 -I@top_srcdir@/include       \
 -I@top_srcdir@/src/libemane  \
 $(AM_CPPFLAGS)               \
 $(libemane_CFLAGS)

noisescenario_LDADD =                        \
 $(libxml2_LIBS)                             \
 @top_srcdir@/src/libemane/.libs/libemane.la

noisescenario_SOURCES =  \
 main.cc            

EXTRA_DIST=   \
 noisescenario.xsd
