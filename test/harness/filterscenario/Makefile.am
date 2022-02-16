noinst_PROGRAMS = filterscenario

filterscenario_CPPFLAGS =      \
 -I@top_srcdir@/include       \
 -I@top_srcdir@/src/libemane  \
 $(AM_CPPFLAGS)               \
 $(libemane_CFLAGS)

filterscenario_LDADD =                        \
 $(libxml2_LIBS)                             \
 @top_srcdir@/src/libemane/.libs/libemane.la

filterscenario_SOURCES =  \
 main.cc

EXTRA_DIST=   \
 filterscenario.xsd
