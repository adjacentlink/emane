noinst_PROGRAMS = gainscenario

gainscenario_CPPFLAGS =               \
 -I@top_srcdir@/include               \
 -I@top_srcdir@/src/libemane          \
 $(AM_CPPFLAGS)                       \
 $(libemane_CFLAGS)

gainscenario_LDADD =                         \
 $(libxml2_LIBS)                             \
 @top_srcdir@/src/libemane/.libs/libemane.la

gainscenario_SOURCES =  \
 main.cc            

EXTRA_DIST=   \
 gainscenario.xsd

