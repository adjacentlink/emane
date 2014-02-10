noinst_PROGRAMS = noisescenario

noisescenario_CPPFLAGS =      \
 -I@top_srcdir@/include       \
 -I@top_srcdir@/src/libemane

noisescenario_CXXFLAGS =    \
 $(libemane_CFLAGS)

noisescenario_SOURCES =  \
 main.cc            

EXTRA_DIST=   \
 noisescenario.xsd

noisescenario_LDADD =                         \
 -L@top_srcdir@/src/libemane/.libs           \
 -L@top_srcdir@/src/libemanexmlparser/.libs  \
 -lemanexmlparser                            \
 $(libemane_LIBS)

