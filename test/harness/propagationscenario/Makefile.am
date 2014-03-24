noinst_PROGRAMS = propagationscenario

propagationscenario_CPPFLAGS =        \
 -I@top_srcdir@/include               \
 -I@top_srcdir@/src/libemane

propagationscenario_CXXFLAGS =        \
 $(libemane_CFLAGS)

propagationscenario_SOURCES =         \
 main.cc            

EXTRA_DIST=                           \
 propagationscenario.xsd

propagationscenario_LDADD =                  \
 -L@top_srcdir@/src/libemane/.libs           \
 -L@top_srcdir@/src/libemanexmlparser/.libs  \
 -lemanexmlparser                            \
 $(libemane_LIBS)
