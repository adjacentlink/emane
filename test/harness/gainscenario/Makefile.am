noinst_PROGRAMS = gainscenario

gainscenario_CPPFLAGS =               \
 -I@top_srcdir@/include               \
 -I@top_srcdir@/src/libemane

gainscenario_CXXFLAGS =    \
 $(libemane_CFLAGS)

gainscenario_SOURCES =  \
 main.cc            

EXTRA_DIST=   \
 gainscenario.xsd

gainscenario_LDADD =                         \
 -L@top_srcdir@/src/libemane/.libs           \
 -L@top_srcdir@/src/libemanexmlparser/.libs  \
 -lemanexmlparser                            \
 $(libemane_LIBS)

