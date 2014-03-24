noinst_PROGRAMS = phyupstreamscenario

phyupstreamscenario_CPPFLAGS =        \
 -I@top_srcdir@/include               \
 -I@top_srcdir@/src/libemane

phyupstreamscenario_CXXFLAGS =        \
 $(libemane_CFLAGS)

phyupstreamscenario_SOURCES =         \
 main.cc            

EXTRA_DIST=                           \
 phyupstreamscenario.xsd

phyupstreamscenario_LDADD =                  \
 -L@top_srcdir@/src/libemane/.libs           \
 -L@top_srcdir@/src/libemanexmlparser/.libs  \
 -lemanexmlparser                            \
 $(libemane_LIBS)

