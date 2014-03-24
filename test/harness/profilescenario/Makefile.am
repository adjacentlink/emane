noinst_PROGRAMS = profilescenario

profilescenario_CPPFLAGS =    \
 -I@top_srcdir@/include       \
 -I@top_srcdir@/src/libemane

profilescenario_CXXFLAGS =    \
 $(libemane_CFLAGS)

profilescenario_SOURCES =  \
 main.cc            

EXTRA_DIST=   \
 profilescenario.xsd

profilescenario_LDADD =                         \
 -L@top_srcdir@/src/libemane/.libs           \
 -L@top_srcdir@/src/libemanexmlparser/.libs  \
 -lemanexmlparser                            \
 $(libemane_LIBS)
