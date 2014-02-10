bin_PROGRAMS = emane

emane_CPPFLAGS =                      \
 -I@top_srcdir@/include               \
 -I@top_srcdir@/src/libemane          \
 -I@top_srcdir@/src/libemanexmlparser

emane_CXXFLAGS =    \
 $(libemane_CFLAGS)

emane_SOURCES =  \
 emane.cc        \
 nemdirector.cc 

EXTRA_DIST=   \
 nemdirector.h

emane_LDADD =                                \
 -L@top_srcdir@/src/libemane/.libs            \
 -L@top_srcdir@/src/libemanexmlparser/.libs  \
 -lemanexmlparser                            \
 $(libemane_LIBS)

