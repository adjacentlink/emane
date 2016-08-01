bin_PROGRAMS = emane

emane_CPPFLAGS =                      \
 -I@top_srcdir@/include               \
 -I@top_srcdir@/src/libemane          \
 -I@top_srcdir@/src/libemanexmlparser \
 $(AM_CPPFLAGS)                       \
 $(libemane_CFLAGS)

emane_LDADD =                                                  \
 @top_srcdir@/src/libemane/.libs/libemane.la                   \
 @top_srcdir@/src/libemanexmlparser/.libs/libemanexmlparser.la

emane_SOURCES =  \
 emane.cc        \
 nemdirector.cc  \
 nemdirector.h

