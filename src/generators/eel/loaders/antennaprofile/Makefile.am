lib_LTLIBRARIES = libeelloaderantennaprofile.la

EXTRA_DIST=                   \
 eelloaderantennaprofile.h

libeelloaderantennaprofile_la_CPPFLAGS= \
 -I@top_srcdir@/include      \
 $(ACE_CFLAGS)               \
 $(AM_CFLAGS)                

libeelloaderantennaprofile_la_SOURCES= \
 eelloaderantennaprofile.cc

libeelloaderantennaprofile_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)        

libeelloaderantennaprofile_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)     \
 $(ACE_LIBS)                \
 -avoid-version

