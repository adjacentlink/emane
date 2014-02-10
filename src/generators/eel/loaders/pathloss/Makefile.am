lib_LTLIBRARIES = libeelloaderpathloss.la

EXTRA_DIST=                   \
 eelloaderpathloss.h

libeelloaderpathloss_la_CPPFLAGS= \
 -I@top_srcdir@/include      \
 $(ACE_CFLAGS)               \
 $(AM_CFLAGS)                \
 $(EMANE_CFLAGS)               

libeelloaderpathloss_la_SOURCES= \
 eelloaderpathloss.cc

libeelloaderpathloss_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)        

libeelloaderpathloss_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)     \
 $(ACE_LIBS)                \
 -avoid-version

