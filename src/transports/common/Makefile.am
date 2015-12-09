lib_LTLIBRARIES = libcommontransport.la

libcommontransport_la_CPPFLAGS= \
 -I@top_srcdir@/include         \
 $(ACE_CFLAGS)                  \
 $(AM_CFLAGS)                   \
 $(libuuid_CFLAGS)

libcommontransport_la_SOURCES= \
 ethernettransport.cc

EXTRA_DIST=                    \
 ethernettransport.h           

libcommontransport_la_LDFLAGS= \
 $(ACE_LIBS)                   \
 -avoid-version
