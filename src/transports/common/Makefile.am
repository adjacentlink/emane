lib_LTLIBRARIES = libcommontransport.la

libcommontransport_la_CPPFLAGS= \
 -I@top_srcdir@/include         \
 $(AM_CFLAGS)                   \
 $(libuuid_CFLAGS)

libcommontransport_la_SOURCES= \
 ethernettransport.cc          \
 ethernettransport.h

libcommontransport_la_LDFLAGS= \
 -avoid-version
