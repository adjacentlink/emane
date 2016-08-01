lib_LTLIBRARIES = libcommontransport.la

libcommontransport_la_CPPFLAGS= \
 -I@top_srcdir@/include         \
 $(AM_CPPFLAGS)                 \
 $(libuuid_CFLAGS)

libcommontransport_la_LIBADD=  \
 $(libuuid_LIBS)

libcommontransport_la_LDFLAGS= \
 $(AM_LDFLAGS)                 \
 -avoid-version

libcommontransport_la_SOURCES= \
 ethernettransport.cc          \
 ethernettransport.h
