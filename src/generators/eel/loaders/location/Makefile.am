lib_LTLIBRARIES = libeelloaderlocation.la

EXTRA_DIST=                   \
 eelloaderlocation.h          \
 locationentry.h              \
 locationentry.inl

libeelloaderlocation_la_CPPFLAGS= \
 -I@top_srcdir@/include      \
 $(ACE_CFLAGS)               \
 $(AM_CFLAGS)                \
 $(EMANE_CFLAGS)               

libeelloaderlocation_la_SOURCES= \
 eelloaderlocation.cc

libeelloaderlocation_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)        

libeelloaderlocation_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)     \
 $(ACE_LIBS)                \
 -avoid-version
