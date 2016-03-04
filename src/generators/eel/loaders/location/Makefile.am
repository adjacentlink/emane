lib_LTLIBRARIES = libeelloaderlocation.la

EXTRA_DIST=                   \
 eelloaderlocation.h          \
 locationentry.h              \
 locationentry.inl

libeelloaderlocation_la_CPPFLAGS= \
 -I@top_srcdir@/include      \
 $(AM_CFLAGS)                \
 $(EMANE_CFLAGS)               

libeelloaderlocation_la_SOURCES= \
 eelloaderlocation.cc

libeelloaderlocation_la_LDFLAGS= \
 -avoid-version
