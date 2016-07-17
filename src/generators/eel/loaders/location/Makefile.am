lib_LTLIBRARIES = libeelloaderlocation.la

libeelloaderlocation_la_CPPFLAGS= \
 -I@top_srcdir@/include      \
 $(AM_CPPFLAGS)              \
 $(libemane_CFLAGS)

libeelloaderlocation_la_LIBADD=              \
 @top_srcdir@/src/libemane/.libs/libemane.la

libeelloaderlocation_la_LDFLAGS= \
 $(AM_LDFLAGS)                   \
 -avoid-version

libeelloaderlocation_la_SOURCES= \
 eelloaderlocation.cc            \
 eelloaderlocation.h             \
 locationentry.h                 \
 locationentry.inl
