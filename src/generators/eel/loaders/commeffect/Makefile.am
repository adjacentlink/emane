lib_LTLIBRARIES = libeelloadercommeffect.la

libeelloadercommeffect_la_CPPFLAGS= \
 -I@top_srcdir@/include      \
 $(AM_CPPFLAGS)              \
 $(libemane_CFLAGS)

libeelloadercommeffect_la_LIBADD=            \
 @top_srcdir@/src/libemane/.libs/libemane.la

libeelloadercommeffect_la_LDFLAGS= \
 $(AM_LDFLAGS)                     \
 -avoid-version

libeelloadercommeffect_la_SOURCES= \
 eelloadercommeffect.cc            \
 eelloadercommeffect.h
