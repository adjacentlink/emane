lib_LTLIBRARIES = libeelloaderpathlossex.la

libeelloaderpathlossex_la_CPPFLAGS= \
 -I@top_srcdir@/include      \
 $(AM_CPPFLAGS)              \
 $(libemane_CFLAGS)

libeelloaderpathlossex_la_LIBADD=              \
 @top_srcdir@/src/libemane/.libs/libemane.la

libeelloaderpathlossex_la_LDFLAGS= \
 $(AM_LDFLAGS)                   \
 -avoid-version

libeelloaderpathlossex_la_SOURCES= \
 eelloaderpathlossex.cc            \
 eelloaderpathlossex.h
