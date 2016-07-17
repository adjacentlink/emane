lib_LTLIBRARIES = libeelloaderpathloss.la

libeelloaderpathloss_la_CPPFLAGS= \
 -I@top_srcdir@/include      \
 $(AM_CPPFLAGS)              \
 $(libemane_CFLAGS)

libeelloaderpathloss_la_LIBADD=              \
 @top_srcdir@/src/libemane/.libs/libemane.la

libeelloaderpathloss_la_LDFLAGS= \
 $(AM_LDFLAGS)                   \
 -avoid-version

libeelloaderpathloss_la_SOURCES= \
 eelloaderpathloss.cc            \
 eelloaderpathloss.h
