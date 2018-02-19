lib_LTLIBRARIES = libeelloaderfadingselection.la

libeelloaderfadingselection_la_CPPFLAGS= \
 -I@top_srcdir@/include      \
 $(AM_CPPFLAGS)              \
 $(libemane_CFLAGS)

libeelloaderfadingselection_la_LIBADD=              \
 @top_srcdir@/src/libemane/.libs/libemane.la

libeelloaderfadingselection_la_LDFLAGS= \
 $(AM_LDFLAGS)                   \
 -avoid-version

libeelloaderfadingselection_la_SOURCES= \
 eelloaderfadingselection.cc            \
 eelloaderfadingselection.h
