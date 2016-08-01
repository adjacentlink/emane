lib_LTLIBRARIES = libeelloaderantennaprofile.la

libeelloaderantennaprofile_la_CPPFLAGS= \
 -I@top_srcdir@/include      \
 $(AM_CPPFLAGS)              \
 $(libemane_CFLAGS)

libeelloaderantennaprofile_la_LIBADD=        \
 @top_srcdir@/src/libemane/.libs/libemane.la

libeelloaderantennaprofile_la_LDFLAGS= \
 $(AM_LDFLAGS)                         \
 -avoid-version

libeelloaderantennaprofile_la_SOURCES= \
 eelloaderantennaprofile.cc            \
 eelloaderantennaprofile.h
