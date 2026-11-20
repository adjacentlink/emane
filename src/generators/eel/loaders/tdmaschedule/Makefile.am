lib_LTLIBRARIES = libeelloadertdmaschedule.la

libeelloadertdmaschedule_la_CPPFLAGS= \
 -I@top_srcdir@/include      \
 $(AM_CPPFLAGS)              \
 $(libemane_CFLAGS)

libeelloadertdmaschedule_la_LIBADD=        \
 @top_srcdir@/src/libemane/.libs/libemane.la

libeelloadertdmaschedule_la_LDFLAGS= \
 $(AM_LDFLAGS)                         \
 -avoid-version

libeelloadertdmaschedule_la_SOURCES= \
 eelloadertdmaschedule.cc            \
 eelloadertdmaschedule.h             \
 schedulehelper.h
