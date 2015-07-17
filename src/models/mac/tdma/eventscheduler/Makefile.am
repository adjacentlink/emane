lib_LTLIBRARIES = libtdmaeventschedulerradiomodel.la

libtdmaeventschedulerradiomodel_la_CPPFLAGS= \
 -I@top_srcdir@/include              \
 $(ACE_CFLAGS)                       \
 $(AM_CFLAGS)                        \
 $(libuuid_CFLAGS)                   \
 $(libxml2_CFLAGS)

libtdmaeventschedulerradiomodel_la_SOURCES = \
 radiomodel.cc                       \
 eventscheduler.cc                   \
 eventtablepublisher.cc

EXTRA_DIST=                          \
 eventscheduler.h                    \
 eventtablepublisher.h               \
 slotter.h                           \
 slotter.inl

libtdmaeventschedulerradiomodel_la_LDFLAGS= \
 $(ACE_LIBS)                         \
 $(libxml2_LIBS)                     \
 -L../.libs                          \
 -ltdmabase                          \
 -avoid-version

edit = sed                                \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

