bin_PROGRAMS = emaneinfo

emaneinfo_CPPFLAGS =                     \
 -I@top_srcdir@/include                  \
 -I@top_srcdir@/src/libemane             \
 $(AM_CPPFLAGS)                          \
 $(libemane_CFLAGS)

emaneinfo_LDADD =                            \
 $(libuuid_LIBS)                             \
 $(libxml2_LIBS)                             \
 @top_srcdir@/src/libemane/.libs/libemane.la

emaneinfo_SOURCES =                      \
 configurationloader.cc                  \
 emaneinfo.cc                            \
 manifest.cc                             \
 configurationloader.h                   \
 manifest.h                              \
 plugintype.h                            
