bin_PROGRAMS = emaneinfo

emaneinfo_CPPFLAGS =                     \
 -I@top_srcdir@/include                  \
 -I@top_srcdir@/src/libemane               

emaneinfo_CXXFLAGS =                     \
 $(libemane_CFLAGS)

emaneinfo_SOURCES =                      \
 configurationloader.cc                  \
 emaneinfo.cc                            \
 manifest.cc                             

EXTRA_DIST=                              \
 configurationloader.h                   \
 manifest.h                              \
 plugintype.h                            

emaneinfo_LDADD =                        \
 -L@top_srcdir@/src/libemane/.libs       \
 $(libemane_LIBS)
