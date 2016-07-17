bin_PROGRAMS = emaneeventd

emaneeventd_CPPFLAGS =                       \
 -I@top_srcdir@/include                      \
 -I@top_srcdir@/src/libemane                 \
 -I@top_srcdir@/src/libemanexmlparser        \
 $(AM_CPPFLAGS)                              \
 $(libemane_CFLAGS)

emaneeventd_LDADD =                                            \
 @top_srcdir@/src/libemane/.libs/libemane.la                   \
 @top_srcdir@/src/libemanexmlparser/.libs/libemanexmlparser.la

emaneeventd_SOURCES =                        \
 emaneeventd.cc                              \
 eventagentdirector.cc                       \
 eventagentdirector.h

EXTRA_DIST  =                                \
 eventdaemon.xml.in

BUILT_SOURCES =                              \
 eventdaemon.xml

edit = sed                                   \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

eventdaemon.xml: eventdaemon.xml.in
	$(edit) eventdaemon.xml.in > eventdaemon.xml

clean-local:
	rm -f $(BUILT_SOURCES)
