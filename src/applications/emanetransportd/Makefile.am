bin_PROGRAMS = emanetransportd

emanetransportd_CPPFLAGS =                   \
 -I@top_srcdir@/include                      \
 -I@top_srcdir@/src/libemane                 \
 -I@top_srcdir@/src/libemanexmlparser        \
 $(AM_CPPFLAGS)                              \
 $(libemane_CFLAGS)

emanetransportd_LDADD =                                        \
 @top_srcdir@/src/libemane/.libs/libemane.la                   \
 @top_srcdir@/src/libemanexmlparser/.libs/libemanexmlparser.la

emanetransportd_SOURCES =                    \
 emanetransportd.cc                          \
 transportdirector.cc                        \
 transportdirector.h

EXTRA_DIST  =                                \
 transportdaemon.xml.in

BUILT_SOURCES =                              \
 transportdaemon.xml

edit = sed                                   \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

transportdaemon.xml: transportdaemon.xml.in
	$(edit) transportdaemon.xml.in > transportdaemon.xml

clean-local:
	rm -f $(BUILT_SOURCES)
