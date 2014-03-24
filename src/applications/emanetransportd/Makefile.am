bin_PROGRAMS = emanetransportd

emanetransportd_CPPFLAGS =                   \
 -I@top_srcdir@/include                      \
 -I@top_srcdir@/src/libemane                 \
 -I@top_srcdir@/src/libemanexmlparser  

emanetransportd_CXXFLAGS =                   \
 $(libemane_CFLAGS)         

emanetransportd_SOURCES =                    \
 emanetransportd.cc                          \
 transportdirector.cc

EXTRA_DIST  =                                \
 transportdirector.h                         \
 transportdaemon.xml.in

BUILT_SOURCES =                              \
 transportdaemon.xml

emanetransportd_LDADD =                      \
 -L@top_srcdir@/src/libemanexmlparser/.libs  \
 -L@top_srcdir@/src/libemane/.libs           \
 -lemanexmlparser                            \
 $(libemane_LIBS)                            

edit = sed                                   \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

transportdaemon.xml: transportdaemon.xml.in
	$(edit) transportdaemon.xml.in > transportdaemon.xml

clean-local:
	rm -f $(BUILT_SOURCES)
