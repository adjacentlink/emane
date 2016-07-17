bin_PROGRAMS = emaneeventd

emaneeventd_CPPFLAGS =                       \
 -I@top_srcdir@/include                      \
 -I@top_srcdir@/src/libemane                 \
 -I@top_srcdir@/src/libemanexmlparser  

emaneeventd_CXXFLAGS =                       \
 $(libemane_CFLAGS)     

emaneeventd_SOURCES =                        \
 emaneeventd.cc                              \
 eventagentdirector.cc                       \
 eventagentdirector.h

EXTRA_DIST  =                                \
 eventdaemon.xml.in

BUILT_SOURCES =                              \
 eventdaemon.xml

emaneeventd_LDADD =                          \
 -L@top_srcdir@/src/libemanexmlparser/.libs  \
 -L@top_srcdir@/src/libemane/.libs           \
 -lemanexmlparser                            \
 $(libemane_LIBS)                            

edit = sed                                   \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

eventdaemon.xml: eventdaemon.xml.in
	$(edit) eventdaemon.xml.in > eventdaemon.xml

clean-local:
	rm -f $(BUILT_SOURCES)
