bin_PROGRAMS = emaneeventservice

emaneeventservice_CPPFLAGS =                  \
 -I@top_srcdir@/include                       \
 -I@top_srcdir@/src/libemane                  \
 -I@top_srcdir@/src/libemanexmlparser         

emaneeventservice_CXXFLAGS =                  \
 $(libemane_CFLAGS)            

emaneeventservice_SOURCES =                   \
 emaneeventserver.cc                          \
 eventdirector.cc                             \
 eventdirector.h

EXTRA_DIST=                                   \
 eventservice.xml.in

BUILT_SOURCES =                               \
 eventservice.xml

emaneeventservice_LDADD =                     \
 -L@top_srcdir@/src/libemanexmlparser/.libs   \
 -L@top_srcdir@/src/libemane/.libs            \
 -lemanexmlparser                             \
 $(libemane_LIBS)


edit = sed                                    \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

eventservice.xml: eventservice.xml.in
	$(edit) eventservice.xml.in > eventservice.xml

clean-local:
	rm -f $(BUILT_SOURCES)
