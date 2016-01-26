lib_LTLIBRARIES = libtransraw.la

libtransraw_la_CPPFLAGS= \
 -I@top_srcdir@/include  \
 -I../common             \
 $(pcap_CFLAGS)          \
 $(libuuid_CFLAGS)       \
 $(AM_CFLAGS)

libtransraw_la_SOURCES= \
 rawtransport.cc        

EXTRA_DIST=             \
 rawtransport.h         \
 transraw.xml.in

BUILT_SOURCES =         \
 transraw.xml

libtransraw_la_LDFLAGS= \
 $(pcap_LIBS)           \
 -L../common            \
 -lcommontransport      \
 -avoid-version

edit = sed                     \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

transraw.xml: transraw.xml.in
	$(edit) $< > $@

clean-local:
	rm -f $(BUILT_SOURCES)

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/transports/raw
	cp -f transraw.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/transports/raw

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/transports/raw/transraw.xml
