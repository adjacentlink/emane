lib_LTLIBRARIES = libtransraw.la

libtransraw_la_CPPFLAGS= \
 -I@top_srcdir@/include  \
 -I../common             \
 $(AM_CPPFLAGS)          \
 $(libemane_CFLAGS)

libtransraw_la_LIBADD=                       \
 $(pcap_LIBS)                                \
 $(libuuid_LIBS)                             \
 @top_srcdir@/src/libemane/.libs/libemane.la \
 ../common/.libs/libcommontransport.la

libtransraw_la_LDFLAGS= \
 $(AM_LDFLAGS)          \
 -avoid-version

libtransraw_la_SOURCES= \
 rawtransport.cc        \
 rawtransport.h

EXTRA_DIST=             \
 transraw.xml.in

BUILT_SOURCES =         \
 transraw.xml

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
