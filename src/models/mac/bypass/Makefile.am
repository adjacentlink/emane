lib_LTLIBRARIES = libbypassmaclayer.la

libbypassmaclayer_la_CPPFLAGS= \
 -I@top_srcdir@/include        \
 $(AM_CPPFLAGS)                \
 $(libemane_CFLAGS)

libbypassmaclayer_la_LIBADD=                 \
 $(libuuid_LIBS)                             \
 @top_srcdir@/src/libemane/.libs/libemane.la

libbypassmaclayer_la_LDFLAGS=  \
 $(AM_LDFLAGS)                 \
 -avoid-version

libbypassmaclayer_la_SOURCES = \
 maclayer.cc                   \
 maclayer.h

EXTRA_DIST =                   \
 bypassnem.xml.in              \
 bypassmac.xml.in

BUILT_SOURCES =                \
 bypassnem.xml                 \
 bypassmac.xml                   

edit = sed                     \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

bypassnem.xml:	bypassnem.xml.in
	$(edit) $< > $@

bypassmac.xml:	bypassmac.xml.in
	$(edit) $< > $@

clean-local:
	rm -f $(BUILT_SOURCES)

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/bypass
	cp -f bypassnem.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/bypass
	cp -f bypassmac.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/bypass

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/bypass/bypassnem.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/bypass/bypassmac.xml
