lib_LTLIBRARIES = libbypassmaclayer.la

libbypassmaclayer_la_CPPFLAGS= \
 -I@top_srcdir@/include        \
 $(ACE_CFLAGS)                 \
 $(libuuid_CFLAGS)             \
 $(AM_CFLAGS)

libbypassmaclayer_la_SOURCES = \
 maclayer.cc

EXTRA_DIST =                   \
 maclayer.h                    \
 bypassnem.xml.in              \
 bypassmac.xml.in

BUILT_SOURCES =                \
 bypassnem.xml                 \
 bypassmac.xml                   

libbypassmaclayer_la_LIBADD=   \
 $(WIN32_EXTRA_LIBADD)        

libbypassmaclayer_la_LDFLAGS=  \
 $(WIN32_EXTRA_LDFLAGS)        \
 $(ACE_LIBS)                   \
 -avoid-version

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
