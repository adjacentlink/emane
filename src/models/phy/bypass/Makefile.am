lib_LTLIBRARIES = libbypassphylayer.la

libbypassphylayer_la_CPPFLAGS= \
 -I@top_srcdir@/include        \
 $(libuuid_CFLAGS)             \
 $(AM_CFLAGS)

libbypassphylayer_la_SOURCES = \
 phylayer.cc                   \
 phylayer.h

EXTRA_DIST =                   \
 bypassphy.xml.in

BUILT_SOURCES =                \
 bypassphy.xml                   

libbypassphylayer_la_LDFLAGS=  \
 -avoid-version

edit = sed                     \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

bypassphy.xml: bypassphy.xml.in
	$(edit) $< > $@

clean-local:
	rm -f $(BUILT_SOURCES)

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/phy/bypass
	cp -f bypassphy.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/phy/bypass

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/phy/bypass/bypassphy.xml

