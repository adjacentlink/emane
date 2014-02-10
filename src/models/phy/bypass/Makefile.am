lib_LTLIBRARIES = libbypassphylayer.la

libbypassphylayer_la_CPPFLAGS= \
 -I@top_srcdir@/include        \
 $(ACE_CFLAGS)                 \
 $(AM_CFLAGS)

libbypassphylayer_la_SOURCES = \
 phylayer.cc 

EXTRA_DIST =                   \
 phylayer.h                    \
 bypassphy.xml.in

BUILT_SOURCES =                \
 bypassphy.xml                   

libbypassphylayer_la_LIBADD=   \
 $(WIN32_EXTRA_LIBADD)        

libbypassphylayer_la_LDFLAGS=  \
 $(WIN32_EXTRA_LDFLAGS)        \
 $(ACE_LIBS)                   \
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

