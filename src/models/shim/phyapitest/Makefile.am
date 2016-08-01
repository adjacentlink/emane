lib_LTLIBRARIES = libphyapitestshim.la

libphyapitestshim_la_CPPFLAGS= \
 -I@top_srcdir@/include        \
 $(AM_CPPFLAGS)                \
 $(libemane_CFLAGS)

libphyapitestshim_la_LIBADD=                 \
 $(libuuid_LIBS)                             \
 @top_srcdir@/src/libemane/.libs/libemane.la

libphyapitestshim_la_LDFLAGS=  \
 $(AM_LDFLAGS)                 \
 -avoid-version

libphyapitestshim_la_SOURCES = \
 shimlayer.cc                  \
 shimlayer.h

EXTRA_DIST =                   \
 phyapitestshim.xml.in

BUILT_SOURCES =                \
 phyapitestshim.xml

edit = sed                     \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

phyapitestshim.xml: phyapitestshim.xml.in
	$(edit) $< > $@

clean-local:
	rm -f $(BUILT_SOURCES)

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/shim/phyapitest
	cp -f phyapitestshim.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/shim/phyapitest

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/shim/phyapitest/phyapitestshim.xml

