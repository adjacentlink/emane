SUBDIRS=loaders

lib_LTLIBRARIES = libeelgenerator.la

libeelgenerator_la_CPPFLAGS= \
 -I@top_srcdir@/include      \
 $(AM_CFLAGS)                \
 $(EMANE_CFLAGS)               

libeelgenerator_la_SOURCES= \
 eeleventgenerator.cc       \
 eelinputparser.cc          \
 eelloaderpluginfactory.cc  \
 eeleventgenerator.h        \
 eelinputparser.h           \
 eelloaderpluginfactory.h

EXTRA_DIST=                 \
 eelgenerator.xml.in

BUILT_SOURCES =                  \
 eelgenerator.xml

libeelgenerator_la_LDFLAGS= \
 -avoid-version

edit = sed                     \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

eelgenerator.xml: eelgenerator.xml.in
	$(edit) $< > $@

clean-local:
	rm -f $(BUILT_SOURCES)

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/generators/eel
	cp -f eelgenerator.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/generators/eel

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/generators/eel/eelgenerator.xml


