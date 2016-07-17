lib_LTLIBRARIES = libtransvirtual.la

libtransvirtual_la_CPPFLAGS= \
 -I@top_srcdir@/include      \
 -I../common                 \
 $(AM_CFLAGS)                \
 $(libuuid_CFLAGS)           

libtransvirtual_la_SOURCES= \
 virtualtransport.cc        \
 tuntap.cc                  \
 virtualtransport.h         \
 tuntap.h

EXTRA_DIST=                 \
 transvirtual.xml.in        

BUILT_SOURCES =             \
 transvirtual.xml

libtransvirtual_la_LDFLAGS= \
 -L../common                \
 -lcommontransport          \
 -avoid-version

edit = sed                     \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

transvirtual.xml: transvirtual.xml.in
	$(edit) $< > $@

clean-local:
	rm -f $(BUILT_SOURCES)

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/transports/virtual
	cp -f transvirtual.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/transports/virtual

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/transports/virtual/transvirtual.xml
