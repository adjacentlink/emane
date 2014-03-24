lib_LTLIBRARIES = libtransvirtual.la

libtransvirtual_la_CPPFLAGS= \
 -I@top_srcdir@/include      \
 -I../common                 \
 $(ACE_CFLAGS)               \
 $(AM_CFLAGS)                \
 $(libuuid_CFLAGS)           

libtransvirtual_la_SOURCES= \
 virtualtransport.cc        \
 tuntap.cc                  \
 ../common/ethernettransport.cc

EXTRA_DIST=                 \
 virtualtransport.h         \
 tuntap.h                   \
 ../common/ethernettransport.h \
 transvirtual.xml.in        

BUILT_SOURCES =             \
 transvirtual.xml

libtransvirtual_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)        

libtransvirtual_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)     \
 $(ACE_LIBS)                \
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
