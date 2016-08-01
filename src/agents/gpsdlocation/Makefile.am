lib_LTLIBRARIES = libgpsdlocationagent.la

libgpsdlocationagent_la_CPPFLAGS= \
 -I@top_srcdir@/include           \
 $(AM_CPPFLAGS)                   \
 $(libemane_CFLAGS)

libgpsdlocationagent_la_LIBADD=              \
 -lutil                                      \
 @top_srcdir@/src/libemane/.libs/libemane.la

libgpsdlocationagent_la_LDFLAGS= \
 $(AM_LDFLAGS)                   \
 -avoid-version

libgpsdlocationagent_la_SOURCES= \
 agent.cc                        \
 agent.h

EXTRA_DIST=                      \
 gpsdlocationagent.xml.in 

BUILT_SOURCES =                  \
 gpsdlocationagent.xml

edit = sed                     \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

gpsdlocationagent.xml: gpsdlocationagent.xml.in
	$(edit) $< > $@

clean-local:
	rm -f $(BUILT_SOURCES)

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/agents/gpsdlocation
	cp -f gpsdlocationagent.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/agents/gpsdlocation

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/agents/gpsdlocation/gpsdlocationagent.xml

