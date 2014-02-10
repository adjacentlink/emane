lib_LTLIBRARIES = libgpsdlocationagent.la

libgpsdlocationagent_la_CPPFLAGS= \
 -I@top_srcdir@/include           \
 $(ACE_CFLAGS)                    \
 $(AM_CFLAGS)

libgpsdlocationagent_la_SOURCES= \
 agent.cc

EXTRA_DIST=                      \
 agent.h                         \
 gpsdlocationagent.xml.in 

BUILT_SOURCES =                  \
 gpsdlocationagent.xml

libgpsdlocationagent_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)        

libgpsdlocationagent_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)          \
 $(ACE_LIBS)                     \
 -lutil                          \
 -avoid-version

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

