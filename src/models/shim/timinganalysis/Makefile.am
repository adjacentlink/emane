lib_LTLIBRARIES = libtiminganalysisshim.la

libtiminganalysisshim_la_CPPFLAGS= \
 -I@top_srcdir@/include
 $(AM_CFLAGS)

libtiminganalysisshim_la_SOURCES= \
 shimlayer.cc                     \
 shimheader.pb.cc                 \
 shimheadermessage.cc

EXTRA_DIST =                      \
 shimheadermessage.h              \
 shimlayer.h                      \
 shimheader.proto                 \
 timinganalysisshim.xml.in

BUILT_SOURCES =                   \
 timinganalysisshim.xml           \
 shimheader.pb.cc                 \
 shimheader.pb.h

libtiminganalysisshim_la_LIBADD=  \
 $(WIN32_EXTRA_LIBADD)        

libtiminganalysisshim_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)           \
 -avoid-version

edit = sed                        \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

timinganalysisshim.xml: timinganalysisshim.xml.in
	$(edit) $< > $@

shimheader.pb.cc shimheader.pb.h: shimheader.proto
	protoc -I=. --cpp_out=. $<

clean-local:
	rm -f $(BUILT_SOURCES)

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/shim/timinganalysis
	cp -f timinganalysisshim.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/shim/timinganalysis

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/shim/timinganalysis/timinganalysisshim.xml
