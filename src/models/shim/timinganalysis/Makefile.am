lib_LTLIBRARIES = libtiminganalysisshim.la

libtiminganalysisshim_la_CPPFLAGS= \
 -I@top_srcdir@/include            \
 $(AM_CPPFLAGS)                    \
 $(libemane_CFLAGS)

libtiminganalysisshim_la_LIBADD=             \
 $(libuuid_LIBS)                             \
 $(protobuf_LIBS)                            \
 @top_srcdir@/src/libemane/.libs/libemane.la

libtiminganalysisshim_la_LDFLAGS= \
 $(AM_LDFLAGS)                    \
 -avoid-version

libtiminganalysisshim_la_SOURCES= \
 shimlayer.cc                     \
 shimheadermessage.cc             \
 shimheadermessage.h              \
 shimlayer.h

nodist_libtiminganalysisshim_la_SOURCES= \
 shimheader.pb.cc                 \
 shimheader.pb.h

EXTRA_DIST =                      \
 shimheader.proto                 \
 timinganalysisshim.xml.in

BUILT_SOURCES =                   \
 timinganalysisshim.xml           \
 $(nodist_libtiminganalysisshim_la_SOURCES)

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
