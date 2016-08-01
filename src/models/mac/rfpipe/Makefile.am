lib_LTLIBRARIES = librfpipemaclayer.la

librfpipemaclayer_la_CPPFLAGS=   \
 -I@top_srcdir@/include          \
 $(AM_CPPFLAGS)                  \
 $(libemane_CFLAGS)

librfpipemaclayer_la_LIBADD=                  \
 $(libuuid_LIBS)                              \
 $(libxml2_LIBS)                              \
 $(protobuf_LIBS)                             \
 @top_srcdir@/src/libemane/.libs/libemane.la

librfpipemaclayer_la_LDFLAGS=    \
 $(AM_LDFLAGS)                   \
 -avoid-version

librfpipemaclayer_la_SOURCES =   \
 maclayer.cc                     \
 downstreamqueue.cc              \
 pcrmanager.cc                   \
 rfpipemacheadermessage.cc       \
 maclayer.h                      \
 downstreamqueue.h               \
 pcrmanager.h                    \
 rfpipemacheadermessage.h

nodist_librfpipemaclayer_la_SOURCES =   \
 rfpipemacheader.pb.cc           \
 rfpipemacheader.pb.h

EXTRA_DIST=                      \
 rfpipenem.xml.in                \
 rfpipemac.xml.in                \
 rfpipepcr.xml.in                \
 rfpipemacheader.proto           

BUILT_SOURCES =                  \
 rfpipenem.xml                   \
 rfpipemac.xml                   \
 rfpipepcr.xml                   \
 $(nodist_librfpipemaclayer_la_SOURCES)

edit = sed                                \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

rfpipenem.xml: rfpipenem.xml.in
	$(edit) $< > $@

rfpipemac.xml: rfpipemac.xml.in
	$(edit) $< > $@

rfpipepcr.xml: rfpipepcr.xml.in
	$(edit) $< > $@

rfpipemacheader.pb.cc rfpipemacheader.pb.h: rfpipemacheader.proto
	protoc -I=. --cpp_out=. $<

clean-local:
	rm -f $(BUILT_SOURCES)

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/rfpipe
	cp -f rfpipenem.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/rfpipe
	cp -f rfpipemac.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/rfpipe
	cp -f rfpipepcr.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/rfpipe

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/rfpipe/rfpipenem.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/rfpipe/rfpipemac.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/rfpipe/rfpipepcr.xml
