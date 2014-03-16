lib_LTLIBRARIES = librfpipemaclayer.la

librfpipemaclayer_la_CPPFLAGS=   \
 -I@top_srcdir@/include          \
 $(AM_CFLAGS)                    \
 $(libuuid_CFLAGS)               \
 $(libxml2_CFLAGS)

librfpipemaclayer_la_SOURCES =   \
 maclayer.cc                     \
 downstreamqueue.cc              \
 pcrmanager.cc                   \
 rfpipemacheadermessage.cc       \
 rfpipemacheader.pb.cc

EXTRA_DIST=                      \
 maclayer.h                      \
 downstreamqueue.h               \
 pcrmanager.h                    \
 rfpipemacheadermessage.h        \
 rfpipenem.xml.in                \
 rfpipemac.xml.in                \
 rfpipepcr.xml.in                \
 rfpipemacheader.proto           

BUILT_SOURCES =                  \
 rfpipenem.xml                   \
 rfpipemac.xml                   \
 rfpipepcr.xml                   \
 rfpipemacheader.pb.cc           \
 rfpipemacheader.pb.h

librfpipemaclayer_la_LIBADD=     \
 $(WIN32_EXTRA_LIBADD)        

librfpipemaclayer_la_LDFLAGS=    \
 $(WIN32_EXTRA_LDFLAGS)          \
 $(libxml2_LIBS)                 \
 -avoid-version

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
