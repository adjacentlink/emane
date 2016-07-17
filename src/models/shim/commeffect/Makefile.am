lib_LTLIBRARIES = libcommeffectshim.la

libcommeffectshim_la_CPPFLAGS= \
 -I@top_srcdir@/include        \
 $(AM_CPPFLAGS)                \
 $(libemane_CFLAGS)

libcommeffectshim_la_LIBADD=                 \
 $(libuuid_LIBS)                             \
 $(libxml2_LIBS)                             \
 $(protobuf_LIBS)                            \
 @top_srcdir@/src/libemane/.libs/libemane.la

libcommeffectshim_la_LDFLAGS=  \
 $(AM_LDFLAGS)                 \
 -avoid-version

libcommeffectshim_la_SOURCES=  \
 ethernetprotocolipv4rule.cc   \
 filter.cc                     \
 filterreader.cc               \
 ipprotocolsimplerule.cc       \
 ipprotocoludprule.cc          \
 profilemanager.cc             \
 shim.cc                       \
 shimheader.cc                 \
 target.cc                     \
 ethernetprotocolipv4rule.h    \
 ethernetprotocolrule.h        \
 filter.h                      \
 filterreader.h                \
 ipprotocolrule.h              \
 ipprotocolsimplerule.h        \
 ipprotocoludprule.h           \
 profilemanager.h              \
 rule.h                        \
 shim.h                        \
 shimheader.h                  \
 shimheader.inl                \
 target.h

nodist_libcommeffectshim_la_SOURCES= \
 commeffectshimheader.pb.cc    \
 commeffectshimheader.pb.h

EXTRA_DIST=                    \
 commeffectshimheader.proto    \
 commeffectfilters.xml.in      \
 commeffectnem.xml.in          \
 commeffectshim.xml.in

BUILT_SOURCES =                \
 commeffectfilters.xml         \
 commeffectnem.xml             \
 commeffectshim.xml            \
 $(nodist_libcommeffectshim_la_SOURCES)

edit = sed                                \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

commeffectnem.xml: commeffectnem.xml.in
	$(edit) commeffectnem.xml.in > commeffectnem.xml

commeffectshim.xml: commeffectshim.xml.in
	$(edit) commeffectshim.xml.in > commeffectshim.xml

commeffectfilters.xml: commeffectfilters.xml.in
	$(edit) commeffectfilters.xml.in > commeffectfilters.xml

commeffectshimheader.pb.cc commeffectshimheader.pb.h: commeffectshimheader.proto
	protoc -I=. --cpp_out=. $<

clean-local:
	rm -f $(BUILT_SOURCES)

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/shim/commeffect
	cp -f commeffectnem.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/shim/commeffect
	cp -f commeffectshim.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/shim/commeffect
	cp -f commeffectfilters.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/shim/commeffect

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/shim/commeffect/commeffectnem.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/shim/commeffect/commeffectshim.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/shim/commeffect/commeffectfilters.xml
