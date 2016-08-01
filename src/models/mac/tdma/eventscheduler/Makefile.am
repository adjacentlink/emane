lib_LTLIBRARIES = libtdmaeventschedulerradiomodel.la

libtdmaeventschedulerradiomodel_la_CPPFLAGS= \
 -I@top_srcdir@/include              \
 $(AM_CPPFLAGS)                      \
 $(libemane_CFLAGS)

libtdmaeventschedulerradiomodel_la_LIBADD=   \
 $(libuuid_LIBS)                             \
 $(libxml2_LIBS)                             \
 @top_srcdir@/src/libemane/.libs/libemane.la \
 ../.libs/libtdmabase.la

libtdmaeventschedulerradiomodel_la_LDFLAGS= \
 $(AM_LDFLAGS)                       \
 -avoid-version

libtdmaeventschedulerradiomodel_la_SOURCES = \
 radiomodel.cc                       \
 eventscheduler.cc                   \
 eventtablepublisher.cc              \
 eventscheduler.h                    \
 eventtablepublisher.h               \
 slotter.h                           \
 slotter.inl

EXTRA_DIST=                          \
 tdmanem.xml.in                      \
 tdmaradiomodel.xml.in

BUILT_SOURCES =                      \
 tdmanem.xml                         \
 tdmaradiomodel.xml

edit = sed                                \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

tdmanem.xml: tdmanem.xml.in
	$(edit) $< > $@

tdmaradiomodel.xml: tdmaradiomodel.xml.in
	$(edit) $< > $@

clean-local:
	rm -f $(BUILT_SOURCES)

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/tdmaeventscheduler
	cp -f tdmanem.xml $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/tdmaeventscheduler
	cp -f tdmaradiomodel.xml $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/tdmaeventscheduler
	cp -f ../tdmabasemodelpcr.xml $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/tdmaeventscheduler

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/tdmaeventscheduler/tdmanem.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/tdmaeventscheduler/tdmaradiomodel.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/tdmaeventscheduler/tdmabasemodelpcr.xml
