lib_LTLIBRARIES = libtdmaeventschedulerradiomodel.la

libtdmaeventschedulerradiomodel_la_CPPFLAGS= \
 -I@top_srcdir@/include              \
 $(ACE_CFLAGS)                       \
 $(AM_CFLAGS)                        \
 $(libuuid_CFLAGS)                   \
 $(libxml2_CFLAGS)

libtdmaeventschedulerradiomodel_la_SOURCES = \
 radiomodel.cc                       \
 eventscheduler.cc                   \
 eventtablepublisher.cc

EXTRA_DIST=                          \
 eventscheduler.h                    \
 eventtablepublisher.h               \
 slotter.h                           \
 slotter.inl                         \
 tdmanem.xml.in                      \
 tdmaradiomodel.xml.in

BUILT_SOURCES =                      \
 tdmanem.xml                         \
 tdmaradiomodel.xml

libtdmaeventschedulerradiomodel_la_LDFLAGS= \
 $(ACE_LIBS)                         \
 $(libxml2_LIBS)                     \
 -L../.libs                          \
 -ltdmabase                          \
 -avoid-version

edit = sed                                \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

tdmanem.xml: tdmaradiomodel.xml.in
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
