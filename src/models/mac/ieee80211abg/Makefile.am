lib_LTLIBRARIES = libieee80211abgmaclayer.la

libieee80211abgmaclayer_la_CPPFLAGS= \
 -I@top_srcdir@/include              \
 $(AM_CFLAGS)                        \
 $(libuuid_CFLAGS)                   \
 $(libxml2_CFLAGS)

libieee80211abgmaclayer_la_SOURCES = \
 maclayer.cc                         \
 modetimingparameters.cc             \
 neighborentry.cc                    \
 neighbor2hopentry.cc                \
 neighbormanager.cc                  \
 macconfig.cc                        \
 pcrmanager.cc                       \
 macstatistics.cc                    \
 downstreamqueue.cc                  \
 transmissiontxstate.cc              \
 idletxstate.cc                      \
 broadcasttxstate.cc                 \
 broadcastpretxstate.cc              \
 broadcastposttxstate.cc             \
 unicasttxstate.cc                   \
 unicastpretxstate.cc                \
 unicastposttxstate.cc               \
 unicastrtsctstxstate.cc             \
 unicastrtsctspretxstate.cc          \
 unicastrtsctsposttxstate.cc         \
 onehopneighborsevent.cc             \
 collisiontable.cc                   \
 macheaderparams.cc                  \
 wmmmanager.cc                       \
 ieee80211abgmacheadermessage.cc     \
 macheaderparamsformatter.cc         \
 configurationvalidator.cc           \
 downstreamqueue.h                   \
 downstreamqueueentry.h              \
 accesscategory.h                    \
 neighborentry.h                     \
 neighbor2hopentry.h                 \
 neighbormanager.h                   \
 macconfig.h                         \
 pcrmanager.h                        \
 macheaderparams.h                   \
 maclayer.h                          \
 macstatistics.h                     \
 idletxstate.h                       \
 modetimingparameters.h              \
 broadcasttxstate.h                  \
 broadcastpretxstate.h               \
 broadcastposttxstate.h              \
 transmissiontxstate.h               \
 unicasttxstate.h                    \
 unicastpretxstate.h                 \
 unicastposttxstate.h                \
 unicastrtsctstxstate.h              \
 unicastrtsctspretxstate.h           \
 unicastrtsctsposttxstate.h          \
 utils.h                             \
 neighbortype.h                      \
 neighbortype.inl                    \
 onehopneighborsevent.h              \
 collisiontable.h                    \
 msgtypes.h                          \
 wmmmanager.h                        \
 configurationvalidator.h            \
 ieee80211abgmacheadermessage.h      \
 macheaderparamsformatter.h

nodist_libieee80211abgmaclayer_la_SOURCES = \
 ieee80211abgmacheader.pb.cc         \
 onehopneighborsevent.pb.cc          \
 ieee80211abgmacheader.pb.h          \
 onehopneighborsevent.pb.h

EXTRA_DIST=                          \
 ieee80211abgmacheader.proto         \
 onehopneighborsevent.proto          \
 ieee80211abgnem.xml.in              \
 ieee80211abgmac.xml.in              \
 ieee80211pcr.xml.in

BUILT_SOURCES =                      \
 ieee80211abgnem.xml                 \
 ieee80211abgmac.xml                 \
 ieee80211pcr.xml                    \
 $(nodist_libieee80211abgmaclayer_la_SOURCES)

libieee80211abgmaclayer_la_LDFLAGS=  \
 $(libxml2_LIBS)                     \
 -avoid-version

ieee80211abgmacheader.pb.cc ieee80211abgmacheader.pb.h: ieee80211abgmacheader.proto
	protoc -I=. --cpp_out=. $<

onehopneighborsevent.pb.cc onehopneighborsevent.pb.h: onehopneighborsevent.proto
	protoc -I=. --cpp_out=. $<

edit = sed                                \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

ieee80211abgnem.xml: ieee80211abgnem.xml.in
	$(edit) $< > $@

ieee80211abgmac.xml: ieee80211abgmac.xml.in
	$(edit) $< > $@

ieee80211pcr.xml: ieee80211pcr.xml.in
	$(edit) $< > $@

clean-local:
	rm -f $(BUILT_SOURCES)

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/ieee80211abg
	cp -f ieee80211abgnem.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/ieee80211abg
	cp -f ieee80211abgmac.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/ieee80211abg
	cp -f ieee80211pcr.xml $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/ieee80211abg

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/ieee80211abg/ieee80211abgnem.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/ieee80211abg/ieee80211abgmac.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/xml/models/mac/ieee80211abg/ieee80211pcr.xml
