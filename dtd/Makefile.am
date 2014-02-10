DTDS=                \
 attrs.ent           \
 eventagent.dtd      \
 eventdaemon.dtd     \
 eventgenerator.dtd  \
 eventservice.dtd    \
 mac.dtd             \
 nemcontents.dtd     \
 nem.dtd             \
 param.dtd           \
 phy.dtd             \
 platform.dtd        \
 shim.dtd            \
 transportdaemon.dtd \
 transport.dtd       \
 rfpipepcr.dtd       \
 ieee80211pcr.dtd    \
 antennaprofile.dtd  \
 commeffectfilters.dtd

EXTRA_DIST  = \
 $(DTDS)

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/dtd
	cp -f  $(DTDS) $(DESTDIR)$(datadir)/$(PACKAGE)/dtd

uninstall-local:
	rm -f  $(DESTDIR)$(datadir)/$(PACKAGE)/dtd/*
