SCHEMAS=              \
 manifest.xsd         \
 tdmabasemodelpcr.xsd

EXTRA_DIST  = \
 $(SCHEMAS)

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/schema/
	cp -f $(SCHEMAS) $(DESTDIR)$(datadir)/$(PACKAGE)/schema

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/schema/*
