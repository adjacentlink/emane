ACLOCAL_AMFLAGS = -I m4

SUBDIRS= src                            \
         include/emane                  \
         scripts                        \
         scripts/doc                    \
         dtd                            \
         doc                            \
         schema                         \
         test

edit = sed                                               \
       -e 's|@VERSION[@]|$(VERSION)|g'                   \
       -e 's|@DATE_RFC2822[@]|$(DATE_RFC2822)|g'         \
       -e 's|@DISTRIB_CODENAME[@]|$(DISTRIB_CODENAME)|g'

cleantar:
	@(rm -f emane*.tar.gz)

if HAVE_RPMBUILD

rpm: emane.spec dist-gzip
	mkdir -p .rpmbuild/BUILD \
	.rpmbuild/SPECS \
	.rpmbuild/SOURCES \
	.rpmbuild/SRPMS \
	.rpmbuild/RPMS/noarch \
	.rpmbuild/tmp
	cp -f $(PACKAGE)-$(VERSION).tar.gz .rpmbuild/SOURCES
	rpmbuild --clean -ba $(top_srcdir)/$(PACKAGE).spec \
    --define "_topdir $$PWD/.rpmbuild" \
    --define "_tmppath $$PWD/.rpmbuild/tmp" \
    $(RPMBUILD_ARGS)
	@echo "============================================================"
	@echo "RPMs located in .rpmbuild/RPMS and .rpmbuild/SRPMS"

endif

clean-local: cleantar
	-rm -rf .rpmbuild
	-rm -rf .debbuild
	-rm -f emane.spec

emane.spec: emane.spec.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@

if HAVE_DEB

JOBS=`getconf _NPROCESSORS_ONLN`

deb: dist-gzip
	rm -rf .debbuild
	mkdir -p .debbuild
	cp -f $(PACKAGE)-$(VERSION).tar.gz .debbuild
	cd .debbuild && tar xzf $(PACKAGE)-$(VERSION).tar.gz
	$(edit) .debbuild/$(PACKAGE)-$(VERSION)/debian/changelog.in > \
          .debbuild/$(PACKAGE)-$(VERSION)/debian/changelog
	if test -n "$(WITHOUT_PYTHON3)"; \
	   then \
	    cp .debbuild/$(PACKAGE)-$(VERSION)/debian/control.without-python3 \
              .debbuild/$(PACKAGE)-$(VERSION)/debian/control; \
            cp .debbuild/$(PACKAGE)-$(VERSION)/debian/rules.without-python3 \
              .debbuild/$(PACKAGE)-$(VERSION)/debian/rules; \
        fi
	cd .debbuild/$(PACKAGE)-$(VERSION) && dpkg-buildpackage -F -us -uc -j$(JOBS)
	@echo "============================================================"
	@echo "debs located in .debbuild"

endif

EXTRA_DIST =         \
 AUTHORS             \
 COPYING             \
 Copyright           \
 ChangeLog           \
 NEWS                \
 emane.spec.in       \
 SPONSOR             \
 README.md           \
 debian              \
 autogen.sh

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/manifest
	./scripts/emanegenmanifests.sh $(DESTDIR)$(datadir)/$(PACKAGE)/manifest

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/bypassmaclayer.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/bypassphylayer.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/commeffectshim.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/eelgenerator.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/emanephy.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/eventagentmanager.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/eventgeneratormanager.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/gpsdlocationagent.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/ieee80211abgmaclayer.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/nemmanager.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/phyapitestshim.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/rfpipemaclayer.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/tdmaeventschedulerradiomodel.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/timinganalysisshim.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/transportmanager.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/transraw.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/transvirtual.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/universalphylayer.xml

