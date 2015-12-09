ACLOCAL_AMFLAGS = -I m4

SUBDIRS= src                            \
         include/emane                  \
         scripts                        \
         scripts/doc                    \
         dtd                            \
         doc                            \
         schema                         \
         test

edit = sed                                \
       -e 's|@VERSION[@]|$(VERSION)|g'      

dist-hook:
	find $(distdir) -name "*.pb.*" -delete
	rm -f $(distdir)/include/emane/version.h

cleantar:
	@(rm -f emane*.tar.gz)

RPMARCH=`arch`

if HAVE_RPMBUILD

rpm: emane.spec dist-gzip
	mkdir -p .rpmbuild/BUILD \
	.rpmbuild/SPECS \
	.rpmbuild/SOURCES \
	.rpmbuild/SRPMS \
	.rpmbuild/RPMS/$(RPMARCH) \
	.rpmbuild/RPMS/noarch \
	.rpmbuild/tmp
	cp -f $(PACKAGE)-$(VERSION).tar.gz .rpmbuild/SOURCES
	rpmbuild --clean -ba $(top_srcdir)/$(PACKAGE).spec --target $(RPMARCH) \
    --define "_topdir $$PWD/.rpmbuild" \
    --define "_tmppath $$PWD/.rpmbuild/tmp"
	$(MAKE) -C src/emanesh rpm &&\
    cp src/emanesh/dist/emanesh-$(VERSION)-1.noarch.rpm .rpmbuild/RPMS/noarch && \
    cp src/emanesh/dist/emanesh-$(VERSION)-1.src.rpm .rpmbuild/SRPMS
	@echo "============================================================"
	@echo "RPMs located in .rpmbuild/RPMS and .rpmbuild/SRPMS"

endif

clean-local: cleantar cleandeb
	-rm -rf .rpmbuild
	-rm -rf .debbuild
	-rm -f emane.spec
	-rm -f debian/changelog

emane.spec: emane.spec.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@

if HAVE_DEB

deb: dist-gzip 
	mkdir -p .debbuild
	cp -f $(PACKAGE)-$(VERSION).tar.gz .debbuild
	cd .debbuild && tar xzf $(PACKAGE)-$(VERSION).tar.gz
	$(edit) .debbuild/$(PACKAGE)-$(VERSION)/debian/changelog.in > \
          .debbuild/$(PACKAGE)-$(VERSION)/debian/changelog
	cd .debbuild/$(PACKAGE)-$(VERSION) && dpkg-buildpackage -F -us -uc
	$(MAKE) -C .debbuild/$(PACKAGE)-$(VERSION)/src/emanesh deb && \
         cp .debbuild/$(PACKAGE)-$(VERSION)/src/emanesh/deb_dist/emanesh_$(VERSION)-1_all.deb .debbuild
	@echo "============================================================"
	@echo "debs located in .debbuild"
cleandeb:
	dh_clean
else

cleandeb:

endif

EXTRA_DIST =         \
 AUTHORS             \
 COPYING             \
 Copyright           \
 ChangeLog           \
 NEWS                \
 emane.spec.in       \
 PROBLEM-REPORT-FORM \
 SPONSOR             \
 README.md           \
 debian              \
 autogen.sh

DISTCLEANFILES = .rpmbuild

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/manifest
	./scripts/emanegenmanifests.sh $(DESTDIR)$(datadir)/$(PACKAGE)/manifest

uninstall-local:
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/bypassmaclayer.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/bypassphylayer.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/commeffectshim.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/eelgenerator.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/gpsdlocationagent.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/ieee80211abgmaclayer.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/phyapitestshim.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/rfpipemaclayer.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/timinganalysisshim.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/transraw.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/transvirtual.xml
	rm -f $(DESTDIR)$(datadir)/$(PACKAGE)/manifest/universalphylayer.xml

