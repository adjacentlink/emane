man_MANS =                  \
 emane.1                    \
 emanetransportd.1          \
 emaneeventd.1              \
 emaneeventservice.1

EXTRA_DIST=                 \
 emane.doxygen.in           \
 emane.1.in                 \
 emaneeventd.1.in           \
 emanetransportd.1.in       \
 emaneeventservice.1.in     

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  \
       -e 's|@PACKAGE_NAME[@]|$(PACKAGE_NAME)|g'  \
       -e 's|@PACKAGE_VERSION[@]|$(PACKAGE_VERSION)|g'  \
       -e 's|@HAVE_DOT[@]|$(HAVE_DOT)|g'   \
       -e 's|@VERSION[@]|$(VERSION)|g'     \
       -e 's|@RELDATE[@]|$(RELDATE)|g'

all-local:

doxygen:	emane.doxygen
	@doxygen emane.doxygen

emane.1:	emane.1.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@

emaneeventd.1:	emaneeventd.1.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@

emanetransportd.1:	emanetransportd.1.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@

emaneeventservice.1:	emaneeventservice.1.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@

emane.doxygen:	emane.doxygen.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@


clean-local:
	-rm -f  $(man_MANS)
	-rm -f emane.doxygen
	-rm -rf html
