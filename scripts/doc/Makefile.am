man_MANS =                  \
 emanegentransportxml.1    

EXTRA_DIST =                    \
 emanegentransportxml.1.in      


edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  \
       -e 's|@VERSION[@]|$(VERSION)|g'     \
       -e 's|@RELDATE[@]|$(RELDATE)|g'

all-local:	$(man_MANS)

emanegentransportxml.1:	emanegentransportxml.1.in
	$(edit) $< > $@

clean-local:
	rm -f $(man_MANS)

