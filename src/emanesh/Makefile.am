PYTHON=python

EXTRA_DIST=                      \
 AUTHORS                         \
 COPYING                         \
 MANIFEST.in                     \
 emanesh/events/antennaprofileevent.py \
 emanesh/events/commeffectevent.py \
 emanesh/events/event.py \
 emanesh/events/eventserviceexception.py \
 emanesh/events/eventservice.py \
 emanesh/events/__init__.py \
 emanesh/events/locationevent.py \
 emanesh/events/onehopneighborsevent.py \
 emanesh/events/pathlossevent.py \
 emanesh/events/tdmascheduleevent.py \
 emanesh/events/tdmaschedule.py \
 emanesh/controlportclient.py    \
 emanesh/controlportexception.py \
 emanesh/ota/__init__.py         \
 emanesh/ota/otamessage.py       \
 emanesh/ota/otapublisherexception.py \
 emanesh/ota/otapublisher.py     \
 emanesh/__init__.py             \
 emanesh/emaneshell.py           \
 emanesh/manifest.py             \
 emanesh/manifestexception.py    \
 emanesh/schema/otapublisherscenario.xsd \
 emanesh/schema/tdmaschedule.xsd \
 scripts/emanesh                 \
 scripts/emaneevent-dump         \
 scripts/emaneevent-pathloss     \
 scripts/emaneevent-commeffect   \
 scripts/emaneevent-location     \
 scripts/emaneevent-antennaprofile \
 scripts/emaneevent-tdmaschedule \
 scripts/emaneota-publisher      \
 scripts/emanecommand-eel        \
 setup.py.in                     

BUILT_SOURCES =                  \
 emanesh/events/antennaprofileevent_pb2.py \
 emanesh/events/commeffectevent_pb2.py  \
 emanesh/events/event_pb2.py            \
 emanesh/events/locationevent_pb2.py    \
 emanesh/events/pathlossevent_pb2.py    \
 emanesh/events/tdmascheduleevent_pb2.py \
 emanesh/remotecontrolportapi_pb2.py \
 emanesh/events/onehopneighborsevent_pb2.py \
 emanesh/ota/otaheader_pb2.py \
 emanesh/ota/commonphyheader_pb2.py \
 emanesh/schema/manifest.xsd \
 setup.py

all-local:
	$(PYTHON) setup.py build

clean-local: setup.py
	$(PYTHON) setup.py clean
	-rm -rf build
	-rm -f MANIFEST
	-rm -f emanesh/events/antennaprofileevent_pb2.py
	-rm -f emanesh/events/commeffectevent_pb2.py
	-rm -f emanesh/events/event_pb2.py
	-rm -f emanesh/events/locationevent_pb2.py
	-rm -f emanesh/events/pathlossevent_pb2.py
	-rm -f emanesh/events/tdmascheduleevent_pb2.py
	-rm -f emanesh/remotecontrolportapi_pb2.py
	-rm -f emanesh/events/onehopneighborsevent_pb2.py
	-rm -f emanesh/ota/otaheader_pb2.py
	-rm -f emanesh/ota/commonphyheader_pb2.py
	-rm -f emanesh/schema/manifest.xsd
	-rm -rf dist
	-find . -name "*.pyc" -delete

DISTCLEANFILES=setup.py

sdist:
	$(PYTHON) setup.py sdist

edit = sed                              \
        -e 's|@VERSION[@]|$(VERSION)|g' \
        -e 's|@PYTHON[@]|$(PYTHON)|g'   \
        -e 's|@DISTRIB_CODENAME[@]|$(DISTRIB_CODENAME)|g'

emanesh/remotecontrolportapi_pb2.py: ../libemane/remotecontrolportapi.proto
	protoc -I../libemane --python_out=emanesh  $<

emanesh/events/event_pb2.py: ../libemane/event.proto
	protoc -I../libemane --python_out=emanesh/events  $<

emanesh/events/locationevent_pb2.py: ../libemane/locationevent.proto
	protoc -I../libemane --python_out=emanesh/events  $<

emanesh/ota/otaheader_pb2.py: ../libemane/otaheader.proto
	protoc -I../libemane --python_out=emanesh/ota  $<

emanesh/ota/commonphyheader_pb2.py: ../libemane/commonphyheader.proto
	protoc -I../libemane --python_out=emanesh/ota  $<

emanesh/events/pathlossevent_pb2.py: ../libemane/pathlossevent.proto
	protoc -I../libemane --python_out=emanesh/events  $<

emanesh/events/commeffectevent_pb2.py: ../libemane/commeffectevent.proto
	protoc -I../libemane --python_out=emanesh/events  $<

emanesh/events/antennaprofileevent_pb2.py: ../libemane/antennaprofileevent.proto
	protoc -I../libemane --python_out=emanesh/events  $<

emanesh/events/onehopneighborsevent_pb2.py: ../models/mac/ieee80211abg/onehopneighborsevent.proto
	protoc -I../models/mac/ieee80211abg --python_out=emanesh/events  $<

emanesh/events/tdmascheduleevent_pb2.py: ../libemane/tdmascheduleevent.proto
	protoc -I../libemane --python_out=emanesh/events  $<

emanesh/schema/manifest.xsd: @top_srcdir@/schema/manifest.xsd	emanesh/schema
	cp $< $@

setup.py:	setup.py.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@

install-exec-hook: $(BUILT_SOURCES)
	$(PYTHON) setup.py install \
    -O1 \
    --root=$(DESTDIR)
