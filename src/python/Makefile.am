EXTRA_DIST= \
 AUTHORS \
 COPYING \
 MANIFEST.in \
 scripts \
 setup.py.in \
 emane/info/__init__.py.in\
 emane/ota/otapublisher.py \
 emane/ota/schema/otapublisherscenario.xsd \
 emane/ota/__init__.py \
 emane/ota/otamessage.py \
 emane/ota/otapublisherexception.py \
 emane/shell/manifest.py \
 emane/shell/manifestexception.py \
 emane/shell/controlportclient.py \
 emane/shell/__init__.py \
 emane/shell/controlportexception.py \
 emane/shell/emaneshell.py \
 emane/__init__.py \
 emane/events/fadingselectionevent.py \
 emane/events/antennaprofileevent.py \
 emane/events/tdmaschedule.py \
 emane/events/commeffectevent.py \
 emane/events/schema/tdmaschedule.xsd \
 emane/events/onehopneighborsevent.py \
 emane/events/eventservice.py \
 emane/events/__init__.py \
 emane/events/locationevent.py \
 emane/events/event.py \
 emane/events/pathlossevent.py \
 emane/events/eventserviceexception.py \
 emane/events/tdmascheduleevent.py

BUILT_SOURCES =                  \
 emane/events/antennaprofileevent_pb2.py \
 emane/events/commeffectevent_pb2.py  \
 emane/events/event_pb2.py            \
 emane/events/locationevent_pb2.py    \
 emane/events/pathlossevent_pb2.py    \
 emane/events/tdmascheduleevent_pb2.py \
 emane/events/fadingselectionevent_pb2.py \
 emane/shell/remotecontrolportapi_pb2.py \
 emane/events/onehopneighborsevent_pb2.py \
 emane/ota/otaheader_pb2.py \
 emane/ota/commonphyheader_pb2.py \
 emane/ota/commonmacheader_pb2.py \
 emane/shell/schema/manifest.xsd \
 setup.py

all-local:
	$(PYTHON) setup.py build

clean-local: setup.py
	$(PYTHON) setup.py clean
	-rm -rf build
	-rm -rf dist
	-rm -f MANIFEST $(BUILT_SOURCES)
	-find . -name "*.pyc" -delete
	if test -d emane/shell/schema; then rmdir emane/shell/schema; fi
	-rm -f .installedfiles

sdist:
	$(PYTHON) setup.py sdist

edit = sed                              \
	-e 's|@VERSION[@]|$(VERSION)|g'

emane/shell/remotecontrolportapi_pb2.py: ../libemane/remotecontrolportapi.proto
	protoc -I../libemane --python_out=emane/shell  $<

emane/events/event_pb2.py: ../libemane/event.proto
	protoc -I../libemane --python_out=emane/events  $<

emane/events/locationevent_pb2.py: ../libemane/locationevent.proto
	protoc -I../libemane --python_out=emane/events  $<

emane/ota/otaheader_pb2.py: ../libemane/otaheader.proto
	protoc -I../libemane --python_out=emane/ota  $<

emane/ota/commonphyheader_pb2.py: ../libemane/commonphyheader.proto
	protoc -I../libemane --python_out=emane/ota  $<

emane/ota/commonmacheader_pb2.py: ../libemane/commonmacheader.proto
	protoc -I../libemane --python_out=emane/ota  $<

emane/events/pathlossevent_pb2.py: ../libemane/pathlossevent.proto
	protoc -I../libemane --python_out=emane/events  $<

emane/events/commeffectevent_pb2.py: ../libemane/commeffectevent.proto
	protoc -I../libemane --python_out=emane/events  $<

emane/events/antennaprofileevent_pb2.py: ../libemane/antennaprofileevent.proto
	protoc -I../libemane --python_out=emane/events  $<

emane/events/onehopneighborsevent_pb2.py: ../models/mac/ieee80211abg/onehopneighborsevent.proto
	protoc -I../models/mac/ieee80211abg --python_out=emane/events  $<

emane/events/tdmascheduleevent_pb2.py: ../libemane/tdmascheduleevent.proto
	protoc -I../libemane --python_out=emane/events  $<

emane/events/fadingselectionevent_pb2.py: ../libemane/fadingselectionevent.proto
	protoc -I../libemane --python_out=emane/events  $<

emane/shell/schema/manifest.xsd: @top_srcdir@/schema/manifest.xsd	emane/shell/schema
	cp $< $@

emane/shell/schema:
	mkdir -p emane/shell/schema

setup.py:	setup.py.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@

install-exec-hook: $(BUILT_SOURCES)
	$(PYTHON) setup.py install \
            --single-version-externally-managed \
	    -O1 \
            --record .installedfiles \
            --prefix=$(prefix) \
            --exec-prefix=$(exec_prefix) \
            $(if $(DESTDIR),--root=$(DESTDIR)) \
            $(if $(subst false,,$(HAVE_DEB)),--install-layout=deb)

uninstall-hook:
	if test -f .installedfiles; then xargs -a .installedfiles rm -f; fi
