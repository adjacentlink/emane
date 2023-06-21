lib_LTLIBRARIES = libemane-model-bentpipe.la

libemane_model_bentpipe_la_CPPFLAGS= \
 -I@top_srcdir@/include \
 $(AM_CPPFLAGS) \
 $(libemane_CFLAGS)

libemane_model_bentpipe_la_LIBADD= \
 $(libuuid_LIBS) \
 $(libxml2_LIBS) \
 $(protobuf_LIBS) \
 @top_srcdir@/src/libemane/.libs/libemane.la

libemane_model_bentpipe_la_LDFLAGS= \
 $(AM_LDFLAGS) \
 -avoid-version

libemane_model_bentpipe_la_SOURCES = \
 antennas.h \
 antennastatuspublisher.cc \
 antennastatuspublisher.h \
 bentpipemessage.h \
 bentpipemessage.inl \
 configurationvalidationmanager.cc \
 configurationvalidationmanager.h \
 configurehelpers.cc \
 configurehelpers.h \
 configurehelpers.inl \
 messagecomponent.h \
 messagecomponent.inl \
 neighborstatuspublisher.cc \
 neighborstatuspublisher.h \
 packetstatuspublisher.cc \
 packetstatuspublisher.h \
 pcrmanager.cc \
 pcrmanager.h \
 queue.cc \
 queue.h \
 queueinfo.h \
 queuemanager.cc \
 queuemanager.h \
 queuestatuspublisher.cc \
 queuestatuspublisher.h \
 radiomodel.cc \
 radiomodel.h \
 radiomodel.inl \
 receivemanager.cc \
 receivemanager.h \
 slotstatuspublisher.cc \
 slotstatuspublisher.h \
 slotter.h \
 slotter.inl \
 transponder.cc \
 transponderconfiguration.cc \
 transponderconfiguration.h \
 transponderconfigurationupdate.cc \
 transponderconfigurationupdate.h \
 transponder.h \
 transpondernoprotocol.cc \
 transpondernoprotocol.h \
 transponderpackettransport.h \
 transponderstatuspublisher.cc \
 transponderstatuspublisher.h \
 transpondertdmaprotocol.cc \
 transpondertdmaprotocol.h \
 transponderuser.h \
 types.h

nodist_libemane_model_bentpipe_la_SOURCES = \
 bentpipemessage.pb.cc \
 bentpipemessage.pb.h

EXTRA_DIST = \
 bentpipemessage.proto

BUILT_SOURCES = \
 $(nodist_libemane_model_bentpipe_la_SOURCES)

clean-local:
	rm -f $(BUILT_SOURCES)

bentpipemessage.pb.cc bentpipemessage.pb.h: bentpipemessage.proto
	protoc -I=. --cpp_out=. $<

