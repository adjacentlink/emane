pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = libemane.pc

lib_LTLIBRARIES = libemane.la

libemane_la_CPPFLAGS =  \
 -I@top_srcdir@/include \
 $(AM_CPPFLAGS)         \
 $(libemane_CFLAGS)

libemane_la_LIBADD=      \
 -lrt                    \
 $(libxml2_LIBS)         \
 $(protobuf_LIBS)        \
 $(libpcre_LIBS)         \
 $(libuuid_LIBS)

libemane_la_LDFLAGS=     \
 $(AM_LDFLAGS)           \
 -avoid-version

nodist_libemane_la_SOURCES =             \
 antennaprofileevent.pb.cc               \
 antennaprofileevent.pb.h                \
 commeffectevent.pb.cc                   \
 commeffectevent.pb.h                    \
 commonmacheader.pb.cc                   \
 commonmacheader.pb.h                    \
 commonphyheader.pb.cc                   \
 commonphyheader.pb.h                    \
 event.pb.cc                             \
 event.pb.h                              \
 fadingselectionevent.pb.cc              \
 fadingselectionevent.pb.h               \
 flowcontrol.pb.cc                       \
 flowcontrol.pb.h                        \
 locationevent.pb.cc                     \
 locationevent.pb.h                      \
 loggermessage.pb.cc                     \
 loggermessage.pb.h                      \
 otaheader.pb.cc                         \
 otaheader.pb.h                          \
 otatransmitter.pb.cc                    \
 otatransmitter.pb.h                     \
 pathlossevent.pb.cc                     \
 pathlossevent.pb.h                      \
 radiotorouter.pb.cc                     \
 radiotorouter.pb.h                      \
 remotecontrolportapi.pb.cc              \
 remotecontrolportapi.pb.h               \
 tdmascheduleevent.pb.cc                 \
 tdmascheduleevent.pb.h

libemane_la_SOURCES =                         \
 antennapattern.cc                            \
 antennaprofilecontrolmessage.cc              \
 antennaprofileevent.cc                       \
 antennaprofileeventformatter.cc              \
 antennaprofilemanifest.cc                    \
 any.cc                                       \
 boundarymessagemanager.cc                    \
 buildidservice.cc                            \
 commeffectevent.cc                           \
 commeffecteventformatter.cc                  \
 commonlayerstatistics.cc                     \
 commonmacheader.cc                           \
 commonphyheader.cc                           \
 configurationcontroller.cc                   \
 configurationqueryhandler.cc                 \
 configurationregistrarproxy.cc               \
 configurationservice.cc                      \
 configurationupdatehandler.cc                \
 controlmessageserializer.cc                  \
 controlportservice.cc                        \
 controlportsession.cc                        \
 datagramsocket.cc                            \
 downstreampacket.cc                          \
 downstreamtransport.cc                       \
 errorresponse.cc                             \
 eventagentbuilder.cc                         \
 eventagentfactory.cc                         \
 eventagentfactorymanager.cc                  \
 eventagentmanagerimpl.cc                     \
 eventgeneratorbuilder.cc                     \
 eventgeneratorfactory.cc                     \
 eventgeneratorfactorymanager.cc              \
 eventgeneratormanagerimpl.cc                 \
 eventregistrarproxy.cc                       \
 eventservice.cc                              \
 eventserviceproxy.cc                         \
 eventstatisticpublisher.cc                   \
 eventtablepublisher.cc                       \
 fadingmanager.cc                             \
 fadingselectionevent.cc                      \
 fadingselectioneventformatter.cc             \
 flowcontrolclient.cc                         \
 flowcontrolcontrolmessage.cc                 \
 flowcontrolmanager.cc                        \
 frameworkphy.cc                              \
 frequencycontrolmessage.cc                   \
 frequencycontrolmessageformatter.cc          \
 frequencyofinterestcontrolmessage.cc         \
 frequencyofinterestcontrolmessageformatter.cc\
 gainmanager.cc                               \
 inetaddr.cc                                  \
 layerfactorymanager.cc                       \
 libemane.cc                                  \
 locationevent.cc                             \
 locationeventformatter.cc                    \
 locationinfoformatter.cc                     \
 locationmanager.cc                           \
 logger.cc                                    \
 loglevelupdatehandler.cc                     \
 loggerrecordmessage.cc                       \
 logservice.cc                                \
 maclayer.cc                                  \
 main.cc                                      \
 manifestqueryhandler.cc                      \
 multicastsocket.cc                           \
 nakagamifadingalgorithm.cc                   \
 neighbormetricmanager.cc                     \
 nembuilder.cc                                \
 nemimpl.cc                                   \
 nemlayerstack.cc                             \
 nemlayerstate.cc                             \
 nemlayerstateconfigured.cc                   \
 nemlayerstatedestroyed.cc                    \
 nemlayerstateinitialized.cc                  \
 nemlayerstaterunning.cc                      \
 nemlayerstatestopped.cc                      \
 nemlayerstateuninitialized.cc                \
 nemmanagerimpl.cc                            \
 nemnetworkadapter.cc                         \
 nemotaadapter.cc                             \
 nemplatformservice.cc                        \
 nemqueuedlayer.cc                            \
 nemstatefullayer.cc                          \
 nemtimerserviceproxy.cc                      \
 noiserecorder.cc                             \
 nopfiledescriptorservice.cc                  \
 orientationformatter.cc                      \
 otamanager.cc                                \
 otastatisticpublisher.cc                     \
 otatransmittercontrolmessage.cc              \
 pathlossevent.cc                             \
 pathlosseventformatter.cc                    \
 phylayer.cc                                  \
 platformservice.cc                           \
 positionformatter.cc                         \
 positionneuformatter.cc                      \
 positionorientationvelocityformatter.cc      \
 queuemetricmanager.cc                        \
 r2rineighbormetriccontrolmessage.cc          \
 r2rineighbormetriccontrolmessageformatter.cc \
 r2riqueuemetriccontrolmessage.cc             \
 r2riqueuemetriccontrolmessageformatter.cc    \
 r2riselfmetriccontrolmessage.cc              \
 r2riselfmetriccontrolmessageformatter.cc     \
 radioservice.cc                              \
 receivepropertiescontrolmessage.cc           \
 receivepropertiescontrolmessageformatter.cc  \
 receivepowertablepublisher.cc                \
 registrarproxy.cc                            \
 requestmessagehandler.cc                     \
 serializedcontrolmessage.cc                  \
 shimlayer.cc                                 \
 socket.cc                                    \
 spectrummonitor.cc                           \
 spectrumwindowutils.cc                       \
 statisticclearupdatehandler.cc               \
 statisticcontroller.cc                       \
 statisticqueryhandler.cc                     \
 statisticregistrarproxy.cc                   \
 statisticservice.cc                          \
 statistictableclearupdatehandler.cc          \
 statistictablequeryhandler.cc                \
 tdmascheduleevent.cc                         \
 timer.cc                                     \
 timerservice.cc                              \
 timerserviceproxy.cc                         \
 timestampcontrolmessage.cc                   \
 transmittercontrolmessage.cc                 \
 transmittercontrolmessageformatter.cc        \
 transportadapterimpl.cc                      \
 transportbuilder.cc                          \
 transportfactory.cc                          \
 transportfactorymanager.cc                   \
 transportlayer.cc                            \
 transportmanagerimpl.cc                      \
 txwhilerxinterferencecontrolmessage.cc       \
 upstreampacket.cc                            \
 upstreamtransport.cc                         \
 velocityformatter.cc                         \
 anyutils.h                                   \
 boundarymessagemanagerexception.h            \
 boundarymessagemanager.h                     \
 buildidservice.h                             \
 configurationqueryhandler.h                  \
 configurationregistrarproxy.h                \
 configurationservice.h                       \
 configurationupdatehandler.h                 \
 controlmessageserializerexception.h          \
 controlmessageserializer.h                   \
 controlmessageserializermessages.h           \
 controlportservice.h                         \
 controlportsession.h                         \
 datagramsocket.h                             \
 emulator.h                                   \
 errorresponse.h                              \
 eventagentfactory.h                          \
 eventagentfactorymanager.h                   \
 eventagentmanagerimpl.h                      \
 eventgeneratorfactory.h                      \
 eventgeneratorfactorymanager.h               \
 eventgeneratormanagerimpl.h                  \
 eventregistrarproxy.h                        \
 eventserviceexception.h                      \
 eventservice.h                               \
 eventserviceproxy.h                          \
 eventstatisticpublisher.h                    \
 frameworkphy.h                               \
 layerfactory.h                               \
 layerfactory.inl                             \
 layerfactorymanager.h                        \
 loggerlevelconvert.h                         \
 loggerlevelconvert.inl                       \
 loglevelupdatehandler.h                      \
 loggerrecordmessage.h                        \
 logservice.h                                 \
 maclayer.h                                   \
 main.h                                       \
 manifestqueryhandler.h                       \
 multicastsocket.h                            \
 nemimpl.h                                    \
 nemlayerstack.h                              \
 nemlayerstateconfigured.h                    \
 nemlayerstatedestroyed.h                     \
 nemlayerstate.h                              \
 nemlayerstateinitialized.h                   \
 nemlayerstaterunning.h                       \
 nemlayerstatestopped.h                       \
 nemlayerstateuninitialized.h                 \
 nemmanagerimpl.h                             \
 nemnetworkadapter.h                          \
 nemotaadapter.h                              \
 nemplatformservice.h                         \
 nemqueuedlayer.h                             \
 nemqueuedlayer.inl                           \
 nemstatefullayer.h                           \
 nemtimerserviceproxy.h                       \
 netadaptermessage.h                          \
 networkadapterexception.h                    \
 nopfiledescriptorservice.h                   \
 otaexception.h                               \
 otamanager.h                                 \
 otaprovider.h                                \
 otastatisticpublisher.h                      \
 otauser.h                                    \
 phylayer.h                                   \
 platformservice.h                            \
 registrarproxy.h                             \
 requestmessagehandler.h                      \
 socket.h                                     \
 socketexception.h                            \
 shimlayer.h                                  \
 statisticclearupdatehandler.h                \
 statisticqueryhandler.h                      \
 statisticregistrarproxy.h                    \
 statisticservice.h                           \
 statistictableclearupdatehandler.h           \
 statistictablequeryhandler.h                 \
 timerserviceexception.h                      \
 timerservice.h                               \
 timerservice.inl                             \
 timerserviceproxy.h                          \
 transportadapterimpl.h                       \
 transportfactory.h                           \
 transportfactorymanager.h                    \
 transportmanagerimpl.h                       \
 transportlayer.h

BUILT_SOURCES =                               \
 $(nodist_libemane_la_SOURCES)

EXTRA_DIST =                                  \
 antennaprofileevent.proto                    \
 commeffectevent.proto                        \
 commonmacheader.proto                        \
 commonphyheader.proto                        \
 event.proto                                  \
 fadingselectionevent.proto                   \
 flowcontrol.proto                            \
 locationevent.proto                          \
 loggermessage.proto                          \
 otaheader.proto                              \
 otatransmitter.proto                         \
 pathlossevent.proto                          \
 radiotorouter.proto                          \
 remotecontrolportapi.proto                   \
 tdmascheduleevent.proto

flowcontrol.pb.cc flowcontrol.pb.h: flowcontrol.proto
	protoc -I=. --cpp_out=. $<

otatransmitter.pb.cc otatransmitter.pb.h: otatransmitter.proto
	protoc -I=. --cpp_out=. $<

otaheader.pb.cc otaheader.pb.h: otaheader.proto
	protoc -I=. --cpp_out=. $<

radiotorouter.pb.cc radiotorouter.pb.h: radiotorouter.proto
	protoc -I=. --cpp_out=. $<

commonphyheader.pb.cc commonphyheader.pb.h: commonphyheader.proto
	protoc -I=. --cpp_out=. $<

antennaprofileevent.pb.cc antennaprofileevent.pb.h: antennaprofileevent.proto
	protoc -I=. --cpp_out=. $<

locationevent.pb.cc locationevent.pb.h:	locationevent.proto
	protoc -I=. --cpp_out=. $<

pathlossevent.pb.cc pathlossevent.pb.h:	pathlossevent.proto
	protoc -I=. --cpp_out=. $<

commeffectevent.pb.cc commeffectevent.pb.h: commeffectevent.proto
	protoc -I=. --cpp_out=. $<

event.pb.cc event.pb.h:	event.proto
	protoc -I=. --cpp_out=. $<

loggermessage.pb.cc loggermessage.pb.h: loggermessage.proto
	protoc -I=. --cpp_out=. $<

commonmacheader.pb.cc commonmacheader.pb.h: commonmacheader.proto
	protoc -I=. --cpp_out=. $<

remotecontrolportapi.pb.cc remotecontrolportapi.pb.h: remotecontrolportapi.proto
	protoc -I=. --cpp_out=. $<

tdmascheduleevent.pb.cc tdmascheduleevent.pb.h: tdmascheduleevent.proto
	protoc -I=. --cpp_out=. $<

fadingselectionevent.pb.cc fadingselectionevent.pb.h: fadingselectionevent.proto
	protoc -I=. --cpp_out=. $<

clean-local:
	rm -f $(BUILT_SOURCES)
