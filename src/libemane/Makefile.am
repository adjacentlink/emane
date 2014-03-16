lib_LTLIBRARIES = libemane.la

libemane_la_CPPFLAGS =  \
 -I@top_srcdir@/include

libemane_la_CXXFLAGS =  \
 $(ACE_CFLAGS)          \
 $(AM_CFLAGS)           \
 $(libuuid_CFLAGS)      \
 $(libxml2_CFLAGS)

BUILT_SOURCES =                          \
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
 remotecontrolportapi.pb.h              

libemane_la_SOURCES =                         \
 antennapattern.cc                            \
 antennaprofilecontrolmessage.cc              \
 antennaprofileevent.cc                       \
 antennaprofilemanifest.cc                    \
 any.cc                                       \
 boundarymessagemanager.cc                    \
 buildidservice.cc                            \
 commeffectevent.cc                           \
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
 eventtablepublisher.cc                       \
 flowcontrolclient.cc                         \
 flowcontrolcontrolmessage.cc                 \
 flowcontrolmanager.cc                        \
 frameworkphy.cc                              \
 frequencycontrolmessage.cc                   \
 frequencycontrolmessageformatter.cc          \
 gainmanager.cc                               \
 layerfactorymanager.cc                       \
 libemane.cc                                  \
 locationevent.cc                             \
 locationinfoformatter.cc                     \
 locationmanager.cc                           \
 logger.cc                                    \
 loglevelupdatehandler.cc                     \
 loggerrecordmessage.cc                       \
 logservice.cc                                \
 maclayer.cc                                  \
 main.cc                                      \
 manifestqueryhandler.cc                      \
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
 nemqueuedlayer.cc                            \
 nemstatefullayer.cc                          \
 noiserecorder.cc                             \
 orientationformatter.cc                      \
 otamanager.cc                                \
 otatransmittercontrolmessage.cc              \
 pathlossevent.cc                             \
 phylayer.cc                                  \
 phylayerfactory.cc                           \
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
 registrarproxy.cc                            \
 requestmessagehandler.cc                     \
 serializedcontrolmessage.cc                  \
 shimlayer.cc                                 \
 spectrummonitor.cc                           \
 spectrumwindowutils.cc                       \
 statisticclearupdatehandler.cc               \
 statisticcontroller.cc                       \
 statisticqueryhandler.cc                     \
 statisticregistrarproxy.cc                   \
 statisticservice.cc                          \
 statistictableclearupdatehandler.cc          \
 statistictablequeryhandler.cc                \
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
 transportmanagerimpl.cc                      \
 upstreampacket.cc                            \
 upstreamtransport.cc                         \
 velocityformatter.cc                         \
 antennaprofileevent.pb.cc                    \
 commeffectevent.pb.cc                        \
 commonmacheader.pb.cc                        \
 commonphyheader.pb.cc                        \
 event.pb.cc                                  \
 flowcontrol.pb.cc                            \
 locationevent.pb.cc                          \
 loggermessage.pb.cc                          \
 otaheader.pb.cc                              \
 otatransmitter.pb.cc                         \
 pathlossevent.pb.cc                          \
 radiotorouter.pb.cc                          \
 remotecontrolportapi.pb.cc                   



EXTRA_DIST=                                   \
 antennapattern.h                             \
 antennaprofileexception.h                    \
 antennaprofilemanifest.h                     \
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
 eventtablepublisher.h                        \
 frameworkphy.h                               \
 freespacepropagationmodelalgorithm.h         \
 gainmanager.h                                \
 layerfactory.h                               \
 layerfactory.inl                             \
 layerfactorymanager.h                        \
 locationinfo.h                               \
 locationinfo.inl                             \
 locationinfoformatter.h                      \
 locationmanager.h                            \
 loggerlevelconvert.h                         \
 loggerlevelconvert.inl                       \
 loglevelupdatehandler.h                      \
 loggerrecordmessage.h                        \
 logservice.h                                 \
 maclayer.h                                   \
 main.h                                       \
 manifestqueryhandler.h                       \
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
 nemqueuedlayer.h                             \
 nemstatefullayer.h                           \
 netadaptermessage.h                          \
 networkadapterexception.h                    \
 noiserecorder.h                              \
 otaexception.h                               \
 otamanager.h                                 \
 otaprovider.h                                \
 otauser.h                                    \
 phylayer.h                                   \
 phylayerfactory.h                            \
 platformservice.h                            \
 positionecef.h                               \
 positionecef.inl                             \
 positionneu.h                                \
 positionneu.inl                              \
 positionneuformatter.h                       \
 positionorientationvelocity.h                \
 positionorientationvelocity.inl              \
 positionorientationvelocityformatter.h       \
 positionutils.h                              \
 precomputedpropagationmodelalgorithm.h       \
 propagationmodelalgorithm.h                  \
 radioservice.h                               \
 registrarproxy.h                             \
 requestmessagehandler.h                      \
 shimlayer.h                                  \
 spectrummonitor.h                            \
 statisticclearupdatehandler.h                \
 statisticqueryhandler.h                      \
 statisticregistrarproxy.h                    \
 statisticservice.h                           \
 statistictableclearupdatehandler.h           \
 statistictablequeryhandler.h                 \
 timerserviceexception.h                      \
 timerservice.h                               \
 timerserviceproxy.h                          \
 transportadapterimpl.h                       \
 transportfactory.h                           \
 transportfactorymanager.h                    \
 transportmanagerimpl.h                       \
 tworaypropagationmodelalgorithm.h            \
 wheel.h                                      \
 wheel.inl                                    \
 antennaprofileevent.proto                    \
 commeffectevent.proto                        \
 commonmacheader.proto                        \
 commonphyheader.proto                        \
 event.proto                                  \
 flowcontrol.proto                            \
 locationevent.proto                          \
 loggermessage.proto                          \
 otaheader.proto                              \
 otatransmitter.proto                         \
 pathlossevent.proto                          \
 radiotorouter.proto                          \
 remotecontrolportapi.proto                   

libemane_la_LIBADD=       \
 $(WIN32_EXTRA_LIBADD)

libemane_la_LDFLAGS=     \
 $(WIN32_EXTRA_LDFLAGS)  \
 $(ACE_LIBS)             \
 $(libxml2_LIBS)         \
 $(libuuid_LIBS)         \
 -avoid-version

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

clean-local:
	rm -f $(BUILT_SOURCES)
