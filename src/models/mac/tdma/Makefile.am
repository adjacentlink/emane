SUBDIRS=                             \
 .                                   \
 eventscheduler

lib_LTLIBRARIES = libtdmabase.la

libtdmabase_la_CPPFLAGS=             \
 -I@top_srcdir@/include              \
 $(AM_CPPFLAGS)                      \
 $(libemane_CFLAGS)

libtdmabase_la_LIBADD=                       \
 $(libuuid_LIBS)                             \
 $(libxml2_LIBS)                             \
 $(protobuf_LIBS)                            \
 @top_srcdir@/src/libemane/.libs/libemane.la

libtdmabase_la_LDFLAGS=              \
 $(AM_LDFLAGS)                       \
 -avoid-version

libtdmabase_la_SOURCES =             \
 aggregationstatuspublisher.cc       \
 basemodel.cc                        \
 basemodelimpl.cc                    \
 basicqueuemanager.cc                \
 packetstatuspublisherimpl.cc        \
 pormanager.cc                       \
 queue.cc                            \
 queuestatuspublisher.cc             \
 receivemanager.cc                   \
 slotstatustablepublisher.cc         \
 txslotinfosformatter.cc             \
 downstreamqueue.cc					 \
 aggregationstatuspublisher.h        \
 basemodelimpl.h                     \
 basemodelmessage.h                  \
 basemodelmessage.inl                \
 packetstatuspublisherimpl.h         \
 pormanager.h                        \
 priority.h                          \
 queue.h                             \
 queuestatuspublisher.h              \
 receivemanager.h                    \
 slotstatustablepublisher.h          \
 txslotinfosformatter.h				 \
 downstreamqueue.h

nodist_libtdmabase_la_SOURCES =      \
 tdmabasemodelmessage.pb.cc          \
 tdmabasemodelmessage.pb.h

EXTRA_DIST=                          \
 tdmabasemodelmessage.proto          \
 tdmabasemodelpcr.xml

BUILT_SOURCES =                      \
 $(nodist_libtdmabase_la_SOURCES)

clean-local:
	rm -f $(BUILT_SOURCES)

tdmabasemodelmessage.pb.cc tdmabasemodelmessage.pb.h: tdmabasemodelmessage.proto
	protoc -I=. --cpp_out=. $<
