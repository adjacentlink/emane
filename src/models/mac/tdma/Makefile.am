SUBDIRS=                             \
 .                                   \
 eventscheduler

lib_LTLIBRARIES = libtdmabase.la

libtdmabase_la_CPPFLAGS=             \
 -I@top_srcdir@/include              \
 $(AM_CFLAGS)                        \
 $(libuuid_CFLAGS)                   \
 $(libxml2_CFLAGS)

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
 tdmabasemodelmessage.pb.cc          \
 txslotinfosformatter.cc 

BUILT_SOURCES =                      \
 tdmabasemodelmessage.pb.cc          \
 tdmabasemodelmessage.pb.h

EXTRA_DIST=                          \
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
 tdmabasemodelmessage.proto          \
 tdmabasemodelpcr.xml                \
 txslotinfosformatter.h

libtdmabase_la_LDFLAGS=              \
 $(libxml2_LIBS)                     \
 -avoid-version

clean-local:
	rm -f $(BUILT_SOURCES)

tdmabasemodelmessage.pb.cc tdmabasemodelmessage.pb.h: tdmabasemodelmessage.proto
	protoc -I=. --cpp_out=. $<
