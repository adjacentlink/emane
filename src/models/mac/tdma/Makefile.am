SUBDIRS=                             \
 .                                   \
 eventscheduler

lib_LTLIBRARIES = libtdmabase.la

libtdmabase_la_CPPFLAGS=             \
 -I@top_srcdir@/include              \
 $(ACE_CFLAGS)                       \
 $(AM_CFLAGS)                        \
 $(libuuid_CFLAGS)                   \
 $(libxml2_CFLAGS)

libtdmabase_la_SOURCES =             \
 basemodel.cc                        \
 basemodelimpl.cc                    \
 basicqueuemanager.cc                \
 pormanager.cc                       \
 receivemanager.cc                   \
 slotstatustablepublisher.cc         \
 txslotinfosformatter.cc             \
 tdmabasemodelheader.pb.cc 

BUILT_SOURCES =                      \
 tdmabasemodelheader.pb.cc           \
 tdmabasemodelheader.pb.h

EXTRA_DIST=                          \
 basemodelheader.h                   \
 basemodelheader.inl                 \
 basemodelimpl.h                     \
 pormanager.h                        \
 queue.h                             \
 receivemanager.h                    \
 slotstatustablepublisher.h          \
 tdmabasemodelheader.proto           \
 tdmabasemodelpcr.xml                \
 txslotinfosformatter.h              


libtdmabase_la_LDFLAGS=              \
 $(ACE_LIBS)                         \
 $(libxml2_LIBS)                     \
 -avoid-version

clean-local:
	rm -f $(BUILT_SOURCES)

tdmabasemodelheader.pb.cc tdmabasemodelheader.pb.h: tdmabasemodelheader.proto
	protoc -I=. --cpp_out=. $<
