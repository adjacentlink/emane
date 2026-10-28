lib_LTLIBRARIES = libemane-transport-traffic.la

libemane_transport_traffic_la_CPPFLAGS= \
 -I@top_srcdir@/include \
 $(AM_CPPFLAGS) \
 $(libemane_CFLAGS)

libemane_transport_traffic_la_LIBADD= \
 $(libuuid_LIBS) \
 @top_srcdir@/src/libemane/.libs/libemane.la

libemane_transport_traffic_la_LDFLAGS= \
 $(AM_LDFLAGS) \
 -avoid-version

nodist_libemane_transport_traffic_la_SOURCES = \
 traffic.pb.cc \
 traffic.pb.h

libemane_transport_traffic_la_SOURCES= \
 session.cc \
 session.h \
 transport.cc \
 transport.h

BUILT_SOURCES = \
 $(nodist_libemane_transport_traffic_la_SOURCES)

EXTRA_DIST = \
 traffic.proto

clean-local:
	rm -f $(BUILT_SOURCES)

traffic.pb.cc traffic.pb.h: traffic.proto
	protoc -I=. --cpp_out=. $<
