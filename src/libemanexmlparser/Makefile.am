lib_LTLIBRARIES = libemanexmlparser.la

libemanexmlparser_la_CPPFLAGS= \
 -I@top_srcdir@/include        \
 $(libxml2_CFLAGS)             \
 $(AM_CFLAGS)

libemanexmlparser_la_SOURCES=      \
 configurationparser.cc            \
 eventagentconfiguration.cc        \
 eventdaemonconfiguration.cc       \
 eventgeneratorconfiguration.cc    \
 eventserviceconfiguration.cc      \
 layerconfiguration.cc             \
 nemconfiguration.cc               \
 platformconfiguration.cc          \
 pluginconfiguration.cc            \
 transportdaemonconfiguration.cc   \
 transportinstanceconfiguration.cc

EXTRA_DIST=                       \
 configurationparser.h            \
 eventagentconfiguration.h        \
 eventdaemonconfiguration.h       \
 eventgeneratorconfiguration.h    \
 emaneparseexception.h            \
 eventserviceconfiguration.h      \
 layerconfiguration.h             \
 nemconfiguration.h               \
 platformconfiguration.h          \
 pluginconfiguration.h            \
 transportdaemonconfiguration.h   \
 transportinstanceconfiguration.h

libemanexmlparser_la_LDFLAGS= \
 $(libxml2_LIBS)              \
 -avoid-version
