AM_CPPFLAGS = -I$(top_srcdir)/include -I$(top_srcdir)/src/libemane

check_PROGRAMS = configurationservice_regex
TESTS = $(check_PROGRAMS)

configurationservice_regex_SOURCES = configurationservice_regex.cc
configurationservice_regex_LDADD = $(top_builddir)/src/libemane/libemane.la
