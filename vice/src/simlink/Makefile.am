# simlink - gemeinsamer Speicher zwischen Emulatoren (IEEE-488 und IEC)
# Bewusst ohne VICE-Header: eigenstaendig auch fuer andere Emulatoren nutzbar.

AM_CPPFLAGS = \
	@VICE_CPPFLAGS@ \
	@ARCH_INCLUDES@ \
	-I$(top_builddir)/src \
	-I$(top_srcdir)/src

AM_CFLAGS = @VICE_CFLAGS@

AM_LDFLAGS = @VICE_LDFLAGS@

noinst_LIBRARIES = libsimlink.a

libsimlink_a_SOURCES = \
	simlink.c \
	simlink.h

EXTRA_DIST = \
	README.md \
	test/ieeedev.c \
	test/iechost.c \
	test/relay.c \
	test/buslog.c \
	test/check.sh \
	test/sltest.c \
	test/run.sh
