## Process this file with automake to produce Makefile.in

INCLUDES = \
	-DPACKAGE_DATA_DIR=\""$(datadir)"\" \
	-DPACKAGE_LOCALE_DIR=\""$(prefix)/$(DATADIRNAME)/locale"\" \
	@PACKAGE_CFLAGS@ -I../../../../SSM03/include -I../../../include

bin_PROGRAMS = param_tuner

param_tuner_SOURCES = \
	main.c \
	support.c support.h \
	interface.c interface.h \
	callbacks.c callbacks.h

param_tuner_LDADD = @PACKAGE_LIBS@ $(INTLLIBS) -L../../../../SSM03/lib \
	-lSSM -L../../../lib -lSHSpur

