#
# my_first_genom codels library
#

lib_LTLIBRARIES = libmy_first_genom_codels.la


# ---- Sources -------------------------------------------------------------

libmy_first_genom_codels_la_SOURCES  = my_first_genom_c_types.h
libmy_first_genom_codels_la_SOURCES += my_first_genom_codels.c
libmy_first_genom_codels_la_SOURCES += my_first_genom_main_codels.c

libmy_first_genom_codels_la_SOURCES += controller.cc
libmy_first_genom_codels_la_SOURCES += geometry.cc
libmy_first_genom_codels_la_SOURCES += codels.h


# ---- Compiler / linker flags --------------------------------------------

libmy_first_genom_codels_la_CPPFLAGS  = $(requires_CFLAGS)
libmy_first_genom_codels_la_LIBADD    = $(requires_LIBS)

libmy_first_genom_codels_la_CPPFLAGS += $(codels_requires_CFLAGS)
libmy_first_genom_codels_la_LIBADD   += $(codels_requires_LIBS)

libmy_first_genom_codels_la_LDFLAGS   = -release $(PACKAGE_VERSION)


# ---- IDL mappings --------------------------------------------------------

BUILT_SOURCES = my_first_genom_c_types.h
CLEANFILES    = $(BUILT_SOURCES)

my_first_genom_c_types.h: ${top_srcdir}/my_first_genom.gen
	${GENOM3} mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< > $@

-include .deps/my_first_genom_c_types.h.d
