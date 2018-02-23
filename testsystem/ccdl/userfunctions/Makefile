# Copyright 2010-2017 Razorcat Development GmbH
# Author: Martin Weber

##################################
# Makefile for a library that implements a
# custom extension to the CCD language.
##################################

#
# Change this path to the location where the custom extension 
# library shall be installed to.
#
TARGET_INSTALL_PATH	:= /opt/example.com/CCDL_extension
#
# custom library's name (without leading 'lib' and filename extension)
# Change this to reflect its purpose.
#
lib_name:= ccd_custom_demo
#
# object files of the library
# Adjust to your needs.
#
lib_objects:= userfunctions.o
#
# TRM execution target.
# Change this to the proper target.
# Run 'make -f /opt/trm/lib/Makefile targets' to list installed execution targets.
EXECTARGET:=gamma5x

CFLAGS+=-O3 -Wall -Wextra -Wno-unused-parameter

#
# Libraries referenced by our library and their header files.
# Required only if these have not been plugged in to
# the TRM's build system.
#third_party_libs:= /opt/3rdparty/lib/libTEEBS.a
#third_party_lib_headers:= /opt/3rdparty/include

####################################
# Do not change anything after here
####################################
.SUFFIXES:
.PHONY: all help lib clean mrproper tests run-tests install

basedir:= $(dir $(realpath $(lastword $(MAKEFILE_LIST))))

lib_file:= lib$(lib_name).a

testing_sources:= $(sort $(wildcard $(basedir)tests/*))

trm_build_hook:= 40libdef-$(lib_name).mk
trm_build_hook_is_ccdl_extension= $(lib_name)-tests/LDK/build_hooks/01Buildmacros.addon.is_ccdl_extension.mk


help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... configure    Generates additional files to be installed. (Run this first)"
	@echo
	@echo "... lib          Builds the custom CCDL extension library."
	@echo "... tests        Builds the infrastructure to run the tests."
	@echo "... run-tests    Compiles all tests the same way the TRM-server would do."
	@echo "                  (Make sure the file ccdconfig4tests/functions.txt contains the"
	@echo "                  declarations of the functions provided by the library)"
	@echo "... all          Same as lib + run-tests."
	@echo "... clean        Deletes most of the generated files."
	@echo "... mrproper     Deletes all generated files."
	@echo
	@echo "... install      Copies the files to $(TARGET_INSTALL_PATH)."


.PHONY: configure
configure: $(trm_build_hook)

all: lib run-tests

lib: $(lib_file)
	@echo
	@echo "+++ CCDL extension library '$(lib_name)' build successfully"
	@echo

clean:
	-rm -f $(lib_file) $(lib_objects)
	-rm -rf $(addprefix $(lib_name)-tests/,$(notdir $(testing_sources)))

mrproper: clean
	-rm -rf $(lib_name)-tests

install:
	@echo 'HINT: Define 'DESTDIR' on the command line to test this target!'
	install -d $(DESTDIR)$(TARGET_INSTALL_PATH)/lib
	install -d $(DESTDIR)$(TARGET_INSTALL_PATH)/include
	install -vp $(lib_file) $(DESTDIR)$(TARGET_INSTALL_PATH)/lib
	install -vp $(basedir)src/userfunctions.h $(DESTDIR)$(TARGET_INSTALL_PATH)/include
	install -d $(DESTDIR)/opt/razorcat/ccd3/libdef
	install -vp $(trm_build_hook) $(DESTDIR)/opt/razorcat/ccd3/libdef/$(trm_build_hook)

# creates Makefile include to hook in to the Test Run Management build
$(trm_build_hook): $(MAKEFILE_LIST)
	@echo "# Generated from template for $(lib_name)! EDIT AS NEEDED!" > $@
	@echo "" >> $@
	@echo "# Makefile include to hook in library $(lib_name)" >> $@
	@echo "# to the Test Run Management's build process" >> $@
	@echo 'ifeq ($(EXECTARGET), $$(TRM.TARGET))' >> $@
	@echo "" >> $@
	@echo "dbg::" >> $@
	@echo '	@echo "+++ CCDL extensions from '$(lib_name)' enabled"' >> $@
	@echo "" >> $@
	@echo "# tell C-Compiler to include file userfunctions.h" >> $@
	@echo "Cpp_defines.local+= INCLUDE_USERFUNCTIONS\n" >> $@
	@echo "Includepath.local+= $(DESTDIR)$(TARGET_INSTALL_PATH)/include $(third_party_lib_headers)" >> $@
	@echo "Libpath.local+= $(DESTDIR)$(TARGET_INSTALL_PATH)/lib" >> $@
	@echo "Libs.local+= $(lib_name) $(third_party_libs)" >> $@
	@echo 'endif #($(EXECTARGET), $$(TRM.TARGET))' >> $@
	@echo "+++ Generated $@. PLEASE EDIT THE FILE AS NEEDED!"


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# testing
.PHONY: tests
tests: $(lib_file) $(addprefix $(lib_name)-tests/,$(notdir $(testing_sources)))
	
# creates a project for testing
$(lib_name)-tests:
	trm-LDKtool create_project -p $(lib_name)-tests -L $(basedir)$(lib_file) $(third_party_libs:%=-L %) -I $(basedir)src $(third_party_lib_headers:%=-I %)

# creates a test for each testing source file found
$(lib_name)-tests/%: $(basedir)tests/% $(lib_name)-tests $(trm_build_hook_is_ccdl_extension)
	trm-LDKtool create_test -p $(lib_name)-tests -s $< $*
	@echo "+++ Created test '$*' for $(wildcard $</*.ccd)"

# tells LDK that whe are a ccdl extension lib
$(trm_build_hook_is_ccdl_extension): $(MAKEFILE_LIST)
	@echo "# Tell LDK we are a ccdl extension lib" > $@
	@echo "# Generated on $$(date -u) by $(MAKEFILE_LIST)" >> $@
	@echo "Cpp_defines.local+= INCLUDE_USERFUNCTIONS" >> $@
	@echo "CCDC_FLAGS=-C $(basedir)ccdconfig4tests" >> $@
#	@echo 'TRM.TARGET= $(EXECTARGET)' >> $@

run-tests: $(addprefix run-,$(notdir $(testing_sources)))
	@echo
	@echo "+++ Finished tests."
	@echo

# test the library (build it like TRM does)
run-%: $(lib_name)-tests/%
	@echo "+++ Running test '$*'..."
	@echo
	$(MAKE) -C $< TRM.TARGET=$(EXECTARGET)


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Pattern rules
#--
%.o: $(basedir)src/%.c
	$(COMPILE.c) $(OUTPUT_OPTION) $<

$(lib_file): $(lib_file)($(lib_objects))

