# Copyright 2010-2016 Razorcat Development GmbH
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
	@echo -e "... configure \t\tGenerates additional files to be installed. (Run this first)"
	@echo
	@echo -e "... lib   \t\tBuilds the custom CCDL extension library."
	@echo -e "... tests \t\tBuilds the infrastructure to run the tests."
	@echo -e "... run-tests \t\tCompiles all tests the same way the TRM-server would do."
	@echo -e "\t\t\t (Make sure the file ccdconfig4tests/functions.txt contains the declarations"
	@echo -e "\t\t\t of the functions provided by the library)"
	@echo -e "... all \t\tSame as lib + run-tests."
	@echo -e "... clean \t\tDeletes most of the generated files."
	@echo -e "... mrproper \t\tDeletes all the generated files."
	@echo
	@echo -e "... install \t\tCopies the files to $(TARGET_INSTALL_PATH)."
#	@echo -e "\n$(MAKEFILE_LIST)"


.PHONY: configure
configure: $(trm_build_hook)

all: lib run-tests

lib: $(lib_file)
	@echo -e "\n+++ CCDL extension library '$(lib_name)' build successfully\n"

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
# 	@echo -e "\n+++ Library installed successfully\n"

# creates Makefile include to hook in to the Test Run Management build
$(trm_build_hook): $(MAKEFILE_LIST)
	@echo -e "# Generated from template for $(lib_name)! EDIT AS NEEDED!\n" > $@
	@echo "# Makefile include to hook in library $(lib_name)" >> $@
	@echo "# to the Test Run Management System's build process" >> $@
	@echo 'ifeq ($(EXECTARGET), $$(TRM.TARGET))' >> $@
	@echo "" >> $@
	@echo "dbg::" >> $@
	@echo -e '\t@echo "+++ CCDL extensions from '$(lib_name)' enabled"' >> $@
	@echo "" >> $@
	@echo "# tell C-Compiler to include file userfunctions.h" >> $@
	@echo -e "Cpp_defines.local+= INCLUDE_USERFUNCTIONS\n" >> $@
	@echo -e "Includepath.local+= $(DESTDIR)$(TARGET_INSTALL_PATH)/include $(third_party_lib_headers)\n" >> $@
	@echo -e "Libpath.local+= $(DESTDIR)$(TARGET_INSTALL_PATH)/lib\n" >> $@
	@echo -e "Libs.local+= $(lib_name) $(third_party_libs)\n" >> $@
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
	@echo -e "# Generated on $$(date -u) by $(MAKEFILE_LIST)\n" >> $@
	@echo -e "Cpp_defines.local+= INCLUDE_USERFUNCTIONS\n" >> $@
	@echo -e "CCDC_FLAGS=-C $(basedir)ccdconfig4tests\n" >> $@
#	@echo 'TRM.TARGET= $(EXECTARGET)' >> $@

run-tests: $(addprefix run-,$(notdir $(testing_sources)))
	@echo -e "\n+++ Finished tests.\n"

# test the library (build it like TRM does)
run-%: $(lib_name)-tests/%
	@echo -e "+++ Running test '$*'...\n"
	$(MAKE) -C $< TRM.TARGET=$(EXECTARGET)


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Pattern rules
#--
%.o: $(basedir)src/%.c
	$(COMPILE.c) $(OUTPUT_OPTION) $<

$(lib_file): $(lib_file)($(lib_objects))

