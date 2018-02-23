# Generated from template for ccd_custom_demo! EDIT AS NEEDED!

# Makefile include to hook in library ccd_custom_demo
# to the Test Run Management's build process
ifeq (gamma5x, $(TRM.TARGET))

dbg::
	@echo "+++ CCDL extensions from ccd_custom_demo enabled"

# tell C-Compiler to include file userfunctions.h
Cpp_defines.local+= INCLUDE_USERFUNCTIONS

Includepath.local+= /opt/example.com/CCDL_extension/include 
Libpath.local+= /opt/example.com/CCDL_extension/lib
Libs.local+= ccd_custom_demo 
endif #(gamma5x, $(TRM.TARGET))
