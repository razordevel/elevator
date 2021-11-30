#!/bin/bash
report_and_exit()
{
	echo "failed executing '$@', exiting..."

	# try a gracefull shutdown
	tessycmd disconnect
        tessyd -v shutdown  --copy-log $(pwd)

	local LOGFILE=~/.razorcat/.tessy/.tessy_50_workspace/.metadata/.log 
	if [ -f $LOGFILE ] ; then
		cp  -v $LOGFILE log.$(date -I)
	fi
	exit 1
}

tessycmd_checked() {
	tessycmd "$@" || report_and_exit "tessycmd $@"
}

# start TESSYD
tessyd -f tessy/tessy.pdbx || report_and_exit "start headless tessy"


tessycmd_checked connect 

tessycmd_checked -a restore-db -target project 

tessycmd_checked -a exec-test "tessy/execution.tbs" 

tessycmd_checked -a exec-test "tessy/test_details_reports.tbs" 

tessycmd_checked xslt $(pwd)/report/TESSY_OverviewReport.xml 

tessycmd_checked disconnect

# stop TESSYD
tessyd -v shutdown  --copy-log $(pwd)
