#!/bin/bash
report_and_exit()
{
	echo "failed executing '$1', exiting..."
        local CURRRENT_LOGFILE=log.$(date -I)
	cp ~/.razorcat/.tessy/.tessy_50_workspace/.metadata/.log $CURRRENT_LOGFILE
	exit 1
}

# start TESSYD
tessyd -f tessy/tessy.pdbx || report_and_exit "start headless tessy"


tessycmd connect || report_and_exit "connect"

tessycmd -a restore-db -target project 

tessycmd -a exec-test "tessy/execution.tbs" || report_and_exit "exec-test"

# tessycmd -a exec-test "tessy/test_details_reports.tbs" || report_and_exit "generate details report"

tessycmd xslt $(pwd)/report/TESSY_OverviewReport.xml

tessycmd disconnect

# stop TESSYD
tessyd -v shutdown  --copy-log $(pwd)
