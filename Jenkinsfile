pipeline {
    agent { 
        // !!! REMARK !!!
        // An agent with label 'tessy' and the coresponding TESSY version installed must be available.
        label 'tessy'
    }
    stages {
        stage('Run Tessy Test') {
            steps {
            // !!! REMARK !!!
            // A TESSY installation named 'TESSY_4.3' must be present in the Jenkins 'Global Tool Configuration'.
            withTessyd(installationName: 'TESSY_4.3', 
                   logFile: 'logs/tessy.log', 
                   projectFile: '$WORKSPACE/controller/tessy/tessy.pdbx') {
                   bat '''
                      cd controller

                      tessycmd connect 

                      tessycmd -a restore-db -target project 
                      
                      tessycmd -a exec-test "tessy/execution.tbs" 
                                            
                      tessycmd -a exec-test "tessy/test_details_reports.tbs" 

                      tessycmd xslt %CD%/report/TESSY_OverviewReport.xml 

                      tessycmd disconnect
                      '''
                }
            }
        }
    }
}
