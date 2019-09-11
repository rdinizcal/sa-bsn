bsn=$PWD

gnome-terminal -x roscore & sleep 5s

################# KNOWLEDGE REPOSITORY #################
#gnome-terminal --working-directory=${bsn}/configurations/repository -e 'roslaunch data_access.launch'

################# MANAGER #################
#gnome-terminal --working-directory=${bsn}/configurations/manager -e 'roslaunch scheduler.launch'

################# LOGGING INFRASTRUCTURE #################
#gnome-terminal --working-directory=${bsn}/configurations/logging_infrastructure -e 'roslaunch logger.launch'

################# APPLICATION #################
#gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch probe.launch'
#gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch effector.launch'

gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch g4t1.launch'
gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch g3t1_1.launch'
gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch g3t1_2.launch'
gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch g3t1_3.launch'
gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch g3t1_4.launch'
