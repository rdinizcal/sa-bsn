bsn=$PWD

gnome-terminal -x roscore & sleep 5s

################# MANAGER #################
gnome-terminal --working-directory=${bsn}/configurations/manager -e 'roslaunch scheduler.launch'
gnome-terminal --working-directory=${bsn}/configurations/manager -e 'roslaunch controller.launch'

################# LOGGING INFRASTRUCTURE #################
gnome-terminal --working-directory=${bsn}/configurations/logging -e 'roslaunch logger.launch'

################# REPOSITORY #################
gnome-terminal --working-directory=${bsn}/configurations/repository -e 'roslaunch data_access_node.launch'
gnome-terminal --working-directory=${bsn}/configurations/repository -e 'roslaunch illness_identifier.launch'

################# APPLICATION #################
gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch probe.launch'
gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch effector.launch'

gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch g4t1.launch'
gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch g3t1_1.launch'
gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch g3t1_2.launch'
gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch g3t1_3.launch'
gnome-terminal --working-directory=${bsn}/configurations/application -e 'roslaunch g3t1_4.launch'
