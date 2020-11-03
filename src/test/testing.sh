bsn=$PWD

gnome-terminal -x roscore & sleep 5s

gnome-terminal --working-directory=${bsn}/src/test -e 'rostest --reuse-master test_suite testnode.launch'

################# KNOWLEDGE REPOSITORY #################
#gnome-terminal --working-directory=${bsn}/configurations/knowledge_repository -e 'roslaunch data_access.launch' & sleep 1s

################# MANAGER #################
#gnome-terminal --working-directory=${bsn}/configurations/system_manager -e 'roslaunch enactor.launch'

################# LOGGING INFRASTRUCTURE #################
#gnome-terminal --working-directory=${bsn}/configurations/logging_infrastructure -e 'roslaunch logger.launch' & sleep 1s

################# APPLICATION #################
gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch probe.launch' & sleep 1s
gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch effector.launch' & sleep 1s

gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g4t1.launch' & sleep 10s
gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_1.launch'
gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_2.launch'
gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_3.launch'
gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_4.launch'
gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_5.launch'
#gnome-terminal --working-directory=${bsn}/configurations/environment   -e 'roslaunch patient.launch'

#gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_4.launch'

################# SIMULATION #################
#gnome-terminal --working-directory=${bsn}/configurations/simulation -e 'roslaunch analyzer.launch' & sleep 5s
#gnome-terminal --working-directory=${bsn}/configurations/simulation -e 'roslaunch injector.launch'

#gnome-terminal --working-directory=${bsn}/configurations/system_manager -e 'roslaunch engine.launch'