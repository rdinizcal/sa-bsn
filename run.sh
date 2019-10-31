bsn=$PWD

mate-terminal -x roscore & sleep 5s

################# KNOWLEDGE REPOSITORY #################
mate-terminal --working-directory=${bsn}/configurations/knowledge_repository -e 'roslaunch data_access.launch' & sleep 1s

################# MANAGER #################
mate-terminal --working-directory=${bsn}/configurations/system_manager -e 'roslaunch enactor.launch'

################# LOGGING INFRASTRUCTURE #################
mate-terminal --working-directory=${bsn}/configurations/logging_infrastructure -e 'roslaunch logger.launch' & sleep 1s

################# APPLICATION #################
mate-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch probe.launch' & sleep 1s
mate-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch effector.launch' & sleep 1s

mate-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g4t1.launch' & sleep 5s
mate-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_1.launch'
mate-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_2.launch'
mate-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_3.launch'
mate-terminal --working-directory=${bsn}/configurations/environment -e 'roslaunch patient.launch'
#mate-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_4.launch'

################# SIMULATION #################
# mate-terminal --working-directory=${bsn}/configurations/simulation -e 'roslaunch injector.launch' & sleep 150s

mate-terminal --working-directory=${bsn}/configurations/system_manager -e 'roslaunch engine.launch' & sleep 600s