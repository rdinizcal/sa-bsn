bsn=$PWD

mate-terminal -x roscore & sleep 5s

################# KNOWLEDGE REPOSITORY #################
mate-terminal --working-directory=${bsn}/configurations/knowledge_repository -e 'roslaunch data_access.launch' & sleep 2s

################# MANAGER #################
mate-terminal --working-directory=${bsn}/configurations/system_manager -e 'roslaunch enactor.launch' & sleep 2s

################# LOGGING INFRASTRUCTURE #################
mate-terminal --working-directory=${bsn}/configurations/logging_infrastructure -e 'roslaunch logger.launch' & sleep 10s

################# APPLICATION #################
mate-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch probe.launch' 
mate-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch effector.launch' & sleep 10s

mate-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g4t1.launch' & sleep 5s
#mate-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_1.launch'
mate-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_2.launch'
#mate-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_3.launch'
#mate-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_4.launch'

################# SIMULATION #################
mate-terminal --working-directory=${bsn}/configurations/simulation -e 'roslaunch injector.launch' & sleep 5s
