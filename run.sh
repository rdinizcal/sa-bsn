bsn=$PWD

gnome-terminal -e roscore & sleep 3s

#gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P7 -e 'roslaunch P7_oximeter_analyzer.launch'
#gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P7 -e 'roslaunch P7_ecg_analyzer.launch'
#gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P7 -e 'roslaunch P7_thermometer_analyzer.launch'

#gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P8 -e 'roslaunch P8_oximeter_analyzer.launch'
#gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P8 -e 'roslaunch P8_ecg_analyzer.launch'
#gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P8 -e 'roslaunch P8_thermometer_analyzer.launch'

gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P9 -e 'roslaunch P9_oximeter_analyzer.launch'
gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P9 -e 'roslaunch P9_ecg_analyzer.launch'
gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P9 -e 'roslaunch P9_thermometer_analyzer.launch'

#gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P10 -e 'roslaunch P10_analyzer.launch'


################# KNOWLEDGE REPOSITORY #################
#gnome-terminal --working-directory=${bsn}/configurations/knowledge_repository -e 'roslaunch data_access.launch' & sleep 1s

################# MANAGER #################
#gnome-terminal --working-directory=${bsn}/configurations/system_manager -e 'roslaunch enactor.launch'

################# LOGGING INFRASTRUCTURE #################
#gnome-terminal --working-directory=${bsn}/configurations/logging_infrastructure -e 'roslaunch logger.launch' & sleep 1s

################# APPLICATION #################
gnome-terminal --working-directory=${bsn}/configurations/experiments/p9/target_system -e 'roslaunch probe.launch' & sleep 1s
gnome-terminal --working-directory=${bsn}/configurations/experiments/p9/target_system -e 'roslaunch effector.launch' & sleep 1s

gnome-terminal --working-directory=${bsn}/configurations/experiments/p9/target_system -e 'roslaunch g4t1.launch' & sleep 5s
gnome-terminal --working-directory=${bsn}/configurations/experiments/p9/target_system -e 'roslaunch g3t1_1.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p9/target_system -e 'roslaunch g3t1_2.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p9/target_system -e 'roslaunch g3t1_3.launch'
#gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_4.launch'
#gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_5.launch'
gnome-terminal --working-directory=${bsn}/configurations/environment   -e 'roslaunch patient.launch'  & sleep 30s

#gnome-terminal --working-directory=${bsn}/configurations/target_system -e 'roslaunch g3t1_4.launch'

################# SIMULATION #################
#gnome-terminal --working-directory=${bsn}/configurations/simulation -e 'roslaunch analyzer.launch' & sleep 5s
gnome-terminal --working-directory=${bsn}/configurations/experiments/p7 -e 'roslaunch injector.launch'

#gnome-terminal --working-directory=${bsn}/configurations/system_manager -e 'roslaunch engine.launch'