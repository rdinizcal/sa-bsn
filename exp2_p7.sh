bsn=$PWD

gnome-terminal -e roscore & sleep 3s

gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P7 -e 'roslaunch P7_oximeter_analyzer.launch'
gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P7 -e 'roslaunch P7_ecg_analyzer.launch'
gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P7 -e 'roslaunch P7_thermometer_analyzer.launch'


gnome-terminal --working-directory=${bsn}/configurations/experiments/p7/target_system -e 'roslaunch probe.launch' & sleep 1s
gnome-terminal --working-directory=${bsn}/configurations/experiments/p7/target_system -e 'roslaunch effector.launch' & sleep 1s

gnome-terminal --working-directory=${bsn}/configurations/experiments/p7/target_system -e 'roslaunch g4t1.launch' & sleep 5s
gnome-terminal --working-directory=${bsn}/configurations/experiments/p7/target_system -e 'roslaunch g3t1_1.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p7/target_system -e 'roslaunch g3t1_2.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p7/target_system -e 'roslaunch g3t1_3.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p7/environment   -e 'roslaunch patient.launch'  & sleep 30s

################# SIMULATION #################
gnome-terminal --working-directory=${bsn}/configurations/experiments/p7/simulation -e 'roslaunch injector.launch'
