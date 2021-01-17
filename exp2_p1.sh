bsn=$PWD

#rm ${bsn}/src/diagnostics/logs/p1/sensors/noiseless/*.log

gnome-terminal -e roscore & sleep 3s

gnome-terminal --working-directory=${bsn}/configurations/experiments/p1/target_system -e 'roslaunch probe.launch' & sleep 1s
gnome-terminal --working-directory=${bsn}/configurations/experiments/p1/target_system -e 'roslaunch effector.launch' & sleep 1s

gnome-terminal --working-directory=${bsn}/configurations/experiments/p1/target_system -e 'roslaunch g4t1.launch' & sleep 5s
gnome-terminal --working-directory=${bsn}/configurations/experiments/p1/target_system -e 'roslaunch g3t1_1.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p1/target_system -e 'roslaunch g3t1_2.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p1/target_system -e 'roslaunch g3t1_3.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p1/environment   -e 'roslaunch patient.launch'  & sleep 120s

rosnode kill -a & sleep 5s

mv -f ${bsn}/src/diagnostics/logs/p1/sensors/*.log ${bsn}/src/diagnostics/logs/p1/sensors/noiseless/
mv -f ${bsn}/src/diagnostics/logs/p1/centralhub/*.log ${bsn}/src/diagnostics/logs/p1/centralhub/noiseless/

gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P1 -e 'roslaunch P1_oximeter_analyzer.launch'
gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P1 -e 'roslaunch P1_ecg_analyzer.launch'
gnome-terminal --working-directory=${bsn}/configurations/property_analyzer/P1 -e 'roslaunch P1_thermometer_analyzer.launch'

gnome-terminal --working-directory=${bsn}/configurations/experiments/p1/target_system -e 'roslaunch probe.launch' & sleep 1s
gnome-terminal --working-directory=${bsn}/configurations/experiments/p1/target_system -e 'roslaunch effector.launch' & sleep 1s

gnome-terminal --working-directory=${bsn}/configurations/experiments/p1/target_system -e 'roslaunch g4t1.launch' & sleep 5s
gnome-terminal --working-directory=${bsn}/configurations/experiments/p1/target_system -e 'roslaunch g3t1_1.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p1/target_system -e 'roslaunch g3t1_2.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p1/target_system -e 'roslaunch g3t1_3.launch'
gnome-terminal --working-directory=${bsn}/configurations/experiments/p1/environment   -e 'roslaunch patient.launch'  & sleep 30s

################# SIMULATION #################
gnome-terminal --working-directory=${bsn}/configurations/experiments/p1/simulation -e 'roslaunch injector.launch' & sleep 90s

rosnode kill -a
