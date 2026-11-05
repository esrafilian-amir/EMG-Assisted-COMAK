#!/bin/bash
cd /c/Users/amires/Desktop/GitHub/EMG_COMAK/EMG_COMAK_BLD
L=/c/Users/amires/Desktop/GitHub/EMG_COMAK/EMG-Assisted-COMAK/scripts/regression
echo "== python"; ctest -C RelWithDebInfo -R "^python_" --output-on-failure --timeout 1200 > $L/rerun_python.log 2>&1; grep -E "Test +#|tests passed" $L/rerun_python.log
echo "== testMocoImplicit"; ctest -C RelWithDebInfo -R "^testMocoImplicit$" --output-on-failure --timeout 1200 > $L/rerun_mocoimplicit.log 2>&1; grep -E "Test +#|tests passed" $L/rerun_mocoimplicit.log
echo "== matlab (one at a time)"; ctest -C RelWithDebInfo -R "^Matlab_" --output-on-failure --timeout 1200 -j 1 > $L/rerun_matlab.log 2>&1; grep -E "Test +#|tests passed" $L/rerun_matlab.log
echo "all rerun done"
