"""Run COMAK again with the calibrated model and EMG of run_muscle_calibration.py.

Takes the COMAK setup of the baseline run, replaces the model, the EMG file and the
results folder, and runs it from the folder the baseline was run from.

    python run_comak_calibrated.py --install <install> --comak-setup <comak_settings.xml>
        --model <calibrated.osim> --emg <calibrated_emg.sto> --results <folder>
"""
import argparse
import os
import sys


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--install', required=True)
    p.add_argument('--comak-setup', required=True)
    p.add_argument('--model', required=True)
    p.add_argument('--emg', required=True)
    p.add_argument('--results', required=True)
    p.add_argument('--cwd', default=None, help='default: the parent folder of the setup file')
    a = p.parse_args()

    sys.path.insert(0, os.path.join(a.install, 'sdk', 'python'))
    os.environ['PATH'] = os.path.join(a.install, 'bin') + os.pathsep + os.environ['PATH']
    import opensim as osim

    setup = os.path.abspath(a.comak_setup)
    os.chdir(a.cwd or os.path.join(os.path.dirname(setup), '..'))
    os.makedirs(a.results, exist_ok=True)

    comak = osim.COMAKTool(setup)
    comak.set_model_file(os.path.abspath(a.model))
    comak.set_emg_file(os.path.abspath(a.emg))
    comak.set_results_directory(os.path.abspath(a.results))
    comak.set_settle_sim_results_directory(os.path.abspath(a.results))
    comak.set_use_visualizer(False)
    comak.set_verbose(0)
    comak.printToXML(os.path.join(a.results, 'comak_settings_calibrated.xml'))
    print('Running COMAK with', a.model)
    comak.run()


if __name__ == '__main__':
    main()
