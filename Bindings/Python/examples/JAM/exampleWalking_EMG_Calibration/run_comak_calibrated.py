"""Run COMAK again with the calibrated model and EMG of run_muscle_calibration.py.

Takes the COMAK setup of the baseline run, replaces the model, the EMG file and the
results folder, and runs it from the folder the baseline was run from.

    python run_comak_calibrated.py --install <install> --comak-setup <comak_settings.xml>
        --model <calibrated.osim> --emg <calibrated_emg.sto> --results <folder>
"""
import argparse
import os
import sys


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--install', default=None, help='OpenSim install, if the bindings are not importable')
    p.add_argument('--comak-setup', required=True)
    p.add_argument('--model', required=True)
    p.add_argument('--emg', required=True)
    p.add_argument('--results', required=True)
    p.add_argument('--cwd', default=None, help='default: the parent folder of the setup file')
    p.add_argument('--tendon-compliance', action='store_true',
                   help='use_tendon_compliance on: optimal fiber length and tendon slack length '
                        'then act on the COMAK muscle force')
    p.add_argument('--joint-mechanics', action='store_true',
                   help='also run the JointMechanicsTool after COMAK (contact forces), '
                        'written to <results>/joint_mechanics')
    a = p.parse_args(argv)

    if a.install:    # not needed when the OpenSim Python bindings can already be imported
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
    if a.tendon_compliance:
        comak.set_use_tendon_compliance(True)
    if a.joint_mechanics:
        comak.set_run_joint_mechanics(True)
        comak.upd_joint_mechanics().set_results_directory(
            os.path.abspath(os.path.join(a.results, 'joint_mechanics')))
    comak.printToXML(os.path.join(a.results, 'comak_settings_calibrated.xml'))
    print('Running COMAK with', a.model)
    comak.run()


if __name__ == '__main__':
    main()
