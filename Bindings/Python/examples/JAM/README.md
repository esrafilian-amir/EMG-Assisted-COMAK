# JAM Python examples

Python versions of the MATLAB examples in `Resources/Code/Matlab/JAM` (same folders,
same file names, same settings). They are installed to `Resources/Code/Python/JAM`
together with the shared `models` folder.

## Python environment

These steps follow "Scripting in Python" in the OpenSim documentation (manual
installation), with `scipy` added. `<OPENSIM_INSTALL_DIR>` is the installation with the
`bin` and `sdk` folders. The bindings in `sdk/Python` were built for one Python version,
so use that version in the environment (64-bit Python is required).

1. Create and activate a conda environment (the name can be changed). `scipy` is used by
   the EMG, passive flexion and anterior laxity examples; `matplotlib` is optional and
   only plots the inputs:

       conda create -n opensim_scripting python=3.11 numpy scipy
       conda activate opensim_scripting

2. Install the OpenSim Python package of this installation into the environment
   (also installs numpy and scipy if they are missing, see `setup.py`):

       cd <OPENSIM_INSTALL_DIR>/sdk/Python
       python -m pip install .

   If `setup_win_python38.py` exists there (Windows, Python 3.8), run
   `python setup_win_python38.py` first.

3. Make the OpenSim libraries findable:
   - Windows: put `<OPENSIM_INSTALL_DIR>/bin` at the top of the `Path` environment
     variable (Edit the system environment variables, Environment Variables, Path, Edit,
     New, move to the top) and restart the terminal.
   - Mac: `export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:<OPENSIM_INSTALL_DIR>/sdk/lib`
     and `.../sdk/Simbody/lib`.
   - Linux: the same with `LD_LIBRARY_PATH`.

   Without installing the package, put `<OPENSIM_INSTALL_DIR>/bin` first on `PATH` and
   `<OPENSIM_INSTALL_DIR>/sdk/Python` on `PYTHONPATH`.

4. Test:

       python
       >>> import opensim as osim
       >>> osim.GetVersionAndDate()

Troubleshooting: a 32-bit Python or `bin` missing from `Path` are the usual causes of an
import error.

## Run

Each script finds its files relative to its own folder, so it can be started from
anywhere:

    python Resources/Code/Python/JAM/exampleWalking/exampleWalking.py

The results are written next to the script (`inputs`, `results`). Copy the example
folder and the `models` folder to a writable location first if the installation is
in a protected folder.

## Muscle calibration (`exampleWalking_EMG_Calibration`)

Calibrates the muscle-tendon parameters (strength, optimal fiber length, tendon slack length)
and the EMG shape of the Lenhart model against the inverse dynamics moments of the walking
trial and the knee contact force, then reruns COMAK with the calibrated model and EMG. It needs
`scipy` and `matplotlib` and takes about 25 minutes. The calibrated model is written next to
the original one in `models`. Not part of the automated tests. See the docstring of
`exampleWalking_EMG_Calibration.py`; the settings are in `muscle_calibration_setup.xml`.
