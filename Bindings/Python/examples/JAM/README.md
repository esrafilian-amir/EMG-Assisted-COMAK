# JAM Python examples

Python versions of the MATLAB examples in `Resources/Code/Matlab/JAM` (same folders,
same file names, same settings). They are installed to `Resources/Code/Python/JAM`
together with the shared `models` folder.

## Python environment

Create a conda environment as described in "Scripting in Python" in the OpenSim
documentation, and add `scipy` (used by the EMG, passive flexion and anterior laxity
examples; `matplotlib` is optional and only plots the inputs):

    conda create -n opensim_scripting python=3.11 numpy scipy
    conda activate opensim_scripting

Install the OpenSim Python package of this installation into that environment:

    cd <OPENSIM_INSTALL_DIR>/sdk/Python
    python -m pip install .

`pip install .` also installs numpy and scipy if they are missing (see `setup.py`).
Alternatively, without installing the package, put `<OPENSIM_INSTALL_DIR>/bin` first
on `PATH` and `<OPENSIM_INSTALL_DIR>/sdk/Python` on `PYTHONPATH`.

## Run

Each script finds its files relative to its own folder, so it can be started from
anywhere:

    python Resources/Code/Python/JAM/exampleWalking/exampleWalking.py

The results are written next to the script (`inputs`, `results`). Copy the example
folder and the `models` folder to a writable location first if the installation is
in a protected folder.
