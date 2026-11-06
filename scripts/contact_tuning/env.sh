# edit for your build: PATH with the OpenSim DLLs and Simbody, PYTHONPATH with the opensim package, $PY the python
B=/path/to/build
export PATH="$B/RelWithDebInfo:$PATH"
export PYTHONPATH="$B/Bindings/Python/RelWithDebInfo"
export PY=python
