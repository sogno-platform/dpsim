# helper script to start jupyter notebook in dpsim-dev container
# assumes dpsim and data-processing to be mounted in /dpsim-dev
cd /dpsim-dev/dpsim/build
export PYTHONPATH="$(pwd)/Source/Python:$(pwd)/../Source/Python"
cd /dpsim-dev/dpsim
jupyter lab --ip="0.0.0.0" --allow-root --no-browser