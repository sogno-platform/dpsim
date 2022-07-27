# Jupyter Extension Matplotlib Widget
pip3 install nodejs
pip3 install ipympl
pip3 install --upgrade jupyterlab
jupyter labextension install @jupyter-widgets/jupyterlab-manager
jupyter labextension install jupyter-matplotlib
jupyter nbextension install --py --symlink ipympl
jupyter nbextension enable --py ipympl

