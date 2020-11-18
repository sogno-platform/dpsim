# Jupyter Extension Matplotlib Widget
pip3 install --user nodejs
pip3 install --user ipympl
pip3 install --user --upgrade jupyterlab
jupyter labextension install @jupyter-widgets/jupyterlab-manager
jupyter labextension install jupyter-matplotlib
jupyter nbextension install --py --symlink --user ipympl
jupyter nbextension enable --py --user ipympl
