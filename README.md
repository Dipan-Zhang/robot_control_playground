# robot_control_playground
simple python control algorithm playground with casadi and pinocchio and visualization with meshcat

## get started (in a terminal, NOT in vscode)
1. Create a python 3.9 conda environment and then activate it
```shell
conda create -n py39 python=3.9.18
conda activate py39
```
2. install acados and requirements 
```shell
./init.sh
pip install -r requirements.txt 
```
3. cd to `acados` and `pip install -e ./interfaces/acados_template`
4. activate the environmental variables and launch jupyterlab
```shell
./start.sh
``` 