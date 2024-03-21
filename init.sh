#!/bin/bash

# Get the directory of the script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

git clone https://github.com/nmansard/jnrh2023.git
git clone https://github.com/acados/acados.git --branch v0.2.6
cd acados
git submodule update --init --recursive
cmake -B build -S. -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=ON -DACADOS_INSTALL_DIR="$SCRIPT_DIR"/acados
cmake --build build -- -j3
cmake --build build --target install
cd ..
