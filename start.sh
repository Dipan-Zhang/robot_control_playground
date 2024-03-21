#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Set the ACADOS_SOURCE_DIR variable
export ACADOS_SOURCE_DIR="$SCRIPT_DIR/acados"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$SCRIPT_DIR/acados/lib"

jupyter-lab
