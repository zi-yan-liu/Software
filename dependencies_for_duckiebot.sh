#!/bin/bash
set -e
set -x

# Important: always use the python-X library if available,
# rather than doing "pip install X".

./dependencies_common.sh

pip install --user --upgrade picamera==1.13
