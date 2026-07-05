#!/bin/bash

SOURCE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
echo "export PATH=\"${SOURCE_DIR}/src/curie_hw_control/lib/spark_mmrt/bin:\$PATH"\" >> "$HOME/.bashrc"