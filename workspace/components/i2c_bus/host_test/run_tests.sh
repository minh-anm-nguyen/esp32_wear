#!/bin/sh
# Run the pure-logic unit tests on a PC. No ESP-IDF, no chip needed.
set -e
DIR=$(cd "$(dirname "$0")" && pwd)
OUT="$DIR/test_i2c_bus"

g++ -std=c++17 -Wall -Wextra -Werror -O1 \
    -I "$DIR/../include" \
    "$DIR/test_i2c_bus.cpp" \
    -o "$OUT"

"$OUT"
