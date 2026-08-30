#!/bin/sh
# Run the pure-logic unit tests on a PC. No ESP-IDF, no chip needed.
set -e
DIR=$(cd "$(dirname "$0")" && pwd)

run() {
    name=$1; shift
    g++ -std=c++17 -Wall -Wextra -Werror -O1 \
        -I "$DIR/../include" \
        -I "$DIR/../../i2c_bus/include" \
        "$@" -o "$DIR/$name"
    "$DIR/$name"
}

run test_qmi8658   "$DIR/test_qmi8658.cpp"   "$DIR/../qmi8658.cpp"
run test_axis_remap "$DIR/test_axis_remap.cpp" "$DIR/../axis_remap.cpp"
