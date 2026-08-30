#!/bin/sh
# Run the pure-logic unit tests on a PC. No ESP-IDF, no chip needed.
set -e
DIR=$(cd "$(dirname "$0")" && pwd)

run() {
    name=$1; shift
    g++ -std=c++17 -Wall -Wextra -Werror -O1 \
        -I "$DIR/../include" \
        "$@" -o "$DIR/$name"
    "$DIR/$name"
}

run test_geometry     "$DIR/test_geometry.cpp"     "$DIR/../display.cpp"
run test_power_policy "$DIR/test_power_policy.cpp" "$DIR/../display.cpp"
