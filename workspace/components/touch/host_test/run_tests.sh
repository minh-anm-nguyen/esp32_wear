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

run test_cst816t         "$DIR/test_cst816t.cpp"         "$DIR/../cst816t.cpp"
run test_touch_transform "$DIR/test_touch_transform.cpp" "$DIR/../touch_transform.cpp"
run test_touch_tracker   "$DIR/test_touch_tracker.cpp"   "$DIR/../touch_tracker.cpp" "$DIR/../cst816t.cpp" "$DIR/../touch_transform.cpp"
