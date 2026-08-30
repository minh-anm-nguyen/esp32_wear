#!/bin/sh
# Run the pure-logic unit tests on a PC. No ESP-IDF, no chip needed.
set -e
DIR=$(cd "$(dirname "$0")" && pwd)
OUT="$DIR/test_motion"

g++ -std=c++17 -Wall -Wextra -Werror -O1 \
    -I "$DIR/../include" \
    -I "$DIR/../../sensor_types/include" \
    "$DIR/test_motion.cpp" "$DIR/../motion_controller.cpp" \
    -o "$OUT"

"$OUT"
