#!/bin/sh
# Run the pure-logic unit tests on a PC. No ESP-IDF, no chip needed.
set -e
DIR=$(cd "$(dirname "$0")" && pwd)
OUT="$DIR/test_wrist_service"

g++ -std=c++17 -Wall -Wextra -Werror -O1 \
    -I "$DIR/../include" \
    -I "$DIR/../../app_core/include" \
    -I "$DIR/../../motion/include" \
    -I "$DIR/../../sensor_types/include" \
    "$DIR/test_wrist_service.cpp" \
    "$DIR/../../motion/motion_controller.cpp" \
    -o "$OUT"

"$OUT"
