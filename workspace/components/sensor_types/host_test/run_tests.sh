#!/bin/sh
# Run the pure-logic unit tests on a PC. No ESP-IDF, no chip needed.
set -e
DIR=$(cd "$(dirname "$0")" && pwd)
OUT="$DIR/test_sample_fanout"

g++ -std=c++17 -Wall -Wextra -Werror -O1 \
    -I "$DIR/../include" \
    "$DIR/test_sample_fanout.cpp" \
    -o "$OUT"

"$OUT"
