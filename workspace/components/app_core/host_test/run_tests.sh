#!/bin/sh
# Run the pure-logic unit tests on a PC. No ESP-IDF, no chip needed.
set -e
DIR=$(cd "$(dirname "$0")" && pwd)
OUT="$DIR/test_topic"

g++ -std=c++17 -Wall -Wextra -Werror -O1 \
    -I "$DIR/../include" \
    "$DIR/test_topic.cpp" \
    -o "$OUT"

"$OUT"
