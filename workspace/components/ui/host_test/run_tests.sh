#!/bin/sh
# Run the pure-logic unit tests on a PC. No ESP-IDF, no LVGL, no chip needed.
#
# Only the headers that avoid lvgl.h are testable here: flush_coordinator.hpp,
# ui_command.hpp and nav_stack.hpp. app.hpp, app_host.hpp, launcher.hpp and
# ui_manager.hpp need LVGL and the display driver, and are covered by the build
# plus on-board bring-up.
set -e
DIR=$(cd "$(dirname "$0")" && pwd)

for t in test_flush_contract test_nav_stack; do
    g++ -std=c++17 -Wall -Wextra -Werror -O1 \
        -I "$DIR/../include" \
        "$DIR/$t.cpp" \
        -o "$DIR/$t"
    "$DIR/$t"
done
