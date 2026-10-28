#!/usr/bin/env sh
# Build and run the host tests for the slcan core. Usage: test/host/run.sh
# Needs a C++17 compiler: g++ by default, or e.g.  CXX="zig c++" test/host/run.sh  on Windows
# (the pty emulator is Linux-only and is skipped elsewhere).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../../src/slcan"
OUT="${BUILD_DIR:-$HERE/build}"
CXX="${CXX:-g++}"
mkdir -p "$OUT"
CXXFLAGS="-std=c++17 -Wall -Wextra -Werror -O1 -I$SRC"
$CXX $CXXFLAGS "$HERE/test_slcan.cpp" "$SRC/slcan.cpp" -o "$OUT/test_slcan"
$CXX $CXXFLAGS -I"$SRC/../mt6701" "$HERE/test_mt6701.cpp" "$SRC/../mt6701/mt6701.cpp" "$SRC/slcan.cpp" \
    -o "$OUT/test_mt6701"
if [ "$(uname -s)" = "Linux" ]; then
    $CXX $CXXFLAGS "$HERE/pty_bridge.cpp" "$SRC/slcan.cpp" -o "$OUT/pty_bridge" -lutil
fi
"$OUT/test_slcan"
"$OUT/test_mt6701"
# Compile-only check of the ESP32 glue against stub headers that mirror the real ESP-IDF / Arduino
# declarations (struct fields, brace-initialiser macros, HWCDC methods). It is built the way the
# Arduino core builds it: GNU dialect (designated initialisers), once per Arduino core major
# version, because set_led() differs between them. This catches typos and wrong usage of those
# APIs; only a real build (Arduino IDE) proves the link and the board configuration.
for major in 2 3; do
    $CXX -std=gnu++17 -Wall -Wextra -Werror -c -DSTUB_ARDUINO_MAJOR=$major \
        -I"$HERE/stubs" -I"$SRC" "$HERE/../../src/main.cpp" \
        -o "$OUT/main_check_$major.o"
done
echo "main.cpp compile check: ok (Arduino core 2.x and 3.x branches)"
# The same check for board C's sketch (firmware/joint_encoder), which uses the shared mt6701 code
# from src/mt6701 the way its Arduino build does (as a library on the include path). Built with the
# defaults and once more with other values, so every branch of its JOINT_ID / CAN_BITRATE checks runs.
JE="$HERE/../../../joint_encoder/src"
for defs in "" "-DJOINT_ID=7 -DCAN_BITRATE=1000000" "-DJOINT_ID=1 -DCAN_BITRATE=125000"; do
    $CXX -std=gnu++17 -Wall -Wextra -Werror -c -DSTUB_ARDUINO_MAJOR=3 $defs \
        -I"$HERE/stubs" -I"$SRC/../mt6701" "$JE/main.cpp" -o "$OUT/joint_encoder_check.o"
done
echo "joint_encoder main.cpp compile check: ok"
