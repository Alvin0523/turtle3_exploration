#!/bin/bash
set -e

cd opencr_update
./update.sh /dev/ttyACM0 waffle.opencr
