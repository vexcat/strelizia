#!/bin/bash
rm bin/*.o
if make -j4; then
prosv5 upload . --execute /dev/rfcomm0
fi
