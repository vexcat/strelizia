#!/bin/bash
rm bin/*.o
make -j4
prosv5 upload

