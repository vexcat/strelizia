#!/bin/bash
if ! cmp -s last_build_mode <(echo auto); then
  echo auto > last_build_mode
  rm -rf bin
  mkdir bin
fi
if make IS_LIBRARY=1; then
echo Uploading program
v5talk /dev/ttyACM0 upload bin/cold.package.bin strel_cold.bin --checkAlreadySent --vid=pros --address=0x03800000
v5talk /dev/ttyACM0 upload bin/hot.package.bin slot_1.bin --vid=user --address=0x07800000 --link=strel_cold.bin --linkVID=pros --onComplete=run
fi
