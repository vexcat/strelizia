#!/bin/bash
if ! cmp -s last_build_mode <(echo user); then
  echo user > last_build_mode
  rm -rf bin
  mkdir bin
fi
cp bin/cougarImage.c.o cougar.o
rm -f bin/*.o
mv cougar.o bin/cougarImage.c.o
#cougarImage.c is intentionally left out.
if make -j4 IS_LIBRARY=1 EXCLUDE_SRC_FROM_LIB="$(ls ./src/*.cpp | tr '\n' ' ' | rev | cut -c 2- | rev)"; then
v5talk /dev/ttyACM0 transfer
v5talk /dev/ttyACM0 upload bin/cold.package.bin strel_cold.bin --checkAlreadySent --vid=pros --address=0x03800000
rm -f bin/hot.package.bin.gz
gzip -9 -k bin/hot.package.bin
v5talk /dev/ttyACM0 upload bin/hot.package.bin.gz slot_1.bin --vid=user --address=0x07800000 --link=strel_cold.bin --linkVID=pros --onComplete=run
v5talk /dev/ttyACM0 transfer --to=pit
echo Successful Compilation at $(date +%T)
fi
