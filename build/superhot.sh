#!/bin/bash
#This file should be called within a project. Stop if we're in superhot's directory
echo SuperHot v1.0
HOME_DIR=$(realpath "$(dirname "${BASH_SOURCE[0]}")")
USER_DIR=$(pwd)
BUILD_DIR=$USER_DIR/build
rm -rf $BUILD_DIR 2>/dev/null
mkdir $BUILD_DIR
if [[ "$HOME_DIR" == "$USER_DIR" ]]; then
  echo Please run this script from your project.
  exit
fi
echo $BUILD_DIR was selected as the build directory.
cd $BUILD_DIR
#intentionally not -r
cp $HOME_DIR/* $BUILD_DIR/ 2>/dev/null
mkdir $BUILD_DIR/src
ln -s $HOME_DIR/src/* $BUILD_DIR/src/
ln -s $USER_DIR/src $BUILD_DIR/src/user
mkdir $BUILD_DIR/include
ln -s $USER_DIR/include/* $BUILD_DIR/include/
ln -s $HOME_DIR/include/kernel $BUILD_DIR/include/kernel
cp $HOME_DIR/include/superhot_compat.hpp $USER_DIR/include
mkdir $BUILD_DIR/firmware
for file in $USER_DIR/firmware/*.a; do
  cp $file $BUILD_DIR/firmware/$(basename $file)
done
cp $HOME_DIR/firmware/*.ld $BUILD_DIR/firmware/

#Building of the linker script is currently not a thing, but it would be here.
#node build_linker_script.js

#Now build and upload.
cp $USER_DIR/config.json config.json
if ./build.sh; then
  rm $USER_DIR/success_sum 2>/dev/null
  dd if=$BUILD_DIR/bin/hot.package.bin of=$BUILD_DIR/bin/hot.package.bin~ bs=8 skip=1
  cat <(echo -n HAWTSTUF) $BUILD_DIR/bin/hot.package.bin~ > $BUILD_DIR/bin/hot.package.bin
  prosv5 upload $* && shasum $BUILD_DIR/bin/hot.package.bin > $USER_DIR/success_sum
fi
