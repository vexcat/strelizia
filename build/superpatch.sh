#!/bin/bash
#This file should be called within a project. Stop if we're in superhot's directory
echo SuperHot v1.0
HOME_DIR=$(realpath "$(dirname "${BASH_SOURCE[0]}")")
USER_DIR=$(pwd)
BUILD_DIR=$USER_DIR/build
if [[ "$HOME_DIR" == "$USER_DIR" ]]; then
  echo Please run this script from your project.
  exit
fi
echo $BUILD_DIR was selected as the build directory.
cd $BUILD_DIR

if [[ ! -e "$1" ]]; then
  echo "no such device $1"
  exit
fi

# Build and submit patch. If anything goes wrong, revert everything!
if cmp -n 40 -s <(shasum $BUILD_DIR/bin/hot.package.bin) $USER_DIR/success_sum; then
  cp bin/hot.package.bin backup.bin
  cp $USER_DIR/config.json config.json
  if ./build_user.sh; then
    dd if=bin/hot.package.bin of=bin/hot.package.bin~ bs=8 skip=1
    cat <(echo -n HAWTSTUF) bin/hot.package.bin~ > bin/hot.package.bin
    $HOME_DIR/minibsdiff/minibsdiff gen backup.bin bin/hot.package.bin patch.bin
    rm patch.bingz 2>/dev/null
    rm patch.bingz.gz 2>/dev/null
    #Yes, there is a point to this.
    #gzip, even with -9, just can't cope with large stretches of 0.
    #In the future, we'll be able to send a stop command, individual module patches,
    #then a start command, in so that heap memory isn't unnecessarily wasted by 0s.
    #But for now, sending most of this 22MiB region as a doubly-gzipped file works
    #okay, even if putting the system heap under some stress.
    gzip -9 -k patch.bin
    mv patch.bin.gz patch.bingz
    gzip -9 -k patch.bingz
    cat <(echo -n +) <(./quirky_escape patch.bingz.gz) <(echo) > final_patch.bin
    if dd if=final_patch.bin of=$1; then
      shasum $BUILD_DIR/bin/hot.package.bin > $USER_DIR/success_sum
    else
      mv backup.bin bin/hot.package.bin
    fi
  else
    mv backup.bin bin/hot.package.bin
  fi
else
  echo "Cannot patch. Remote and local copies differ."
fi