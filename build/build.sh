make clean
HOME_DIR=$(realpath "$(dirname "${BASH_SOURCE[0]}")")
node $HOME_DIR/link-build.js
make -j4