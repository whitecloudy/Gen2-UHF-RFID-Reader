#! /bin/sh
git pull origin parallel_decoding
cd ../build
make
make test
sudo make install
sudo ldconfig
cd ../apps
