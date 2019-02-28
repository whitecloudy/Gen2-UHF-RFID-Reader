#! /bin/sh
git pull origin 190220_parallel
cd ../build
make
make test
sudo make install
sudo ldconfig
cd ../apps
