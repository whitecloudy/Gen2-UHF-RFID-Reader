#! /bin/sh
rm -r debug_data
mkdir debug_data
rm log result
python reader.py
cat result
rm a
