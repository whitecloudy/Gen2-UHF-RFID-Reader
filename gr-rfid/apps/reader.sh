#! /bin/sh
rm -r debug_data
mkdir debug_data
rm log result cluster_sample chk
python reader.py
cat result
rm a
