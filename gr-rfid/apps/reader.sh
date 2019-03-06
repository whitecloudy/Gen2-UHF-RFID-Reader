#! /bin/sh
rm -r debug_data
mkdir debug_data
rm debug_message result flip cluster iq
python reader.py
cat result
rm a
