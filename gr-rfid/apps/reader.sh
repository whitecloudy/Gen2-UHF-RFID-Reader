#! /bin/sh
rm -r debug_data
mkdir debug_data
rm debug_message result flip cluster iq cluster_sample chk
python reader.py
cat result
rm a
