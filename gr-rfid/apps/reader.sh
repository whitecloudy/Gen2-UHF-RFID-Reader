#! /bin/sh
rm -r debug_data
rm iq_log.csv
rm gateOpenTracker/*
mkdir debug_data
cd debug_data
mkdir log
mkdir RN16_input RN16_preamble RN16_sample
mkdir EPC_input EPC_preamble EPC_sample
cd ../
rm log result time.csv
python reader.py
cat result
