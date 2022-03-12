#!/bin/bash
set -e
#delete old logs
#rm -f ../log/csv_log/*

# call python script to write csv_logs
cd ../log/csv_log/MEKF
python3 ../../../postProcess/write_csv_MEKF.py