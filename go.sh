#!/bin/sh
#profiling -pg
g++ main.cpp -Wall -Wno-psabi -g -O2 && ./a.out 2>logerr.txt | tee output.csv | less && ./plot.py
