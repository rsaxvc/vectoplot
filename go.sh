#!/bin/sh
#profiling -pg
g++ main.cpp -Wall -Wno-psabi -g -O2 && ./a.out 2>logerr.txt > output.csv && ./plot.py
