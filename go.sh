#!/bin/sh
g++ main.cpp -Wall -Wno-psabi -g -pg -Og && ./a.out  | tee output.csv | less && ./plot.py
