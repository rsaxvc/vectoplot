#!/bin/sh
g++ main.cpp -Wall -g -pg -Og && ./a.out  | tee output.csv | less && ./plot.py
