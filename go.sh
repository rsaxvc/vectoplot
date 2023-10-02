#!/bin/sh
g++ main.cpp -Wall -g -Og && ./a.out  | tee output.csv | less && ./plot.py
