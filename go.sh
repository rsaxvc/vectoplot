#!/bin/sh
g++ main.cpp -Wall -Og && ./a.out  | tee output.csv | less
