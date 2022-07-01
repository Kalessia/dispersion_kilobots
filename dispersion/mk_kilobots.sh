#!/bin/sh

cp $1 dispersion.c
echo
echo ">>> Kilobots dispersion - Making " $1
date
echo

export KILOHEADERS=/path/to/your/kilolib
make clean
make dispersion

echo
echo ">>> Kilobots dispersion - Getting .hex..."
make hex
cp dispersion.hex build/$1.hex

rm dispersion.c
make clean
