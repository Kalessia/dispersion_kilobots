cp $1 dispersion.c
echo "Kilobots dispersion - Making " $1
date
echo

make clean
make dispersion
rm dispersion.c

./dispersion -p simulation.json
