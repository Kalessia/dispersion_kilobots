# Tmp README for the LIMMS SWARM project

## To clone this repository

```bash
git clone git@bitbucket.org:AubertKato/limmsswarm.git
```

## Installation
Python 3.7+ required.

This project uses the [Kilombo simulator](https://github.com/JIC-CSB/kilombo/blob/master/doc/manual.md).

On Debian/Ubuntu:
```bash
sudo apt-get install -yq rsync gosu build-essential git gcc-avr gdb-avr binutils-avr avr-libc avrdude libsdl1.2-dev libjansson-dev libsubunit-dev cmake check xserver-xorg-video-dummy xserver-xorg-input-void x11-apps
```

Install kilombo:
```bash
git clone https://github.com/JIC-CSB/kilombo.git
cd kilombo && mkdir build && cd build
cmake .. && make -j 10 && sudo make install
```

Install python dependencies (only for stats/plots scripts):
```bash
pip3 install numpy scipy pyaml matplotlib seaborn scikit-learn
```

Compile main executable ("limmsswarm"):
```bash
cd kilo
make clean; make -j 5
```


## Quickstart
Example config files for all arenas are provided in directory `conf/json_examples`. To launch one example, proceed as follow:
```bash
./limmsswarm -p conf/json_examples/square_kilombo.json
```
The GUI window of the Kilombo should open. GUI keybinds are listed in the [Kilombo documentation](https://github.com/JIC-CSB/kilombo/blob/master/doc/manual.md#controls).

The examples file use the same parameters as listed in Table 1 of the SI. All example runs simulate one day of experiment (i.e. 68 iterations).
By default, the LED color emitted by each robot correspond to the current value of FinalLambda, They initially start black, but should converge rapidly towards the colors corresponding to each arena, as seen in Figs. 2 and 3 of the paper.

It is possible for the robots to display other metrics, such as the current values of lambda, or the sign of s. To change the metric displayed, uncomment the corresponding `#define ENABLE_COLOR_FROM_*` line at the beginning of `limmsswarm.h`, and comment out the others. For example, to make the robots display the current sign of s, the `limmsswarm.h` file should begins as follow:
```C
//#define ENABLE_COLOR_FROM_X0
//#define ENABLE_COLOR_FROM_X
#define ENABLE_COLOR_FROM_SIGNX
//#define ENABLE_COLOR_FROM_SIGNDELTAX
//#define ENABLE_COLOR_FROM_LAMBDA
//#define ENABLE_COLOR_FROM_AVGLAMBDA
```
Then recompile `limmsswarm` with the following command:
```bash
make clean; make -j 5
```


## Compile Binaries for real Kilobots
You need to first install kilolib:
```bash
sudo apt install gcc-avr avr-libc binutils-avr
git clone https://github.com/acornejo/kilolib
cd kilolib
make -j 5
```
Then add the following line at the beginning of the `Makefile` file in the `limmsswarm/kilo` directory:
```make
KILOHEADERS=/absolute/path/to/kilolib/
```
or export it to the current shell session:
```bash
export KILOHEADERS=/absolute/path/to/kilolib/
```

Finally, compile the kilobot binary:
```bash
cd limmsswarm/kilo
make hex
```

