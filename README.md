# dispersion_kilobots   
Study of the coverage area problem within a robotic swarm.  

&nbsp;

***System :*** 
- Ubuntu 20.04.5 LTS
- Python 3.9.13

&nbsp;

***To run a simulation on kilombo :***  
- install the kilombo simulator :  
    > **RUN**  
    > git clone https://github.com/JIC-CSB/kilombo.git  
    > cd kilombo ; mkdir build ; cd build  
    > cmake ..  
    > make -j 10  
    > make install  

- from *dispersion* folder, **RUN** ./mk_simulation *EXECUTABLEFILENAME*.c

&nbsp;
    
***To build a file.hex to upload on kilobots :***  
- from *dispersion* folder, modify the *mk_kilobots* file at line "export KILOHEADERS=" : replace its value by the path to the official kilolib on your computer.  
- from *dispersion* folder, **RUN** ./mk_kilobots *EXECUTABLEFILENAME*.c  
- find the *EXECUTABLEFILENAME.c*.hex in the *dispersion/build* folder.

&nbsp;

***To run an analysis :***
- run a simulation on Kilombo
- run *dispersion_kilobots/simulationAnalysis/simulationAnalysis_nonRestrictedVoronoi.py* for not limited Voronoi regions
  or *dispersion_kilobots/simulationAnalysis/simulationAnalysis_restrictedVoronoi.py* for limited Voronoi regions
- find results and plot in *dispersion_kilobots/simulationAnalysis/results*  
