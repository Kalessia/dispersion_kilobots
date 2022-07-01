# dispersion_kilobots   
Study of the coverage area problem within a robotic swarm.  

&nbsp;

***To run a simulation on kilombo :***  
- install the kilombo simulator :  
    > **RUN**  
    > git clone https://github.com/JIC-CSB/kilombo.git  
    > cd kilombo ; mkdir build ; cd build  
    > cmake ..  
    > make -j 10  
    > make install  

- from *dispersionKale* folder, **RUN** ./mk_simulation *EXECUTABLEFILENAME*.c

&nbsp;
    
***To build a file.hex to upload on kilobots :***  
- from *dispersionKale* folder, modify the *mk_kilobots* file at line "export KILOHEADERS=" : replace its value by the path to the official kilolib on your computer.  
- from *dispersionKale* folder, **RUN** ./mk_kilobots *EXECUTABLEFILENAME*.c  
- find the *EXECUTABLEFILENAME.c*.hex in the *dispersionKale/build* folder.
