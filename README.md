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

***To run a Kilombo simulation analysis :***
- run a simulation on Kilombo
- Check/set parameters and **RUN** *dispersion_kilobots/simulationAnalysis/simulationAnalysis_nonRestrictedVoronoi.py* for not limited Voronoi regions
  or *dispersion_kilobots/simulationAnalysis/simulationAnalysis_restrictedVoronoi.py* for limited Voronoi regions
- find results and plot in *dispersion_kilobots/simulationAnalysis/results*  

&nbsp;

***To run a video analysis :***

This video analysis only works with the ISIR laboratory settings with the camera fully zoomed out.
- put the videos you want to analyze in the path *./data/videos/[date_exp]* (date_exp is the date when the videos were shot and is in the format YYYY-MM-DD)
- **all video names must end with the type of arena ("disk" or "ring")**
- change the variable *nb_k* by the number of kilobots in the videos of the experiments (**the code assumes that it is the same number in all the videos**)
- now, you can run the python file *videoAnalysis.py*

The different files obtained after analysis of the videos :
- the frames of the video will be put in the folder *./data/frames/[date_exp]/[name_video]*
- the processed images will be put in the folder *./data/images/[name_video]*
  - the *be* folder is for images with black edge detection,
  - *kb* is for kilobots detection,
  - *rs* is for images where shadows have been removed (this function exists, but is not used to do the analysis),
  - and the image that is not in a folder is the one for the arena detection
- the robots' positions are saved in a text file in *./data/txt/[name_exp]/name_video.txt*
  - the first line is: number of kilobots, x,y coordinates of the arena, its radius (if it is a ring, there will be the coordinates and the radius of the lower and upper disk, if not, the coordinates and the radius of the upper disk are -1)
  - the other lines are: the frame number and the coordinates of a kilobot
  - all coordinates are relative to the frame and not to the center of the arena
- the positions of the kilobots in relation to the center of the arena is put in a json file *./data/json/[name_exp]/name_video.json* to then do the analysis

Here are all the libraries used: time, cv2, os, math, shutil, json
