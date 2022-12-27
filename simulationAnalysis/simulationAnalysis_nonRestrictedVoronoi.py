###############################################################
# SimulationAnalysis_nonRestrictedVoronoi
###############################################################

# This file allows to obtain analyses about the quality of the dispersion 
# of a swarm of kilobots. To characterise the dispersion state, we use a 
# metric based on Voronoi regions.
# The arena is not represented, there aren't boundaries : it means that
# Voronoi regions areas are illimited for those regions related to 
# kilobots' positions in the external perimeter of the arena shape.  

# Results are saved on a csv file.




###############################################################
# Imports
###############################################################

import os

from datetime import datetime
from time import strftime

import numpy as np

import json
import csv

import matplotlib.pyplot as plt

from scipy.spatial import Voronoi, ConvexHull, voronoi_plot_2d




###############################################################
# Parameters
###############################################################

# Set the 'simulationStates.json' or the 'endstate.json' path
# Select the preferred path (only one) to get kilobot's positions data from kilombo's json file
#statesJsonPath = 'dispersion/endstate.json' # final state of the entire simulation
statesJsonPath = 'dispersion/simulationStates.json' # intermediate states of the simulation 





###############################################################
# Functions
###############################################################

def drawVoronoi(saveFileName, nSimulation, ticksSimulation, points, showPlot=False):

    vor = Voronoi(points)
    fig = voronoi_plot_2d(vor)
    plt.savefig(saveFileName + "/nSim=" + str(nSimulation) + "_ticks=" + str(ticksSimulation) + ".png")
    
    if showPlot:
        plt.show()

#--------------------------------------------------------------

def voronoiVolumes(points):

    vor = Voronoi(points)
    volumes = np.zeros(vor.npoints)
    for i, nRegion in enumerate(vor.point_region): # point_region : index of the region associated with a given point
        indices = vor.regions[nRegion] # regions : used to get indices of the Voronoi vertices associated with the region
        if -1 in indices :
            volumes[i] = np.inf
        else :
            volumes[i] = ConvexHull(vor.vertices[indices]).volume # vertices : used to get coordinates of the Voronoi vertices
        
    return volumes

# source : https://stackoverflow.com/questions/68747267/how-to-link-initial-points-coordinates-to-corresponding-voronoi-vertices-coordin
# source : https://stackoverflow.com/questions/19634993/volume-of-voronoi-cell-python

#--------------------------------------------------------------

def getDataFromJsonFile(path):

    with open(path, 'r') as f:
        data = json.load(f)

    points = []
    ticks = []

    if 'endstate' in path:

        # 'data' is a dict containing 2 keys : 'bot_states' and 'ticks'
        # 'data['bot_states']' is a list of botStates
        # 'botStates' is a dict containing the following keys : 'ID', 'direction', 'state', 'x_position', 'y_position'

        pointsSimulation = []
        for botStates in data['bot_states']:
            pointsSimulation.append([ botStates['x_position'], botStates['y_position'] ])

        points.append(pointsSimulation)
        ticks.append(data['ticks'])

    else :
    
        # 'data' is a list of simulations (dict)
        # 'simulation' is a dict containing 2 keys : 'bot_states' and 'ticks'
        # 'simulation['bot_states']' is a list of botStates
        # 'botStates' is a dict containing the following keys : 'ID', 'direction', 'state', 'x_position', 'y_position'

        for simulation in data:
            pointsSimulation = []
            for botStates in simulation['bot_states']:
                pointsSimulation.append([ botStates['x_position'], botStates['y_position'] ])

            points.append(pointsSimulation)
            ticks.append(simulation['ticks'])

    return points, ticks

#--------------------------------------------------------------

def saveDataToCsv(saveFileName, listVolumesRegions):
    with open(saveFileName + "/voronoiData.csv", 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow("Region" + str(i) for i in range(len(listVolumesRegions[0])))

        for v in listVolumesRegions:
            writer.writerow(v)

#--------------------------------------------------------------






###############################################################
# Main
###############################################################

# Getting kilobot's positions data from kilombo's json file
points, ticks = getDataFromJsonFile(statesJsonPath)


# New folder to collect voronoi plots and voronoi's volumes data
saveFileName = "simulationAnalysis/simVoronoi_" + datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
os.mkdir(saveFileName)


# Computing and saving data in 'simulationAnalysis' folder
listAreaRegions = []
for i in range(len(points)):
    volumesRegions = voronoiVolumes(np.array(points[i]))
    listAreaRegions.append(volumesRegions)

    drawVoronoi(saveFileName, i, ticks[i], np.array(points[i]), showPlot=False)

saveDataToCsv(saveFileName, listAreaRegions)









