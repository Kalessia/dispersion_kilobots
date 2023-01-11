###############################################################
# SimulationAnalysis_restrictedVoronoi
###############################################################

# This file allows to obtain analyses about the quality of the dispersion 
# of a swarm of kilobots. To characterise the dispersion state, we use a 
# metric based on Voronoi regions.

# Phases:
# 1) Construction of an arena-like polygon, disk or annular shape
# 2) Collection of points representing the positions of the kilobots 
#    at a given tick of the simulation, taken from one of the following files:
#        - 'simulationStates.json'  (intermediate states of the simulation) 
#        - 'endstate.json'          (final state of the entire simulation)
# 3) Calculation of the Voronoi regions, and their areas (voronoi_regions_from_coords function)
# 4) Calculation of the difference between the regions average areas and the
#    area of each single region (computeEvalDispersion function)
# 5) Other statistics : standard deviation, ...
# 6) Plots and save results

# NB. Do not modify the 'simulation.json' file between the Kilombo's simulation 
# and the analysis, as some of the information useful for analysis is retrieved 
# from the file. 

# NB. Check and set the parameters in the 'Parameters' section.




###############################################################
# Imports
###############################################################

import os
import shutil

from time import sleep
from datetime import datetime
from time import strftime

import numpy as np
import geopandas as gpd
from geovoronoi import voronoi_regions_from_coords

import json
import csv

from shapely.geometry import Point

import matplotlib.pyplot as plt

import warnings
warnings.filterwarnings('ignore')





###############################################################
# Parameters
###############################################################

# Set the radius of the arena's shapes, in millimetres
diskRadius_mm = 150
ringExtRadius_mm = 200 # diam 391
ringIntRadius_mm = 132 # diam 259


# Set the 'simulationStates.json' or the 'endstate.json' path
# Select the preferred path (only one) to get kilobot's positions data from kilombo's json file
#statesJsonPath = 'dispersion/endstate.json' # final state of the entire simulation
statesJsonPath = 'dispersion/simulationStates.json' # intermediate states of the simulation 

# Vidéos
#statesJsonPath = 'positions.json'

# Set the 'simulation.json' path
simulationJsonPath = 'dispersion/simulation.json'

showPlot = False # Set 'True' to see plots during the execution





###############################################################
# Functions
###############################################################

def plot_polygon(ax, polyToPlot, color):
    poly = gpd.GeoSeries([polyToPlot])
    ax.autoscale_view()
    poly.plot(ax=ax, color=color)

#--------------------------------------------------------------

def drawRestrictedVoronoi(saveFileName, nSimulation, ticksSimulation, poly, region_polys, gdfPoints, gdfOrigin, showPlot=False):

    # Customised colormap
    N = 256
    matRGB = np.ones((N,4)) # matrice RGBA 256 x 4
    r = g = b = 160 # color range, light gray palette
    matRGB[:, 0] = np.linspace(r/N, 1, N) # red component to white
    matRGB[:, 1] = np.linspace(g/N, 1, N) # blue component to white
    matRGB[:, 2] = np.linspace(b/N, 1, N) # green component to white


    fig, ax = plt.subplots(figsize=(12, 10))

    plot_polygon(ax, poly, color = 'black')
    for i in range(len(region_polys)):
        plot_polygon(ax, region_polys[i], color = matRGB[i*int(N/len(region_polys))]) # plot Voronoi's regions' polygons

    gdfPoints.plot(ax=ax, markersize=10, color='red') # plot kilobots' positions
    gdfOrigin.plot(ax=ax, marker="+", markersize=1000, color='black') # plot the origin point of the polygon's shape

    ax.axis('off')
    plt.axis('equal')
    plt.title("Sim=" + str(nSimulation) + ", ticks=" + str(ticksSimulation))

    plt.savefig(saveFileName + "/nSim=" + str(nSimulation) + "_ticks=" + str(ticksSimulation) + ".png")
    
    if showPlot:
        plt.show()

#--------------------------------------------------------------

def getBotsPositionsFromJsonFile(path):

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

        data = data[1:] # we don't want the simulation 0 in order to avoid problems with kilobots initial positions (they come out of the polygon)        
        for simulation in data:
            pointsSimulation = []
            for botStates in simulation['bot_states']:
                pointsSimulation.append([ botStates['x_position'], botStates['y_position'] ])

            points.append(pointsSimulation)
            ticks.append(simulation['ticks'])

    return points, ticks

#--------------------------------------------------------------

def saveDataToCsv(saveFileName, ticks, listAreaRegionsTicks):
    with open(saveFileName + "/data_voronoiData.csv", 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Tick"] + ["Region" + str(i) for i in range(len(listAreaRegionsTicks[0]))])

        for i in range(len(listAreaRegionsTicks)):
            writer.writerow([ticks[i]] + listAreaRegionsTicks[i])

#--------------------------------------------------------------

def getInfoFromSimulationJson(path):
    with open(path, 'r') as f:
        simulationData = json.load(f)
    
    return simulationData

#--------------------------------------------------------------

def buildPolygonShape(shapePath, arenaNormalizedArea, diskRadius_mm, ringExtRadius_mm, ringIntRadius_mm):

    print("\n---------------------------------------------------------------")
    if 'disk' in shapePath:
        poly = Point((0, 0)).buffer(diskRadius_mm)

        print("Python Polygon diskRadius in mm :", diskRadius_mm)

    elif 'annulus' in shapePath:
        poly1 = Point((0, 0)).buffer(ringExtRadius_mm)
        poly2 = Point((0, 0)).buffer(ringIntRadius_mm)
        poly = poly1.difference(poly2)

        print("Python Polygon ringExtRadius in mm :", ringExtRadius_mm)
        print("Python Polygon ringIntRadius in mm :", ringIntRadius_mm)
    
    else:
        print("Error in buildPolygonShape : unrecognised shape")
        return None

    print("\nPython Polygon area in mm : ", poly.area)
    print("Kilombo 'arenaNormalizedArea' in simulation.json : ", arenaNormalizedArea)
    print("---------------------------------------------------------------\n")

    return poly

#--------------------------------------------------------------

def computeDistsFromAreaRef(listAreaRegionsTicks):
    distsFromAreaRef = []

    for tick in range(len(listAreaRegionsTicks)):
        areaRef = np.mean(listAreaRegionsTicks[tick]) # 1st option : areaRef is the average of the areas of the different regions

        tmp = []
        for area in listAreaRegionsTicks[tick]:
            tmp.append(area - areaRef)

        distsFromAreaRef.append(tmp)

    return areaRef, distsFromAreaRef

#--------------------------------------------------------------

def saveDistsFromAreaRefToCsv(saveFileName, ticks, polyArea, nbBots, areaRef, distsFromAreaRef):
    with open(saveFileName + "/data_dispersionEvaluation.csv", 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Tick"] + ["AreaPerBot"] + ["AreaRef"] + ["Region" + str(i) for i in range(len(distsFromAreaRef[0]))])

        areaPerBot = polyArea / nbBots
        for i in range(len(distsFromAreaRef)):
            writer.writerow([ticks[i]] + [areaPerBot, areaRef] + distsFromAreaRef[i])

#--------------------------------------------------------------

def computeStdRegionsPerTick(listAreaRegionsTicks):
    sigma = []

    for tick in range(len(listAreaRegionsTicks)):
        sigma.append(np.std(listAreaRegionsTicks[tick]))

    return sigma

#--------------------------------------------------------------

def plotDistsFromAreaRef(saveFileName, ticks, areaRef, listAreaRegionsTicks, nRegions, showPlot=False):
    x = [i for i in range(nRegions)]
    
    for tick in range(len(listAreaRegionsTicks)):
        y = listAreaRegionsTicks[tick]
        plt.plot(x, y, label = ticks[tick])

    plt.axhline(areaRef, color = 'red', linestyle = '-')
    plt.title("Distances of " + str(nRegions) + " Voronoi Regions from the AreaRef Per Tick\n AreaRef (red line) = " + str(areaRef))
    plt.xlabel("region id (not related to kilobots id)")
    plt.ylabel("region area (mm)")
    plt.legend(loc = 'upper right')
    plt.savefig(saveFileName + "/plot_distsFromAreaRef.png")
    
    if showPlot:
        plt.show()

#--------------------------------------------------------------

def plotSigma(saveFileName, ticks, sigma, nRegions, showPlot=False):
    plt.plot(ticks, sigma)
    plt.title("Standard Deviation of " + str(nRegions) + " Voronoi Regions Per Tick")
    plt.xlabel("ticks")
    plt.ylabel("sigma (mm)")
    plt.savefig(saveFileName + "/plot_sigma.png")
    
    if showPlot:
        plt.show()

#--------------------------------------------------------------











###############################################################
# Main
###############################################################

print("\nAnalysing positions in", statesJsonPath)

# New folder to collect voronoi plots and voronoi's areas data
os.makedirs('simulationAnalysis/results', exist_ok=True)
os.makedirs('simulationAnalysis/results/restrictedVoronoi', exist_ok=True)

saveFileName = "simulationAnalysis/results/restrictedVoronoi/simVoronoi_" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
os.mkdir(saveFileName)
os.mkdir(saveFileName + "/plots_Voronoi")

# Save a copy of parameters file and kilobots positions file
shutil.copy(statesJsonPath, saveFileName) 
shutil.copy(simulationJsonPath, saveFileName) 

# Getting information from the simulation, contained in the simulation.json file
simulationData = getInfoFromSimulationJson(simulationJsonPath)

# Getting kilobot's positions data from kilombo's json file
pointsPerTick, ticks = getBotsPositionsFromJsonFile(statesJsonPath)

# Building the arena boundaries
poly = buildPolygonShape(simulationData['arenaFileName'], simulationData['arenaNormalizedArea'], diskRadius_mm, ringExtRadius_mm, ringIntRadius_mm)

# Definition of the origin point in the shape (to plot purpose)
gdfOrigin = gpd.GeoDataFrame({'col1': ["origin"], 'geometry': [Point(0.,0.)]}, crs="EPSG:4326")


# Computing and saving data in 'simulationAnalysis' folder
listAreaRegionsTicks = []
for index in range(len(ticks)):

    # convert kilobot's positions in a geodataframe
    pts = [Point(xy) for xy in pointsPerTick[index]]
    tmp = {'col1': ["kilobot_" + str(k) for k in range(len(pts))], 'geometry': pts}
    gdfPoints = gpd.GeoDataFrame(tmp, crs="EPSG:4326")

    # compute Voronoi's regions
    region_polys, region_pts = voronoi_regions_from_coords(pts, poly)

    # collect the Voronoi's regions' areas in the current tick
    listAreaRegions = []
    for idRegion in region_polys.keys():
        listAreaRegions.append(region_polys[idRegion].area)

    # collect the Voronoi's regions' areas of all ticks
    listAreaRegionsTicks.append(listAreaRegions)

    # plot the Voronoi's regions within the polygon shape and save the figure
    nSim = index
    if len(ticks) > 1:
        nSim = index + 1
    drawRestrictedVoronoi(saveFileName + "/plots_Voronoi", nSim, ticks[index], poly, region_polys, gdfPoints, gdfOrigin, showPlot)


plt.close('all')

# save the Voronoi's regions' areas
saveDataToCsv(saveFileName, ticks, listAreaRegionsTicks)

# for each region, compute and save the difference between its area and the optimal area (areaRef)
areaRef, distsFromAreaRef = computeDistsFromAreaRef(listAreaRegionsTicks)
saveDistsFromAreaRefToCsv(saveFileName, ticks, poly.area, simulationData['nBots'], areaRef, distsFromAreaRef)

# Distances from the AreaRef : each points represent a region area, each line represents the list of regions in a tick.
# We would like to see the entire line get closer to AreaRef, with a minimal distance variation of its points.
plotDistsFromAreaRef(saveFileName, ticks, areaRef, listAreaRegionsTicks, len(region_polys), showPlot)

# Standard deviation (sigma) : as a function of time ticks, we would like to see sigma get closer to zero
sigma = computeStdRegionsPerTick(listAreaRegionsTicks)
plotSigma(saveFileName, ticks, sigma, len(region_polys), showPlot)

print("The analysis is complete. Please find the data and plots in the", saveFileName, "directory.\n")

if showPlot:
    sleep(3)
    plt.close('all')
