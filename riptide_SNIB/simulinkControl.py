import random
from time import sleep
import matlab.engine
from threading import Thread

MODEL_NAME = "Simple_3_Model" # the name of the simulink simulation
MAX_RUN_TIME = "1000" #seconds

def launchSimulink():
    #create a unique name for matlab engine
    name = "rs" + str(round(random.random() * 100000))

    engineThread = Thread(target=babysitMatlabEngine, args=(name, ), name="matlabsitter")
    engineThread.start()

def babysitMatlabEngine(engineName):
    path = '~/osu-uwrt/riptide_software/src/riptide_simulink/Models/Simple_3_Model'
    
    eng = matlab.engine.start_matlab("-desktop")
    eng.matlab.engine.shareEngine(engineName, nargout=0)
    eng.cd(path, nargout=0)
    eng.StartSim(nargout=0)
    
    sleep(10)
    if(eng == None):
        return False
    #run until the engine can no longer be found - ie the window has been closed
    while engineName in matlab.engine.find_matlab():
        sleep(10)


#starts sim - True success, False fail
def startSimulation(eng):
    if(eng == None):
        return False

    #set the sim timeout
    eng.set_param(MODEL_NAME, "StopTime", MAX_RUN_TIME, nargout=0)
    eng.set_param(MODEL_NAME, "SimulationCommand", "start", nargout=0)

    #verify if the command went through
    status = eng.get_param(MODEL_NAME, "SimulationStatus")
    if(status == "running"):
        return True
    
    return False

#stops sim - True success, False fail
def stopSimulation(eng):
    if(eng == None):
        return False

    eng.set_param(MODEL_NAME, "SimulationCommand", "stop", nargout=0)

    #verify if the command went through
    status = eng.get_param(MODEL_NAME, "SimulationStatus")
    if(status == "stopped"):
        return True
    
    return False

#starts sim - True success, False fail
def pauseSimulation(eng):
    if(eng == None):
        return False

    eng.set_param(MODEL_NAME, "SimulationCommand", "pause", nargout=0)

    #verify if the command went through
    status = eng.get_param(MODEL_NAME, "SimulationStatus")
    if(status == "paused"):
        return True
    
    return False


#starts sim - True success, False fail
def continueSimulation(eng):
    if(eng == None):
        return False

    eng.set_param(MODEL_NAME, "SimulationCommand", "continue", nargout=0)

    #verify if the command went through
    status = eng.get_param(MODEL_NAME, "SimulationStatus")
    if(status == "running"):
        return True
    
    return False


