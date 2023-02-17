import random
from time import sleep
import matlab.engine
from threading import Thread
import os

MODEL_NAME = "Simple_3_Model" # the name of the simulink simulation
MAX_RUN_TIME = "1000" #seconds

def getMatlabEngine():
    # return ifFound, engine

    #look for matlab sessions
    session_names = matlab.engine.find_matlab()
    
    if(len(session_names) != 0):
        #if there is a session connect
        for name in session_names:
            if "SNIBulink" in name:
                return True, matlab.engine.connect_matlab(name)
    
    return False, None
        

def launchSimulink(foreground):
    #foreground -- if true run in new terminal

    #don't launch if matlab is already running -- 
    ifEngine, eng = getMatlabEngine()
    if ifEngine:
        prepMatlabEngine(eng)
        return

    if(foreground):
        os.system("gnome-terminal --window -- python3 ~/osu-uwrt/development/software/src/riptide_SNIB/riptide_SNIB/simulinkControl.py")
        return

    #create a unique name for matlab engine
    name = "SNIBulink" + str(round(random.random() * 100000))

    engineThread = Thread(target=babysitMatlabEngine, args=(name, ), name="matlabsitter")
    engineThread.start()

def prepMatlabEngine(eng):
    #called when a previous matlab engine is used

    try:
        #ensure the simulation is running
        eng.get_param(MODEL_NAME, "SimulationStatus")
    except:
        #SNIB engine running but not in sim mode
        path = '~/osu-uwrt/development/software/src/riptide_simulink/Models/Simple_3_Model'
        eng.cd(path, nargout=0)
        eng.StartSim(nargout=0)

def launchSimulinkForeground():
    # launch in different window

    #don't launch if matlab is already running -- 
    ifEngine, _ = getMatlabEngine()
    if ifEngine:
        return

    #create a unique name for matlab engine
    name = "SNIBulink" + str(round(random.random() * 100000))
    path = '~/osu-uwrt/development/software/src/riptide_simulink/Models/Simple_3_Model'

    eng = matlab.engine.start_matlab("-desktop")
    eng.matlab.engine.shareEngine(name, nargout=0)
    eng.cd(path, nargout=0)
    eng.StartSim(nargout=0)
    
    sleep(10)
    if(eng == None):
        return False
    #run until the engine can no longer be found - ie the window has been closed
    while name in matlab.engine.find_matlab():
        sleep(10)

def babysitMatlabEngine(engineName):
    path = '~/osu-uwrt/development/software/src/riptide_simulink/Models/Simple_3_Model'
    
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

def getSimulationStatus(eng):
    # get the current simulink status
    # also functions as a wait until loaded
    status = eng.get_param(MODEL_NAME, "SimulationStatus")
    return status



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


if __name__ == "__main__":
    #launch a new window when called by default
    launchSimulinkForeground()

