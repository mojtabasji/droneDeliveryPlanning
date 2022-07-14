import sys
import json
import optparse
from sumolib import checkBinary
import traci
import time


def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true",
                          default=False, help="run the commandLine version of sumo")
    options, args = opt_parser.parse_args()
    return options

line_1_Cells = []
line_2_Cells = []
line_3_Cells = []
line_4_Cells = []
line_5_Cells = []
line_7_Cells = []
line_8_Cells = []
line_9_Cells = []
line_10_Cells = []

def loc2point(inp):
    return (int(inp[0]/10), int(inp[1] /10))

def run():

    '''   ************* get lines on gride  '''
    step = 0
    start_time = time.time()
    traci.simulationStep()

    L1_tracking = True
    L2_tracking = True
    L3_tracking = True
    L4_tracking = True
    L5_tracking = True
    L7_tracking = True
    L8_tracking = True
    L9_tracking = True
    L10_tracking = True
    
    L1_lastPoint = loc2point(traci.vehicle.getPosition("39"))
    L2_lastPoint = loc2point(traci.vehicle.getPosition("52"))
    L3_lastPoint = loc2point(traci.vehicle.getPosition("78"))
    L4_lastPoint = loc2point(traci.vehicle.getPosition("0"))
    L5_lastPoint = loc2point(traci.vehicle.getPosition("65"))
    L7_lastPoint = loc2point(traci.vehicle.getPosition("15"))
    L8_lastPoint = loc2point(traci.vehicle.getPosition("72"))
    L9_lastPoint = loc2point(traci.vehicle.getPosition("90"))
    L10_lastPoint = loc2point(traci.vehicle.getPosition("28"))

    L1_StartPos = L1_lastPoint
    L2_StartPos = L2_lastPoint
    L3_StartPos = L3_lastPoint
    L4_StartPos = L4_lastPoint
    L5_StartPos = L5_lastPoint
    L7_StartPos = L7_lastPoint
    L8_StartPos = L8_lastPoint
    L9_StartPos = L9_lastPoint
    L10_StartPos = L10_lastPoint

    while 1:
        bus39loc = loc2point( traci.vehicle.getPosition("39"))
        bus52loc = loc2point( traci.vehicle.getPosition("52"))
        bus78loc = loc2point( traci.vehicle.getPosition("78"))
        bus0loc = loc2point( traci.vehicle.getPosition("0"))
        bus65loc = loc2point( traci.vehicle.getPosition("65"))
        bus15loc = loc2point(traci.vehicle.getPosition("15"))
        bus72loc = loc2point(traci.vehicle.getPosition("72"))
        bus90loc = loc2point(traci.vehicle.getPosition("90"))
        bus28loc = loc2point(traci.vehicle.getPosition("28"))

        if L1_tracking and bus39loc != L1_lastPoint:
            line_1_Cells.append(bus39loc)
            L1_lastPoint = bus39loc

        if L2_tracking and bus52loc != L2_lastPoint:
            line_2_Cells.append(bus52loc)
            L2_lastPoint = bus52loc

        if L3_tracking and bus78loc != L3_lastPoint:
            line_3_Cells.append(bus78loc)
            L3_lastPoint = bus78loc

        if L4_tracking and bus0loc != L4_lastPoint:
            line_4_Cells.append(bus0loc)
            L4_lastPoint = bus0loc

        if L5_tracking and bus65loc != L5_lastPoint:
            line_5_Cells.append(bus65loc)
            L5_lastPoint = bus65loc

        if L7_tracking and bus15loc != L7_lastPoint:
            line_7_Cells.append(bus15loc)
            L7_lastPoint = bus15loc

        if L8_tracking and bus72loc != L8_lastPoint:
            line_8_Cells.append(bus72loc)
            L8_lastPoint = bus72loc

        if L9_tracking and bus90loc != L9_lastPoint:
            line_9_Cells.append(bus90loc)
            L9_lastPoint = bus90loc

        if L10_tracking and bus28loc != L10_lastPoint:
            line_10_Cells.append(bus28loc)
            L10_lastPoint = bus28loc


        if step > 500 and L1_StartPos == bus39loc:
            L1_tracking = False
        if step > 500 and L2_StartPos == bus52loc:
            L2_tracking = False
        if step > 500 and L3_StartPos == bus78loc:
            L3_tracking = False
        if step > 500 and L4_StartPos == bus0loc:
            L4_tracking = False
        if step > 500 and L5_StartPos == bus65loc:
            L5_tracking = False
        if step > 500 and L7_StartPos == bus15loc:
            L7_tracking = False
        if step > 500 and L8_StartPos == bus72loc:
            L8_tracking = False
        if step > 500 and L9_StartPos == bus90loc:
            L9_tracking = False
        if step > 500 and L10_StartPos == bus28loc:
            L10_tracking = False

        if not ( L4_tracking or L7_tracking or L10_tracking or L1_tracking or L2_tracking or L5_tracking or L3_tracking or L8_tracking or L9_tracking):
            break

        step += 1
        traci.simulationStep()
    
    All_lines = {"line1":line_1_Cells, "line2": line_2_Cells, "line3": line_3_Cells, "line4" : line_4_Cells,"line5":line_5_Cells ,"line7": line_7_Cells, "line8": line_8_Cells, "line9": line_9_Cells, "line10": line_10_Cells}
    all_lines = json.dumps(All_lines)

    
    f = open('linesRoute6.json', 'w')
    f.writelines(all_lines)
    f.close()

    print('going to close...')
    traci.close()
    sys.stdout.flush() 


    '''   ************ check sumo work and vehicles
    while traci.simulation.getMinExpectedNumber() > 0 :
        traci.simulationStep()
        print ("Step ",step,": ",traci.vehicle.getPosition("0"))
        print("Buss in last stop: ", traci.busstop.getVehicleIDs("98_0"))
        #print(step)
        step += 1
    traci.close()
    sys.stdout.flush()'''


if __name__ == "__main__":
    options = get_options()

    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui') # 'sumo-gui'

    traci.start([sumoBinary, "-c", "osm.sumocfg",
                "--tripinfo-output", "tripinfo.xml"])
    run()
    print("done.")