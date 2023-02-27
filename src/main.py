from Connector import CompanionComputerToAutopilot as CC2A
from Connector import CompanionComputerToQGroundControl as CC2QGC
from Connector import SurfaceComputerToAutopilot as SC2AP
from Controller import Commands
import time
import pymavlink.mavutil as mavutil
import ExampleSequences.Testsequence as examples
import ExampleSequences.PymavlinkExamples as pymav_examples
import ExampleSequences.set_target_depthattitude as set_target_depthattitude
def main():
    # create connection from surface computer to autopilot
    examples.Testsequence_SurfaceComputerToAutopilot(useManualControl=False, useSetTargetPosition=True)

if __name__ == '__main__':
    main()





