import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = self.calculate_box(20.0, 3.0)#[[15.0, 0.0, 3.0],[15.0, 15.0, 3.0],[0.0, 15.0, 3.0],[0.0, 0.0, 3.0]]
        self.in_mission = True
        self.check_state = {}

        # counter for waypoints
        self.count = 0

        #position error threshold
        self.err = 0.1

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF or self.flight_state == States.WAYPOINT:
            
            #coordinate conversion
            altitude = -1.0*self.local_position[2]

            #check if current position is close to target position
            x_err = np.abs(self.local_position[0] - self.target_position[0])
            y_err = np.abs(self.local_position[1] - self.target_position[1])
            z_err = np.abs(altitude - self.target_position[2])
            if  x_err <self.err and y_err <self.err and z_err <self.err:
                self.waypoint_transition()
        if self.flight_state == States.LANDING:
            land_err = np.sqrt(self.local_position[0]**2 + self.local_position[1]**2 + self.local_position[2]**2)
            if ((land_err < 1.0) and abs(self.local_position[2]) < 0.01):
                print("landing error is" + " ", land_err)
                self.disarming_transition()
 

    def velocity_callback(self):
        pass

    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()


    def calculate_box(self,side, altitude):
        #[[15.0, 0.0, 3.0],[15.0, 15.0, 3.0],[0.0, 15.0, 3.0],[0.0, 0.0, 3.0]]
        x = 0.0
        y = 0.0
        return [[x+side, y, altitude], [x+side, y+side, altitude], [x, y+side, altitude], [x,y,altitude]]

    def arming_transition(self):
        print("arming_transition")
        self.take_control()
        self.arm()

        self.set_home_position(self.global_position[0],self.global_position[1],self.global_position[2])
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF
 

    def waypoint_transition(self):
        if self.count < 4:
            print("waypoint transition - ",self.count)
            self.flight_state = States.WAYPOINT
            self.target_position = self.all_waypoints[self.count]
            self.cmd_position(self.target_position[0],self.target_position[1],self.target_position[2],0)
            self.count += 1
        else:
            self.landing_transition()


    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING
  

    def disarming_transition(self):
        print("disarming transition")
        self.disarm()
        self.flight_state = States.DISARMING


    def manual_transition(self):
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL
  

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
 
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
