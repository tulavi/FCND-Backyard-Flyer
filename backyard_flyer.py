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
        self.all_waypoints = [[15.0, 0.0, 3.0],[15.0, 15.0, 3.0],[0.0, 15.0, 3.0],[0.0, 0.0, 3.0]]
        self.in_mission = True
        self.check_state = {}

        # counter for waypoints
        self.count = 0

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
            print(self.local_position,self.target_position,altitude)

            #check if current position is close to target position
            x_err = np.abs(self.local_position[0] - self.target_position[0])
            y_err = np.abs(self.local_position[1] - self.target_position[1])
            z_err = np.abs(altitude - self.target_position[2])
            print(x_err,y_err,z_err)
            if  x_err <0.2 and y_err < 0.2 and z_err <0.2:
                self.waypoint_transition()
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            land_err = np.sqrt(self.local_position[0]**2 + self.local_position[1]**2 + self.local_position[2]**2)
            print("landing error is" + " ", land_err)
            if ((land_err < 1.0) and abs(self.local_position[2]) < 0.01):
                self.disarming_transition()
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """

    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        pass

    def arming_transition(self):
        print("arming_transition")
        self.take_control()
        self.arm()

        self.set_home_position(self.global_position[0],self.global_position[1],self.global_position[2])
        self.flight_state = States.ARMING
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")

    def takeoff_transition(self):
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """

    def waypoint_transition(self):
        if self.count < 4:
            print(self.count)
            print("waypoint transition")
            self.flight_state = States.WAYPOINT
            self.target_position = self.all_waypoints[self.count]
            self.cmd_position(self.target_position[0],self.target_position[1],self.target_position[2],0)
            self.count += 1
        else:
            self.landing_transition()
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """

    def disarming_transition(self):
        print("disarming transition")
        self.disarm()
        self.flight_state = States.DISARMING

        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """

    def manual_transition(self):
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
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
