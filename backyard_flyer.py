import argparse
import time
from enum import Enum

import visdom
import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

NORTH=0
EAST=1
DOWN=2

LON=0
LAT=1
ALT=2  

class MyDrone(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        # default opens up to http://localhost:8097
        self.v = visdom.Visdom()
        assert self.v.check_connection()

        # Plot NE
        ne = np.array(self.local_position[:2]).reshape(-1, 2)

        self.ne_plot = self.v.scatter(ne, opts=dict(
            title="Local position (north, east)", 
            xlabel='North', 
            ylabel='East'
        ))

        # Plot D
        d = np.array([self.local_position[2]])        
        self.t = 1
        print(d.ndim) 
        self.d_plot = self.v.line(d, X=np.array([self.t]), opts=dict(
            title="Altitude (meters)", 
            xlabel='Timestep', 
            ylabel='Down'
        ))

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
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

        # # for plotting realtime NEO data with visdom
        # self.register_callback(MsgID.LOCAL_POSITION, self.update_ne_plot)
        # self.register_callback(MsgID.LOCAL_POSITION, self.update_d_plot)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:

            # coordinate conversion 
            altitude = -1.0 * self.local_position[2]

            # check if altitude is within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()

        elif self.flight_state == States.WAYPOINT:            
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:                    
                        self.landing_transition()            



    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[ALT] - self.global_home[ALT] < 0.1:
                if abs(self.local_position[DOWN]) < 0.01:
                    self.disarming_transition()


    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed & self.guided : 
                    self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    # # for plotting realtime NEO data with visdom
    # def update_ne_plot(self):
    #     ne = np.array([self.local_position[0], self.local_position[1]]).reshape(-1, 2)
    #     self.v.scatter(ne, win=self.ne_plot, update='append')

    # # for plotting realtime NEO data with visdom
    # def update_d_plot(self):
    #     d = -np.array([self.local_position[2]])
    #     # update timestep
    #     self.t += 1    
    #     self.v.line(d, X=np.array([self.t]), win=self.d_plot, update='append')
        
    def calculate_box(self):
        # Waypoints are dimensions NORTH, EAST, ALT
        altitude = 3.0
        local_waypoints = [[10.0, 0.0, altitude], [10.0, 10.0, altitude], [0.0, 10.0, altitude], [0.0, 0.0, altitude]]        
        return local_waypoints

    def arming_transition(self):
        print("arming transition")

        # if MsgID.STATE is received before MsgID.GLOBAL_POSITION don't want to set_home_position(0,0,0)
        if self.global_position[NORTH] == 0.0 and self.global_position[EAST] == 0.0:
            print("no global position data, wait")
            return

        self.take_control()
        self.arm()

        # set the current location to be the home position
        self.set_home_position(self.global_position[LON],
                               self.global_position[LAT],
                               self.global_position[ALT])        

        self.flight_state = States.ARMING                        


    def takeoff_transition(self):
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[ALT] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        self.target_position = self.all_waypoints.pop(0)
        print('target position', self.target_position)        
        self.cmd_position(self.target_position[NORTH], self.target_position[EAST], self.target_position[ALT], 0.0)
        self.flight_state =States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
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
