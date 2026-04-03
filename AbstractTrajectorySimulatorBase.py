""" Abstract base class for a trajectory simulation

This class provides basic services that will generate and feed broadcasting
thread with appropriate messages.

2 abstract methods need to be overriden in derived classes :
- refresh_delay which should return the simulation timestep in seconds
- update_aircraftinfos which is reponsible for animating the aircraftinfos
  object at each time step, thus making the simulation "alive"

mutex protection occurs when calling replace_message

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""

import time, datetime
import abc
import threading
from CustomDecorators import *
from ModeS import ModeS

DEBUG = False  # Set to True to enable debug output

class AbstractTrajectorySimulatorBase(threading.Thread,abc.ABC):
    def __init__(self,mutex,broadcast_thread,aircraftinfos):
        super().__init__()
        self._mutex = mutex
        self._broadcast_thread = broadcast_thread
        self._aircraftinfos = aircraftinfos

        self._modeSencoder = ModeS(df=17,icao=self._aircraftinfos.icao,ca=self._aircraftinfos.capability)

        self._do_stop = False
    
        self._position_message_period_us = 150000           # max should be 0.2s
        self._velocity_message_period_us = 1200000          # max should be 1.3s
        self._identitification_message_period_us = 10000000 # max should be 15s
        self._squawk_message_period_us = 800000             # TODO : specs says that interval should be randomized between [0.7s;0.9s] and max is 1.0s

        self._startruntime = None
        self._simstep = 0
    
    # Thread core function
    def run(self):

        while not self._do_stop:
            now = datetime.datetime.now(datetime.timezone.utc)
            if (not self._startruntime or (now - self._startruntime).total_seconds() >= self._simstep*self.refresh_delay()):
                self._simstep += 1
                encoder_changed = False

                if DEBUG:
                    print(f"[Trajectory] Simulation step {self._simstep} at {now.strftime('%H:%M:%S.%f')[:-3]}")

                #self._broadcast_thread.getMutex().acquire()
                # check if modeS encoder needs update
                if self._aircraftinfos.icao_changed or self._aircraftinfos.capability_changed or not self._startruntime:
                    self._modeSencoder.icao = self._aircraftinfos.icao
                    self._modeSencoder.ca = self._aircraftinfos.capability
                    encoder_changed = True
                    if DEBUG:
                        print(f"[Trajectory] Encoder updated: ICAO={self._aircraftinfos.icao}, capability={self._aircraftinfos.capability}")

                if encoder_changed or self._aircraftinfos.callsign_changed:
                    if DEBUG:
                        print(f"[Trajectory] Encoding identification: callsign='{self._aircraftinfos.callsign}'")
                    self.df_callsign = self._modeSencoder.callsign_encode(self._aircraftinfos.callsign)
                    if DEBUG:
                        print(f"[Trajectory] Identification frame: {bytes(self.df_callsign).hex()}")
                    self._broadcast_thread.replace_message("identification",self.df_callsign)

                if encoder_changed or self._aircraftinfos.squawk_changed:
                    if DEBUG:
                        print(f"[Trajectory] Encoding squawk: code='{self._aircraftinfos.squawk}'")
                    self.frame_6116 = self._modeSencoder.modaA_encode(self._aircraftinfos.squawk)
                    if DEBUG:
                        print(f"[Trajectory] Squawk frame: {bytes(self.frame_6116).hex()}")
                    self._broadcast_thread.replace_message("register_6116",self.frame_6116)

                # message generation only if needed
                if encoder_changed or self._aircraftinfos.on_surface_changed \
                    or self._aircraftinfos.lat_changed or self._aircraftinfos.lon_changed or self._aircraftinfos.alt_msl_changed \
                    or self._aircraftinfos.type_code_changed or self._aircraftinfos.surveillance_status_changed or self._aircraftinfos.nicsupb_changed \
                    or self._aircraftinfos.timesync_changed:
                    if not self._aircraftinfos.on_surface:
                        if DEBUG:
                            print(f"[Trajectory] Encoding airborne position: lat={self._aircraftinfos.lat_deg}, lon={self._aircraftinfos.lon_deg}, alt={self._aircraftinfos.alt_msl_ft}ft, type_code={self._aircraftinfos.type_code}")
                        (self.df_pos_even, self.df_pos_odd) = self._modeSencoder.df_encode_airborne_position(self._aircraftinfos.lat_deg, self._aircraftinfos.lon_deg, self._aircraftinfos.alt_msl_ft, \
                            self._aircraftinfos.type_code, self._aircraftinfos.surveillance_status, self._aircraftinfos.nicsupb, self._aircraftinfos.timesync)
                        if DEBUG:
                            print(f"[Trajectory] Position even frame: {bytes(self.df_pos_even).hex()}")
                            print(f"[Trajectory] Position odd frame:  {bytes(self.df_pos_odd).hex()}")
                        self._broadcast_thread.replace_message("airborne_position",self.df_pos_even,self.df_pos_odd)
                        self._broadcast_thread.replace_message("surface_position",[], [])                        
                    else:
                        if DEBUG:
                            print(f"[Trajectory] Encoding surface position: lat={self._aircraftinfos.lat_deg}, lon={self._aircraftinfos.lon_deg}, alt={self._aircraftinfos.alt_msl_ft}ft, type_code={self._aircraftinfos.type_code}")
                        (self.df_pos_even, self.df_pos_odd) = self._modeSencoder.df_encode_surface_position(self._aircraftinfos.lat_deg, self._aircraftinfos.lon_deg, self._aircraftinfos.alt_msl_ft, \
                            self._aircraftinfos.type_code, self._aircraftinfos.surveillance_status, self._aircraftinfos.nicsupb, self._aircraftinfos.timesync)
                        if DEBUG:
                            print(f"[Trajectory] Surface position even frame: {bytes(self.df_pos_even).hex()}")
                            print(f"[Trajectory] Surface position odd frame:  {bytes(self.df_pos_odd).hex()}")
                        self._broadcast_thread.replace_message("surface_position",self.df_pos_even,self.df_pos_odd)
                        self._broadcast_thread.replace_message("airborne_position",[], [])

                if encoder_changed or self._aircraftinfos.speed_changed or self._aircraftinfos.track_angle_changed or self._aircraftinfos.vspeed_changed:
                    if DEBUG:
                        print(f"[Trajectory] Encoding velocity: speed={self._aircraftinfos.speed_kt}kt, track={self._aircraftinfos.track_angle_deg}deg, vspeed={self._aircraftinfos.vspeed_ftpmin}fpm")
                    self.df_velocity = self._modeSencoder.df_encode_ground_velocity(self._aircraftinfos.speed_kt, self._aircraftinfos.track_angle_deg, self._aircraftinfos.vspeed_ftpmin) 
                    if DEBUG:
                        print(f"[Trajectory] Velocity frame: {bytes(self.df_velocity).hex()}")
                    self._broadcast_thread.replace_message("airborne_velocity",self.df_velocity)

                #self._broadcast_thread.getMutex().release()
                if not self._startruntime:
                    self._startruntime = now
                    if DEBUG:
                        print(f"[Trajectory] Simulation started at {now.strftime('%H:%M:%S.%f')[:-3]}")
                
                self.update_aircraftinfos()         # update_aircraftinfos() : abstract method that need to be implemented in derived classes
                
                #elapsed = (datetime.datetime.now(datetime.timezone.utc) - now).total_seconds()
                #sleep_time = self.refresh_delay() - elapsed
                #if sleep_time < 0.0:
                #    sleep_time = 0.0
            time.sleep(0.05*self.refresh_delay())    # refresh_delay()        : abstract method that need to be implemented in derived classes

        # upon exit, reset _do_stop flag in case there is a new start
        self._do_stop = False

    @abc.abstractmethod
    def refresh_delay(self):
        ...

    @abc.abstractmethod
    def update_aircraftinfos(self):
        ...

    def stop(self):
        self._do_stop = True

    @property
    def elapsed_sim_time(self):
        return self._simstep*self.refresh_delay()

    @property
    def aircraftinfos(self):
        return self._aircraftinfos

    @property
    def position_message_period_us(self):
        return self._position_message_period_us

    @position_message_period_us.setter
    def position_message_period_us(self,value):
        self._position_message_period_us = value

    @property
    def velocity_message_period_us(self):
        return self._velocity_message_period_us

    @velocity_message_period_us.setter
    def velocity_message_period_us(self,value):
        self._velocity_message_period_us = value

    @property
    def identitification_message_period_us(self):
        return self._identitification_message_period_us

    @identitification_message_period_us.setter
    def identitification_message_period_us(self,value):
        self._identitification_message_period_us = value

    @property
    def squawk_message_period_us(self):
        return self._squawk_message_period_us

    @squawk_message_period_us.setter
    def squawk_message_period_us(self,value):
        self._squawk_message_period_us = value
