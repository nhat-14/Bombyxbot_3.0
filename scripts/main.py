# @file bombyx_2.0_arduino.ino
#  *
# @brief This code detects gas existance using ARX model. 
#  *
# @author Luong Duc Nhat
# Contact: luong.d.aa@m.titech.ac.jp
# 
# @copyright Copyright 2021, The Chemical Plume Tracing (CPT) Robot Project"
# credits ["Luong Duc Nhat", "Cesar Hernandez-Reyes"]
# license GPL
# 
# @version    = "2.0.0"
# maintainer  = "Luong Duc Nhat"
# status      = "Production"
 
 #!/usr/bin/env python3
import rospy
from collections import namedtuple
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from olfaction_msgs.msg import anemometer

#=============== Bio inspired constanst of moth behavior ======================
Delay       = namedtuple('Delay', ['surge', 'turn1', 'turn2', 'turn3'])
LinearVel   = namedtuple('LinearVel', ['stop', 'surge', 'turn'])
AngularVel  = namedtuple('AngularVel', ['stop', 'surge', 'turnccw', 'turncw'])

delay = Delay(0.5, 1.2, 1.9, 2.5)
lin_v = LinearVel(0.0, 0.30, 0.08)
ang_v = AngularVel(0.0, 0.062, 1.0, -1.0)  #radians per second 


class Bombyxbot():    
    def __init__(self):
        self._tblank = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self._state = 'stop'
        self._robot_dir = 0.0
        self._wind_dir = 0.0
        self._dt = 0.1     
        self._last_hit = 0                              

    def get_agent_dir(self):
        return self._robot_dir
    
    def get_wind_dir(self):
        return self._wind_dir

    def set_agent_dir(self, angle):
        self._robot_dir = angle
    
    def set_wind_dir(self, angle):
        self._wind_dir = angle

    def get_last_hit(self):
        return self._last_hit
    
    def tblank_update(self):
        self._tblank += self._dt

    def get_state(self, ant_state):
        if ant_state != 'nothing':   
            self._state = 'surge'
            self._tblank = 0.0
            if ant_state != 'both':
                self._last_hit = ant_state

        else:
            if self._state == 'surge' and self._tblank >= delay.surge:
                self._state = 'turn1'
            if self._state == 'turn1' and self._tblank >= delay.turn1:
                self._state = 'turn2'
            if self._state == 'turn2' and self._tblank >= delay.turn2:
                self._state = 'turn3'
            if self._state == 'turn3' and self._tblank >= delay.turn3:
                self._state = 'loop'
  
        return self._state


def gasCb(msg):
    linear_vel, angular_vel = (0,0)
    # wind_dir =  agent.get_wind_dir()
    wind_dir = agent.get_agent_dir()
    print(wind_dir, "")
    # if wind_dir > 1.57 or  wind_dir <  -1.57 :
    hit_side = msg.data
    # if wind_dir < 1.6  and  wind_dir > -1.6 :
    #     hit_side = msg.data
    # else:
    #     hit_side = 'nothing'
    # print(hit_side)
    state = agent.get_state(hit_side)
    last_hit = agent.get_last_hit()

    u = {
        'stop': lambda last_hit: (lin_v.stop, ang_v.stop),
        'surge': lambda last_hit: (lin_v.surge, ang_v.surge),
        'turn1': lambda last_hit: (
            lin_v.turn,
            ang_v.turncw if (last_hit == 'left') else ang_v.turnccw
        ),
        'turn2': lambda last_hit: (
            lin_v.turn,
            ang_v.turncw if (last_hit == 'right') else ang_v.turnccw
        ),
        'turn3': lambda last_hit: (
            lin_v.turn,
            ang_v.turncw if (last_hit == 'left') else ang_v.turnccw
        ),
        'loop': lambda last_hit: (
            lin_v.turn,
            ang_v.turncw if (last_hit == 'right') else ang_v.turnccw
        )
    }

    now = rospy.get_rostime()
    if (now - start_time).to_sec() > 5:
        linear_vel, angular_vel = u[state](last_hit)

        move_cmd = Twist()
        move_cmd.linear.x = linear_vel
        move_cmd.angular.z = angular_vel
        pub.publish(move_cmd)
        agent.tblank_update()


def odometryCb(msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    agent.set_agent_dir(yaw)



"""
Callback function for readings from wind sensor. Input data is string 
with the form of "$WI,WVP=000.3,173,0*76\r\n" where 000.3 is wind speed 
and 173 is wind direction in degree.

"""
def windCb(msg):
    wind_spd = msg.wind_speed
    wind_dir = msg.wind_direction
    agent.set_wind_dir(wind_dir)


def plume_tracing():
    global start_time 
    rospy.init_node('gas_sensing', anonymous=True)
    start_time = rospy.get_rostime()
    rospy.Subscriber('arx', String, gasCb)
    rospy.Subscriber('odom',Odometry, odometryCb)
    rospy.Subscriber('chatter',anemometer, windCb)
    rospy.spin()

pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
start_time = 0

#======================= MAIN ============================
if __name__ == '__main__':
    agent = Bombyxbot()
    try:
        plume_tracing()
    except rospy.ROSInterruptException:
        pass