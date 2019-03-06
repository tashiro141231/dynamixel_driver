# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Antons Rebguns.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


__author__ = 'Antons Rebguns'
__copyright__ = 'Copyright (c) 2010-2011 Antons Rebguns'
__credits__ = 'Cody Jorgensen, Cara Slutter'

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'


import time
import math
import sys
import errno
from collections import deque
from threading import Thread
from collections import defaultdict

from time import sleep

import roslib
roslib.load_manifest('dynamixel_driver')

from snake_msgs.msg import SnakeJointData, SnakeJointCommand, SnakeHeadUnitCommand

import rospy
import dynamixel_io
from dynamixel_driver.dynamixel_const import *
from dynamixel_driver.head_unit_const import *

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import MotorStateList

from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Header

JointNumber = 29 # ジョイント数 

class SerialProxy():
    def __init__(self,
                 port_name='/dev/ttyUSB0',
                 port_namespace='ttyUSB0',
                 baud_rate='1000000',
                 min_motor_id=10,
                 max_motor_id=38,
                 update_rate=20,
                 diagnostics_rate=1,
                 error_level_temp=75,
                 warn_level_temp=70,
                 readback_echo=False):
        self.port_name = port_name
        self.port_namespace = port_namespace
        self.baud_rate = baud_rate
        # self.min_motor_id = min_motor_id
        # self.max_motor_id = max_motor_id
        self.min_motor_id = 10
        self.max_motor_id = 38
        self.update_rate = update_rate
        self.diagnostics_rate = diagnostics_rate
        self.error_level_temp = error_level_temp
        self.warn_level_temp = warn_level_temp
        self.readback_echo = readback_echo
        self.target_position = [0 for _ in range(JointNumber)]      #目標関節角度
        self.joint_torque_on = [True for _ in range(JointNumber)]   #モータのON/OFF
        self.joint_ready = [False for _ in range(JointNumber)]
        self.js = JointState()                                      #isnake_descriptionに投げてtfを発行させる用
        self.js.name = ['' for _ in range(JointNumber)]             #関節名
        self.js.position = [0 for _ in range(JointNumber)]        #現在の角度
        self.js.header = Header()
        self.js.header.stamp = rospy.Time.now()
        for i in range(JointNumber):
            self.js.name[i] = "current_joint" + str(i+1)                #スクリプトから指定できるように要改正
            print self.js.name[i]
        self.past = rospy.Time.now()

        self.actual_rate = update_rate
        self.error_counts = {'non_fatal': 0, 'checksum': 0, 'dropped': 0}
        self.current_state = MotorStateList()
        self.num_ping_retries = 5
        
        self.motor_state_list_pub = rospy.Publisher('motor_states/%s' % self.port_namespace, MotorStateList, queue_size=1)
        self.motor_state_pub = rospy.Publisher("joint_position", SnakeJointData, queue_size=1)
        self.diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)
        self.scanner_data_pub = rospy.Publisher("scanner_data", PointCloud, queue_size = 10)

    def connect(self):
        try:
            self.dxl_io = dynamixel_io.DynamixelIO(self.port_name, self.baud_rate, self.readback_echo)
            self.__find_motors()
        except dynamixel_io.SerialOpenError, e:
            rospy.logfatal(e.message)
            sys.exit(1)
            
        # for MotorID in range(10,JointNumber+10):
        #     self.dxl_io.write_without_response(MotorID, DXL_RETURN_LEVEL, [0x01])
        #     time.sleep(0.01)
        #     self.running = True
        self.InitMotors()
        rospy.Subscriber('joint_command', SnakeJointCommand, self.RespondToRequest)
        rospy.Subscriber('head_unit_command', SnakeHeadUnitCommand, self.RespondToRequestHU)
        rospy.loginfo("Initialzation has completed");

    def disconnect(self):
        self.running = False

    def InitMotors(self):
        for MotorID in self.motors:
            #リターンレベル、ディバイダー、マルチターンオフセット、EEROMのロックの設定
            self.dxl_io.write_without_response(MotorID, DXL_RETURN_LEVEL, [0x01])
            sleep(0.01)
            self.dxl_io.write_without_response(MotorID, DXL_RESOLUTION_DEVIDER, [0x01])
            sleep(0.01)
            self.dxl_io.write_without_response(MotorID, DXL_MULTI_TURN_OFFSET_L, [0,0])
            sleep(0.01)
            self.dxl_io.write_without_response(MotorID, DXL_RETURN_DELAY_TIME, [0x01])  #返信時間を設定
            sleep(0.01)
            self.dxl_io.write_without_response(MotorID, DXL_LOCK, [0x01])
            sleep(0.01)
            #ジョイントの角度制限(±90度)
            self.dxl_io.set_angle_limit_cw_without_response(MotorID, 1023)
            sleep(0.01)
            self.dxl_io.set_angle_limit_ccw_without_response(MotorID, 3071)
            sleep(0.01)
            #目標速度設定
            # self.dxl_io.set_speed_without_response(MotorID, 70)
            self.dxl_io.set_speed(MotorID, 70)
            sleep(0.01)

            #ヘビ型の角度イニシャライズ 一旦5回ずつ送っておく．
            for loop_times in range(5):
                self.dxl_io.set_position_without_response(MotorID, 2047)

    def ReceiveTargetPosition(self ,joint_data):
        joint_angle = int(2047 - joint_data.value / 360 * 4096)
        self.target_position[joint_data.joint_index] = joint_angle

    def ParseJointAngle(self, data):
        now = rospy.get_rostime()
        i_name = 0
        if data:
            # print str(data)
            i_num = 0
            for index in range(len(data)-6):
                if data[index] == 0xFF and data[index+1] == 0xFF and data[index+4] == 0x00 and data[index + 2] != 0xFF:
                    joint_data = SnakeJointData()
                    joint_data.timestamp = now
                    joint_data.joint_index = data[index+2] - self.min_motor_id
                    joint_data.value = (2048 - (data[index+5] + (data[index+6] << 8))) * 180.0 / 2048.0
                    # joint_state.name.append("current_joint"+str(data[index+2] - self.min_motor_id))
                    # joint_state.position.append((2048 - (data[index+5] + (data[index+6] << 8))) * 3.141592 / 2048 )
                    # joint_state.position.append((2048 - (data[index+5] + (data[index+6] << 8))))
                    # joint_state.position.append(data[index+5] + (data[index+6] << 8))
                    # print joint_state.name[i_num] + " : " + str(joint_state.position[i_num])
                    # print "joint " + str(joint_data.joint_index) + " : " + str(joint_data.value)
                    # i_num = i_num + 1
                    self.motor_state_pub.publish(joint_data)
            # self.motor_state_pub.publish(joint_state)
            # self.motor_state_pub.publish(joint_states)

    def RespondToRequest(self, joint_command):
        if joint_command.ping:
            self.dxl_io.ping(joint_command.joint_index + 10)

        if joint_command.read_position:
            # print "Reading motor state"
            start = time.time()
            # print(get_time/ 1000000)
            if joint_command.target_all:
                # rospy.loginfo("Getting motor states")
                for motor_id in range(self.min_motor_id, self.max_motor_id+1):
                # for motor_id in range(10, 14):
                    # Just request joint position but don't read serial buffer.
                    self.dxl_io.get_position_for_HU(motor_id)
                # time.sleep(0.0007)
                self.ParseJointAngle(self.dxl_io.read_all_buffer())
                end = time.time() - start
                # print("Took :" + str(end) + "[s]")
            if joint_command.target_range:
                # rospy.loginfo("Getting motor states from %d to %d", joint_command.start_joint + 10, joint_command.last_joint + 10)
                for motor_id in range(joint_command.start_joint + self.min_motor_id, joint_command.last_joint + self.min_motor_id + 1):
                    # Just request joint position but don't read serial buffer.
                    self.dxl_io.get_position_for_HU(motor_id)
                # time.sleep(0.0007)
                self.ParseJointAngle(self.dxl_io.read_all_buffer())

        if joint_command.set_position or joint_command.set_position_time:
            # print "Setting target position"
            start = time.time()
            if joint_command.target_range:
                print "Target range using"
                for index in range(joint_command.start_joint, joint_command.last_joint + 1):
                    self.target_position[index - joint_command.start_joint] = int(2047 - joint_command.target_positions[index - joint_command.start_joint] / 360 * 4096)
              
                    if self.joint_torque_on[index]:
                        self.dxl_io.set_position_without_response(index + 10, self.target_position[index - joint_command.start_joint])
                
                end = time.time() - start
                # print("Took :" + str(end) + "[s]")
            else:
                self.target_position[joint_command.joint_index] = int(2047 - joint_command.target_position / 360 * 4097)
                self.dxl_io.set_position_without_response(joint_command.joint_index + 10, self.target_position[joint_command.joint_index])
                end = time.time() - start
                # print("Took :" + str(end) + "[s]")

        if joint_command.set_pid_gain:
            self.dxl_io.set_p_gain_without_response(joint_command.joint_index + 10, joint_command.p_gain)
            self.dxl_io.set_i_gain_without_response(joint_command.joint_index + 10, joint_command.i_gain)
            self.dxl_io.set_d_gain_without_response(joint_command.joint_index + 10, joint_command.d_gain)

        # if joint_command.set_position_velocity:
        #     self.dxl_io.set_position_and_speed_without_response(joint_command.joint_index + 10, joint_command.target_position, joint_commnad.target_velocity)

        if joint_command.change_mode_to_free:
            if joint_command.target_all:
                for i in range(JointNumber):
                    if self.joint_torque_on[i]:
                        rospy.loginfo("Motor %d torque off" , i + 10)
                        self.dxl_io.set_torque_enabled_without_response(i + 10, 0)
                        self.joint_torque_on[i] = False
            elif joint_command.target_range:
                for i in range(joint_command.start_joint, joint_command.last_joint + 1):
                   rospy.loginfo("Motor %d torque off", i + 10)
                   self.dxl_io.set_torque_enabled_without_response(i + 10, 0)
                   self.joint_torque_on[i] = False
            else:
                if self.joint_torque_on[joint_command.joint_index - 1]:
                    rospy.loginfo("Motor %d torque off" ,joint_command.joint_index + 10)
                    self.dxl_io.set_torque_enabled_without_response(joint_command.joint_index + 10, 0)
                    self.joint_torque_on[joint_command.joint_index - 1] = False 

        if joint_command.change_mode_to_active:
            if joint_command.target_all:
                for i in range(JointNumber):
                    if self.joint_torque_on[i] == False:
                        self.joint_torque_on[i] = True
                        self.dxl_io.set_torque_enabled_without_response(i + 10, 1)
                    else:
                        continue

        if joint_command.read_error:
            self.__publish_diagnostic_information()

        if joint_command.read_distance:
            distance = self.dxl_io.get_distance(HU_ID)
            data = PointCloud()
            p = Point32()
            p.x = distance / 100.0
            p.y = 0
            p.z = 0
            data.points.append(p)
            data.header.stamp = rospy.Time.now()
            self.scanner_data_pub.publish(data)

        if joint_command.set_target_velocity:
            self.dxl_io.set_speed_without_response(joint_command.joint_index, joint_command.target_velocity)

        if joint_command.set_position_velocity:
            pos = int(2047 - joint_command.target_position / 180 * 2048)
            vel = int(joint_command.target_velocity / 117.07 * 1023)
            print "pos : " + str(joint_command.target_position) + "  speed: " + str(vel)
            self.dxl_io.set_position_and_speed_without_response(joint_command.joint_index, pos, vel)


    def RespondToRequestHU(self, hu_command):
        if hu_command.read_distance_sensor:
            # self.dxl_io.get_distance(HU_ID)
            print("Not implemented in multi subscriber.")
        if hu_command.read_roll_pitch_yaw:
            print("Not implemented yet")
    
    def __fill_motor_parameters(self, motor_id, model_number):
        """
        Stores some extra information about each motor on the parameter server.
        Some of these paramters are used in joint controller implementation.
        """
        angles = self.dxl_io.get_angle_limits(motor_id)
        voltage = self.dxl_io.get_voltage(motor_id)
        voltages = self.dxl_io.get_voltage_limits(motor_id)
        
        rospy.set_param('dynamixel/%s/%d/model_number' %(self.port_namespace, motor_id), model_number)
        rospy.set_param('dynamixel/%s/%d/model_name' %(self.port_namespace, motor_id), DXL_MODEL_TO_PARAMS[model_number]['name'])
        rospy.set_param('dynamixel/%s/%d/min_angle' %(self.port_namespace, motor_id), angles['min'])
        rospy.set_param('dynamixel/%s/%d/max_angle' %(self.port_namespace, motor_id), angles['max'])
        
        torque_per_volt = DXL_MODEL_TO_PARAMS[model_number]['torque_per_volt']
        rospy.set_param('dynamixel/%s/%d/torque_per_volt' %(self.port_namespace, motor_id), torque_per_volt)
        rospy.set_param('dynamixel/%s/%d/max_torque' %(self.port_namespace, motor_id), torque_per_volt * voltage)
        
        velocity_per_volt = DXL_MODEL_TO_PARAMS[model_number]['velocity_per_volt']
        rpm_per_tick = DXL_MODEL_TO_PARAMS[model_number]['rpm_per_tick']
        rospy.set_param('dynamixel/%s/%d/velocity_per_volt' %(self.port_namespace, motor_id), velocity_per_volt)
        rospy.set_param('dynamixel/%s/%d/max_velocity' %(self.port_namespace, motor_id), velocity_per_volt * voltage)
        rospy.set_param('dynamixel/%s/%d/radians_second_per_encoder_tick' %(self.port_namespace, motor_id), rpm_per_tick * RPM_TO_RADSEC)
        
        encoder_resolution = DXL_MODEL_TO_PARAMS[model_number]['encoder_resolution']
        range_degrees = DXL_MODEL_TO_PARAMS[model_number]['range_degrees']
        range_radians = math.radians(range_degrees)
        rospy.set_param('dynamixel/%s/%d/encoder_resolution' %(self.port_namespace, motor_id), encoder_resolution)
        rospy.set_param('dynamixel/%s/%d/range_degrees' %(self.port_namespace, motor_id), range_degrees)
        rospy.set_param('dynamixel/%s/%d/range_radians' %(self.port_namespace, motor_id), range_radians)
        rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_degree' %(self.port_namespace, motor_id), encoder_resolution / range_degrees)
        rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_radian' %(self.port_namespace, motor_id), encoder_resolution / range_radians)
        rospy.set_param('dynamixel/%s/%d/degrees_per_encoder_tick' %(self.port_namespace, motor_id), range_degrees / encoder_resolution)
        rospy.set_param('dynamixel/%s/%d/radians_per_encoder_tick' %(self.port_namespace, motor_id), range_radians / encoder_resolution)
        
        # keep some parameters around for diagnostics
        self.motor_static_info[motor_id] = {}
        self.motor_static_info[motor_id]['model'] = DXL_MODEL_TO_PARAMS[model_number]['name']
        self.motor_static_info[motor_id]['firmware'] = self.dxl_io.get_firmware_version(motor_id)
        self.motor_static_info[motor_id]['delay'] = self.dxl_io.get_return_delay_time(motor_id)
        self.motor_static_info[motor_id]['min_angle'] = angles['min']
        self.motor_static_info[motor_id]['max_angle'] = angles['max']
        self.motor_static_info[motor_id]['min_voltage'] = voltages['min']
        self.motor_static_info[motor_id]['max_voltage'] = voltages['max']

    def __find_motors(self):
        rospy.loginfo('%s: Pinging motor IDs %d through %d...' % (self.port_namespace, self.min_motor_id, self.max_motor_id))
        self.motors = []
        self.motor_static_info = {}
      
        # for motor_id in range(self.min_motor_id, self.max_motor_id + 1):
        for motor_id in range(10, 41):
            for trial in range(self.num_ping_retries):
                try:
                    result = self.dxl_io.ping(motor_id)
                except Exception as ex:
                    rospy.logerr('Exception thrown while pinging motor %d - %s' % (motor_id, ex))
                    continue

                if result:
                    self.motors.append(motor_id)
                    break

      # while True:
      #   try:
      #     self.dxl_io.ping(0)
      #   except Exception as ex:
      #     rospy.logerr('Exception thrown while pinging motor %d - %s' % (motor_id, ex))
      #     continue
      #             
      # if not self.motors:
      #   rospy.logfatal('%s: No motors found.' % self.port_namespace)
      #   sys.exit(1)
          
        counts = defaultdict(int)
      
        to_delete_if_error = []
        for motor_id in self.motors:
            for trial in range(self.num_ping_retries):
                try:    
                    model_number = self.dxl_io.get_model_number(motor_id)
                    self.__fill_motor_parameters(motor_id, model_number)
                except Exception as ex:
                    rospy.logerr('Exception thrown while getting attributes for motor %d - %s' % (motor_id, ex))
                    if trial == self.num_ping_retries - 1: to_delete_if_error.append(motor_id)
                    continue
                    
                counts[model_number] += 1
                break
                
        for motor_id in to_delete_if_error:
            self.motors.remove(motor_id)
            
        rospy.set_param('dynamixel/%s/connected_ids' % self.port_namespace, self.motors)
        
        status_str = '%s: Found %d motors - ' % (self.port_namespace, len(self.motors))
        for model_number,count in counts.items():
            if count:
                model_name = DXL_MODEL_TO_PARAMS[model_number]['name']
                status_str += '%d %s [' % (count, model_name)
                
            for motor_id in self.motors:
                if self.motor_static_info[motor_id]['model'] == model_name:
                    status_str += '%d, ' % motor_id
                        
            status_str = status_str[:-2] + '], '
                
            rospy.loginfo('%s' % status_str[:-2])

    def __get_joint_states_in_range(self, start_joint, last_joint):
        num_events = 50
        motor_returned = 0
        rates = deque([float(self.update_rate)]*num_events, maxlen=num_events)
        last_time = rospy.Time.now()
        
        motor_states = []
        for motor_id in range(start_joint, last_joint + 1):
            try:
                state = self.dxl_io.get_feedback(motor_id)
                if state:
                    motor_states.append(MotorState(**state))
                    motor_returned += 1
                    if dynamixel_io.exception: raise dynamixel_io.exception
            except dynamixel_io.FatalErrorCodeError, fece:
                rospy.logerr(fece)
            except dynamixel_io.NonfatalErrorCodeError, nfece:
                self.error_counts['non_fatal'] += 1
                rospy.logdebug(nfece)
            except dynamixel_io.ChecksumError, cse:
                self.error_counts['checksum'] += 1
                rospy.logdebug(cse)
            except dynamixel_io.DroppedPacketError, dpe:
                self.error_counts['dropped'] += 1
                rospy.logdebug(dpe.message)
            except OSError, ose:
                if ose.errno != errno.EAGAIN:
                    rospy.logfatal(errno.errorcode[ose.errno])
                        
        if motor_states:
            msl = MotorStateList()
            msl.motor_states = motor_states
            msl.size = motor_returned
            self.motor_state_list_pub.publish(msl)
          
            self.current_state = msl
          
            # calculate actual update rate
            current_time = rospy.Time.now()
            rates.append(1.0 / (current_time - last_time).to_sec())
            self.actual_rate = round(sum(rates)/num_events, 2)
            last_time = current_time
    
    def __update_motor_states(self):
        num_events = 50
        rates = deque([float(self.update_rate)]*num_events, maxlen=num_events)
        last_time = rospy.Time.now()
        
        motor_states = []
        for motor_id in self.motors:
            try:
                state = self.dxl_io.get_feedback(motor_id)
                _stamp = rospy.Time.now()
                if state:
                    # tf用に実際の関節角度をはかせる
                    self.js.header.stamp = _stamp
                    self.js.position[motor_id - 10] = (float)((2047 - MotorState(**state).position) / 4096 * 2 *3.141592)
                    print ("No.%d joint state = %f" % (motor_id,self.js.position[motor_id - 10]))
                    motor_states.append(MotorState(**state))
                    self.motor_state_pub.publish(self.js)
                    if dynamixel_io.exception: raise dynamixel_io.exception
                else:
                    print "did not pub No." + str(motor_id) + " joint state."
            except dynamixel_io.FatalErrorCodeError, fece:
                rospy.logerr(fece)
            except dynamixel_io.NonfatalErrorCodeError, nfece:
                self.error_counts['non_fatal'] += 1
                rospy.logdebug(nfece)
            except dynamixel_io.ChecksumError, cse:
                self.error_counts['checksum'] += 1
                rospy.logdebug(cse)
            except dynamixel_io.DroppedPacketError, dpe:
                self.error_counts['dropped'] += 1
                rospy.logdebug(dpe.message)
            except OSError, ose:
                if ose.errno != errno.EAGAIN:
                    rospy.logfatal(errno.errorcode[ose.errno])
                        
        if motor_states:
            msl = MotorStateList()
            msl.motor_states = motor_states
            self.motor_state_list_pub.publish(msl)
            
            self.current_state = msl
            
            # calculate actual update rate
            current_time = rospy.Time.now()
            rates.append(1.0 / (current_time - last_time).to_sec())
            self.actual_rate = round(sum(rates)/num_events, 2)
            last_time = current_time

    def __publish_diagnostic_information(self):
        diag_msg = DiagnosticArray()
        
        diag_msg.status = []
        diag_msg.header.stamp = rospy.Time.now()
        
        status = DiagnosticStatus()
        
        status.name = 'Dynamixel Serial Bus (%s)' % self.port_namespace
        status.hardware_id = 'Dynamixel Serial Bus on port %s' % self.port_name
        status.values.append(KeyValue('Baud Rate', str(self.baud_rate)))
        status.values.append(KeyValue('Min Motor ID', str(self.min_motor_id)))
        status.values.append(KeyValue('Max Motor ID', str(self.max_motor_id)))
        status.values.append(KeyValue('Desired Update Rate', str(self.update_rate)))
        status.values.append(KeyValue('Actual Update Rate', str(self.actual_rate)))
        status.values.append(KeyValue('# Non Fatal Errors', str(self.error_counts['non_fatal'])))
        status.values.append(KeyValue('# Checksum Errors', str(self.error_counts['checksum'])))
        status.values.append(KeyValue('# Dropped Packet Errors', str(self.error_counts['dropped'])))
        status.level = DiagnosticStatus.OK
        status.message = 'OK'
        
        if self.actual_rate - self.update_rate < -5:
            status.level = DiagnosticStatus.WARN
            status.message = 'Actual update rate is lower than desired'
            
        diag_msg.status.append(status)
        
        for motor_state in self.current_state.motor_states:
            mid = motor_state.id
            
            status = DiagnosticStatus()
            
            status.name = 'Robotis Dynamixel Motor %d on port %s' % (mid, self.port_namespace)
            status.hardware_id = 'DXL-%d@%s' % (motor_state.id, self.port_namespace)
            status.values.append(KeyValue('Model Name', str(self.motor_static_info[mid]['model'])))
            status.values.append(KeyValue('Firmware Version', str(self.motor_static_info[mid]['firmware'])))
            status.values.append(KeyValue('Return Delay Time', str(self.motor_static_info[mid]['delay'])))
            status.values.append(KeyValue('Minimum Voltage', str(self.motor_static_info[mid]['min_voltage'])))
            status.values.append(KeyValue('Maximum Voltage', str(self.motor_static_info[mid]['max_voltage'])))
            status.values.append(KeyValue('Minimum Position (CW)', str(self.motor_static_info[mid]['min_angle'])))
            status.values.append(KeyValue('Maximum Position (CCW)', str(self.motor_static_info[mid]['max_angle'])))
          
            status.values.append(KeyValue('Goal', str(motor_state.goal)))
            status.values.append(KeyValue('Position', str(motor_state.position)))
            status.values.append(KeyValue('Error', str(motor_state.error)))
            status.values.append(KeyValue('Velocity', str(motor_state.speed)))
            status.values.append(KeyValue('Load', str(motor_state.load)))
            status.values.append(KeyValue('Voltage', str(motor_state.voltage)))
            status.values.append(KeyValue('Temperature', str(motor_state.temperature)))
            status.values.append(KeyValue('Moving', str(motor_state.moving)))
          
            if motor_state.temperature >= self.error_level_temp:
                status.level = DiagnosticStatus.ERROR
                status.message = 'OVERHEATING'
            elif motor_state.temperature >= self.warn_level_temp:
                status.level = DiagnosticStatus.WARN
                status.message = 'VERY HOT'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'OK'
                
            diag_msg.status.append(status)
            # print str(diag_msg.status)
              
            # self.diagnostics_pub.publish(diag_msg)

if __name__ == '__main__':
    try:
        serial_proxy = SerialProxy()
        serial_proxy.connect()
        rospy.spin()
        serial_proxy.disconnect()
    except rospy.ROSInterruptException: pass

