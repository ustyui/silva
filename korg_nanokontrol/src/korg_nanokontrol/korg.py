#!/usr/bin/env python
#
# joystick input driver for Korg NanoKontrol input device
#
# Author: Austin Hendrix
# Author: Allison Thackston

import rospy
import rospkg
import rosparam

import pygame
import pygame.midi

from sensor_msgs.msg import Joy


class KorgNanoKontrol(object):

    ''' Class to connect to and publish the Korg NanoKontrol midi device
        as a ros joystick
    '''

    def __init__(self):
        ''' connect to midi device and set up ros
        '''
        pygame.midi.init()
        devices = pygame.midi.get_count()
        if devices < 1:
            rospy.logerr("No MIDI devices detected")
            exit(-1)
        rospy.loginfo("Found %d MIDI devices" % devices)

        input_dev = int(rospy.get_param("~input_dev",
                                        pygame.midi.get_default_input_id))

        rospy.loginfo("Using input device %d" % input_dev)

        self.controller = pygame.midi.Input(input_dev)

        # load in default parameters if not set
        if not rospy.has_param('~modes'):
            rospack = rospkg.RosPack()
            paramlist = rosparam.load_file(rospack.get_path('korg_nanokontrol') +
                                           '/config/nanokontrolakai_config.yaml')
            for params, ns in paramlist:
                rosparam.upload_params(ns, params)

        self.modes = rospy.get_param('~modes')

        # determine how axis should be interpreted
        self.center_axis = rospy.get_param('~center_axis', True)

        self.msg = Joy()
        self.mode = None

        self.pub = rospy.Publisher('joy', Joy, queue_size=10, latch=True)

    def finish(self):
        del self.controller
        pygame.midi.quit()

    def update(self):
        ''' run the update hook (poll the midi controller, publish the message)
        '''
        while self.controller.poll():
            data = self.controller.read(1)
            if self.mode == None:
                self.set_mode(self.guess_mode(data))
            else:
                self.pub_data(data)

    def set_mode(self, mode):
        ''' Set up the message for the correct mode
        '''
        if not mode == None:
            self.mode = mode
            self.msg.axes = [0]*len(self.modes[mode]['control_axis'])
            self.msg.buttons = [0]*len(self.modes[mode]['control_buttons'])
            self.msg.buttons.append(mode)

    def guess_mode(self, data):
        ''' Given data, try to guess the mode we are in
        '''
        for event in data:
            control = event[0]

            # look for continuous controller commands
            if (control[0] & 0xF0) == 176:
                control_id = control[1] | ((control[0] & 0x0F) << 8)

                # guess initial mode based on command
                candidate = None
                for index, control_mode in enumerate(self.modes):
                    if control_id in control_mode['control_axis']:
                        if candidate is not None:
                            candidate = None
                            break
                        candidate = index

                    if control_id in control_mode['control_buttons']:
                        if candidate is not None:
                            candidate = None
                            break
                        candidate = index
                if not candidate:
                    rospy.loginfo("determining mode..")
                return candidate

    def clip(self, min, max, val):
        if val < min:
            val = min
        if val > max:
            val = max
        return val

    def pub_data(self, data):
        ''' Given data, interpret into Joy message and publish
        '''
        do_publish = False
        self.msg.header.stamp = rospy.Time.now()
	self.msg.header.frame_id ='main'
        for event in data:
            control = event[0]
            # @todo use timestamp from msg
            # timestamp = event[1]

            # look for continuous controller commands
            if (control[0] & 0xF0) == 176:
                control_id = control[1] | ((control[0] & 0x0F) << 8)

                control_axis = self.modes[self.mode]['control_axis']
                control_buttons = self.modes[self.mode]['control_buttons']

                if control_id in control_axis:
                    if self.center_axis:
                        control_val = self.clip(-1.0, 1.0,
                                                float(control[2] - 63) / 63.0)
                    else:
                        control_val = self.clip(0.0, 1.0,
                                                float(control[2]) / 127)

                    axis = control_axis.index(control_id)
                    self.msg.axes[axis] = control_val
                    do_publish = True

                if control_id in control_buttons:
                    button = control_buttons.index(control_id)
                    if control[2] != 0:
                        self.msg.buttons[button] = 1
                    else:
                        self.msg.buttons[button] = 0
                    do_publish = True

            # look for mode commands
            elif control[0] == 79:
                self.set_mode(control[1])
                do_publish = True

        if do_publish:
            self.pub.publish(self.msg)
