# Copyright (c) 2014-2020 Michael E. Ferguson
# Copyright (c) 2010-2011 Vanadium Labs LLC.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
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
#  * Neither the name of the copyright holder nor the names of its
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

from math import pi
import xml.dom.minidom

import rospy


class Joint:
    """Joints hold current values."""

    def __init__(self, node, name):
        """
        Construct a Joint instance.

        node -- The node instance.
        name -- The joint name.
        """
        self.node = node
        self.name = name
        self.controller = None

        self.position = 0.0
        self.velocity = 0.0
        self.last = rospy.Time.now()

    def interpolate(self, frame):
        """
        Get new output, in raw data format.

        frame is the frame length in seconds to interpolate forward.
        Returns the new output, in a raw data format.
        """
        return None

    def setCurrentFeedback(self, raw_data):
        """
        Set the current position from feedback data.

        raw_data -- The current feedback

        Returns The current position, in radians or meters.
        """
        return None

    def setControlOutput(self, position):
        """
        Set the goal position.

        position -- The goal position, in radians/meters.

        Returns the output position, in raw data format.
        """
        return None

    def getDiagnostics(self):
        """Return a diagnostics message for this joint."""
        return None


def getJointsFromURDF():
    """Get joint parameters from URDF."""
    try:
        description = rospy.get_param("robot_description")
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        joints = {}
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed':
                    continue
                name = child.getAttribute('name')
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    limit = child.getElementsByTagName('limit')[0]
                    minval = float(limit.getAttribute('lower'))
                    maxval = float(limit.getAttribute('upper'))

                if minval > 0 or maxval < 0:
                    zeroval = (maxval + minval) / 2
                else:
                    zeroval = 0

                joint = {'min': minval, 'max': maxval, 'zero': zeroval, 'value': zeroval}
                joints[name] = joint
        return joints
    except Exception:
        rospy.loginfo('No URDF defined, proceeding with defaults')
        return dict()
