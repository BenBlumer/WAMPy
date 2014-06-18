    WAMPy - Control a Barrett WAM robot using Python
    Copyright (C) 2014 Benjamin Aaron Blumer

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.


NOTE: WAM robots are heavy and can move quickly. And they're expensive. Be careful. This software, ROS, BarrettRos, or the WAM itself may contain bugs that cause the robot to behave in unexpected ways. Never have the robot positioned so that it could hit anyone or anything. I offer NO warranty or guaranty.




This package allows you to easily control a Barrett WAM robot using the Python programming language.  

Technically: It's a ROS (ros.org) node with the necessary services and publishers set up to communicate, through ROS, with BarrettRos (http://support.barrett.com/wiki/WAM/InstallBarrettRosPkg) to control the WAM. 

The motivation for this software is that the BarrettRos package has some great features, but because it's no longer maintained and its documentation is a bit sparse, some of it can be hard to access.  This package is aimed to make it as simple as possible for a new user to get the WAM running.

Note: it includes functions to control the Barrett Hand, but can be used without the hand as well.


Quickstart (Linux):

Installation:
1) Install ROS (ros.org). Note: you may need to install the older version, Fuerte. This is recomended by BarrettRosPkg.
2) Install the BarrettRos package (http://support.barrett.com/wiki/WAM/InstallBarrettRosPkg)
3) Set your ROS_PACKAGE_PATH to whereever this package is located.
 e.g. if it's in /home/userjoe/ros_packages/WAMPy/ then
	a) Open up your bashrc file (located at ~/.bashrc)
	b) add "export ROS_PACKAGE_PATH = $ROS_PACKAGE_PATH:/home/userjoe/ros_packages/" to the end.
	c) Close the terminal and open again.
4) build the package by typing "rosmake WAMPy"

Use:
1) Have ROS running. Open a terminal and run "roscore".
2) You must have the WAM node running. IF you've correctly installed BarrettRos package, in a separate terminal, type "rosrun wam_node wam_node".
3) I've included functions that expose a lot of the BarrettRos functionality. They're described in detail in the docstrings in the code. You can simply write your code under "if __name__ == "__main__" in WAMPy.py, or you can import the code as a module, or you can copy and paste functions into your own code (under, and only under, the terms of the GPLV3 license).

To move to a certain joint position: I recomend using the move_wam_from_current_location() function. This interpolates between the WAM's current joint coordinates and whatever joint coordinates you specify in the amount of time you specify. The first argument is a 7-long list containing the final joint coordinates. The second argument is the duration the trajectory should last. If you try to make it happen to quickly, the WAM may exceed its velocity limits and shut down. The final argument is the frequency at which WAMPy should send the message to wam_node. 500 is the default and what it should remain at.

Note: Some of the other move functions (e.g. create_and_send_wam_trajectory()) allow you to specify a start and end point. However, if the WAM isn't currently at the start position, this can result in eratic movements and WAM failure.

To obtain the current joint pose, use get_wam_joint_coordinates()
Other handy functions include close_wam_hand() and open_wam_hand().

There's some sample code at the bottom of the file. This sample code moves from the current position to the WAM home position, then reaches forward, moves to the side, closes the hand, moves to the other side, opens the hand, moves back to the middle, shifts all the fingers to the same side, then returns to the WAM home position. It's the kind of thing you might use for a pick and place task.

 

BarretRos Package has the following services. I've included acess to the ones marked with an asterisk, but the rest should be easy to implement following the examples I've provided.

Available services:
/bhand/close_grasp *
/bhand/close_spread *
/bhand/finger_pos
/bhand/finger_vel
/bhand/grasp_pos
/bhand/grasp_vel
/bhand/open_grasp *
/bhand/open_spread *
/bhand/spread_pos
/bhand/spread_vel
/rosout/get_loggers
/rosout/set_logger_level
/wam/cart_move
/wam/go_home
/wam/gravity_comp
/wam/hold_cart_pos
/wam/hold_joint_pos
/wam/hold_ortn
/wam/joint_move   *
/wam/ortn_move
/wam/pose_move
/wam_node/get_loggers
/wam_node/set_logger_level
   
Available ROS topics:
/bhand/joint_states
/rosout
/rosout_agg
/wam/cart_pos_cmd
/wam/cart_vel_cmd
/wam/jnt_pos_cmd *
/wam/jnt_vel_cmd
/wam/joint_states  
/wam/ortn_vel_cmd
/wam/pose *
