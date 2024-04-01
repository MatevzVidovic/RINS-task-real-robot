#! /usr/bin/env python3
# Mofidied from Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from enum import Enum
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import Spin, NavigateToPose
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler

from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus

import tf_transformations

from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler

import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

qos_profile = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

class RobotCommander(Node):

    def __init__(self, node_name='robot_commander', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)
        
        self.pose_frame_id = 'map'
        
        # Flags and helper variables
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.initial_pose_received = False
        self.is_docked = None

        map_topic = "/map"

        self.map_np = None
        self.map_data = {"map_load_time":None,
                         "resolution":None,
                         "width":None,
                         "height":None,
                         "origin":None} # origin will be in the format [x,y,theta]


        # ROS2 subscribers
        self.create_subscription(DockStatus,
                                 'dock_status',
                                 self._dockCallback,
                                 qos_profile_sensor_data)
        
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self._amclPoseCallback,
                                                              amcl_pose_qos)
        
        # ROS2 publishers
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)
        
        # ROS2 Action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.undock_action_client = ActionClient(self, Undock, 'undock')
        self.dock_action_client = ActionClient(self, Dock, 'dock')

        self.occupancy_grid_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_callback, qos_profile)

        self.get_logger().info(f"Robot commander has been initialized!")
        
    def destroyNode(self):
        self.nav_to_pose_client.destroy()
        super().destroy_node()     


    def map_pixel_to_world(self, x, y, theta=0):
        ### Convert a pixel in an numpy image, to a real world location
        ### Works only for theta=0
        assert not self.map_data["resolution"] is None

        # Apply resolution, change of origin, and translation
        # 
        world_x = x*self.map_data["resolution"] + self.map_data["origin"][0]
        world_y = (self.map_data["height"]-y)*self.map_data["resolution"] + self.map_data["origin"][1]

        # Apply rotation
        return world_x, world_y

    def world_to_map_pixel(self, world_x, world_y, world_theta=0.2):
        ### Convert a real world location to a pixel in a numpy image
        ### Works only for theta=0
        assert self.map_data["resolution"] is not None

        # Apply resolution, change of origin, and translation
        # x is the first coordinate, which in opencv (numpy) that is the matrix row - vertical
        x = int((world_x - self.map_data["origin"][0])/self.map_data["resolution"])
        y = int(self.map_data["height"] - (world_y - self.map_data["origin"][1])/self.map_data["resolution"] )
        
        # Apply rotation
        return x, y



    def map_callback(self, msg):
            self.get_logger().info(f"Read a new Map (Occupancy grid) from the topic.")
            # reshape the message vector back into a map
            self.map_np = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
            # fix the direction of Y (origin at top for OpenCV, origin at bottom for ROS2)
            self.map_np = np.flipud(self.map_np)
            # change the colors so they match with the .pgm image
            self.map_np[self.map_np==0] = 127
            self.map_np[self.map_np==100] = 0
            # load the map parameters
            self.map_data["map_load_time"]=msg.info.map_load_time
            self.map_data["resolution"]=msg.info.resolution
            self.map_data["width"]=msg.info.width
            self.map_data["height"]=msg.info.height
            quat_list = [msg.info.origin.orientation.x,
                        msg.info.origin.orientation.y,
                        msg.info.origin.orientation.z,
                        msg.info.origin.orientation.w]
            self.map_data["origin"]=[msg.info.origin.position.x,
                                    msg.info.origin.position.y,
                                    tf_transformations.euler_from_quaternion(quat_list)[-1]]
            #self.get_logger().info(f"Read a new Map (Occupancy grid) from the topic.")

    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def spin(self, spin_dist=1.57, time_allowance=10):
        self.debug("Waiting for 'Spin' action server")
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist
        goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info(f'Spinning to angle {goal_msg.target_yaw}....')
        send_goal_future = self.spin_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Spin request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def undock(self):
        """Perform Undock action."""
        self.info('Undocking...')
        self.undock_send_goal()

        while not self.isUndockComplete():
            time.sleep(0.1)

    def undock_send_goal(self):
        goal_msg = Undock.Goal()
        self.undock_action_client.wait_for_server()
        goal_future = self.undock_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)

        self.undock_goal_handle = goal_future.result()

        if not self.undock_goal_handle.accepted:
            self.error('Undock goal rejected')
            return

        self.undock_result_future = self.undock_goal_handle.get_result_async()

    def isUndockComplete(self):
        """
        Get status of Undock action.

        :return: ``True`` if undocked, ``False`` otherwise.
        """
        if self.undock_result_future is None or not self.undock_result_future:
            return True

        rclpy.spin_until_future_complete(self, self.undock_result_future, timeout_sec=0.1)

        if self.undock_result_future.result():
            self.undock_status = self.undock_result_future.result().status
            if self.undock_status != GoalStatus.STATUS_SUCCEEDED:
                self.info(f'Goal with failed with status code: {self.status}')
                return True
        else:
            return False

        self.info('Undock succeeded')
        return True

    def cancelTask(self):
        """Cancel pending task request of any type."""
        self.info('Canceling current task.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Task succeeded!')
        return True

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl'):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate(localizer)
        if not self.initial_pose_received:
            time.sleep(1)
        self._waitForNodeToActivate(navigator)
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.debug(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f'Result of get_state: {state}')
            time.sleep(2)
        return
    
    def YawToQuaternion(self, angle_z = 0.):
        quat_tf = quaternion_from_euler(0, 0, angle_z)

        # Convert a list to geometry_msgs.msg.Quaternion
        quat_msg = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])
        return quat_msg

    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        self.current_pose = msg.pose
        return

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return
    
    def _dockCallback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def setInitialPose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = pose
        msg.header.frame_id = self.pose_frame_id
        msg.header.stamp = 0
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return




def get_pose_obj(x, y, fi, rc):

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = rc.get_clock().now().to_msg()

    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.orientation = rc.YawToQuaternion(fi)

    return goal_pose

def add_to_nav_list(to_add_list, nav_list, rc, spin_full_after_go=False):

    for tup in to_add_list:
        if tup[0] == "go":
            to_add = (*tup[1], rc)
            nav_list.append(("go", get_pose_obj(*to_add)))
            if spin_full_after_go:
                nav_list.append(("spin", 3.14))
        elif tup[0] == "spin":
            nav_list.append(("spin", tup[1]))



def main(args=None):
    
    rclpy.init(args=args)
    rc = RobotCommander()

    # Wait until Nav2 and Localizer are available
    rc.waitUntilNav2Active()

    # Check if the robot is docked, only continue when a message is recieved
    while rc.is_docked is None:
        rclpy.spin_once(rc, timeout_sec=0.5)

    # If it is docked, undock it first
    if rc.is_docked:
        rc.undock()
    




    # The mesh in rviz is coordinates.
    # The docking station is 0,0
    # Use Publish Point to hover and see the coordinates.
    # x: -2 to 4
    # y: -2.5 to 5
        

    





    # contains tuples of two types:
    # ("go", <PoseStamped object>), ("spin", angle_to_spin_to)
    navigation_list = []

    

    add_to_navigation = [
        
        # spin spins the robot in place for fi. It doesn't orient it to fi.
        ("spin", 3.14),
        
        # Starting point
        ("go", (0.0, 0.0, 0.57)),

        # Down right
        ("go", (-1.0, 0.25, 0.57)),
        ("go", (-1.6, -0.7, 0.57)),
        ("go", (-0.4, -0.6, 0.57)),
        ("go", (-0.3, -1.85, 0.57)),

        # Right
        ("go", (1.0, -1.9, 0.57)),
        ("go", (2.2, -2.0, 0.57)),

        # Right up
        ("go", (3.4, -1.3, 0.57)),
        ("go", (2.0, -1.0, 0.57)),

        # Centre up
        ("go", (1.5, 0.0, 0.57)),
        ("go", (1.0, 0.0, 0.57)),
        ("go", (2.5, 1.0, 0.57)),

        # Slightly left slightly up
        ("go", (1.5, 2.0, 0.57)),
        ("go", (1.0,1.0, 0.57)),
        ("go", (1.0,2.0, 0.57)),
        ("go", (0.0, 2.0, 0.57)),

        # Slightly left slightly down
        ("go", (-1.0, 1.0, 0.57)),
        ("go", (-1.75, 1.0, 0.57)),
        ("go", (-1.75, 2.0, 0.57)),

        # Left down
        ("go", (-1.5, 4.5, 0.57)),
        ("go", (-1.0, 3.0, 0.57)),

        # Left corridor
        ("go", (0.0, 3.2, 0.57)),
        ("go", (0.5, 2.8, 0.57)),
        ("go", (1.0, 3.5, 0.57)),

        # Left up
        ("go", (1.5, 2.9, 0.57)),
        ("go", (2.0,3.0, 0.57)),

        # Back to slightly left slightly up
        ("go", (1.5, 2.0, 0.57)),
    ]



    add_to_nav_list(add_to_navigation, navigation_list, rc, spin_full_after_go=False)






    # Old way of doing it:
    if False:
        # Path:
        # -1, 0.25
        # -1.6, 0.7
        # -0.4, -0.6
        # -0.3, -1.85
        # 1, -1.9
        # 2.2, -2
        # 3.4, -1.3
        # 2, -1
        # 1.5, 0
        # 1, 0
        # 2.5, 1
        # 1.5, 2
        # 1,1
        # 1,2
        # 0, 2
        # -1, 1
        # -1,75, 1
        # -1,75, 2
        # -1.5, 4.5
        # -1, 3
        # 0, 3.2
        # 0.5, 2.8
        # 1, 3.5
        # 1.5, 2.9
        # 2,3
        # Down right
        navigation_list.append(("go", get_pose_obj(-1.0, 0.25, 0.57, rc)))
        navigation_list.append(("go", get_pose_obj(-1.6, 0.7, 0.57, rc)))
        navigation_list.append(("go", get_pose_obj(-0.4, -0.6, 0.57, rc)))
        navigation_list.append(("go", get_pose_obj(-0.3, -1.85, 0.57, rc)))

        # Right
        navigation_list.append(("go", get_pose_obj(1.0, -1.9, 0.57, rc)))
        navigation_list.append(("go", get_pose_obj(2.2, -2.0, 0.57, rc)))

        # Right up
        navigation_list.append(("go", get_pose_obj(3.4, -1.3, 0.57, rc)))
        navigation_list.append(("go", get_pose_obj(2.0, -1.0, 0.57, rc)))

        # Centre up
        navigation_list.append(("go", get_pose_obj(1.5, 0.0, 0.57, rc)))
        navigation_list.append(("go", get_pose_obj(1.0, 0.0, 0.57, rc)))
        navigation_list.append(("go", get_pose_obj(2.5, 1.0, 0.57, rc)))

        # Slightly left slightly up
        navigation_list.append(("go", get_pose_obj(1.5, 2.0, 0.57, rc)))
        navigation_list.append(("go", get_pose_obj(1.0,1.0, 0.57, rc)))
        navigation_list.append(("go", get_pose_obj(1.0,2.0, 0.57, rc)))
        navigation_list.append(("go", get_pose_obj(0.0, 2.0, 0.57, rc)))

        # Slightly left slightly down
        navigation_list.append(("go", get_pose_obj(-1.0, 1.0, 0.57, rc)))
        navigation_list.append(("go", get_pose_obj(-1.75, 1.0, 0.57, rc)))
        navigation_list.append(("go", get_pose_obj(-1.75, 2.0, 0.57, rc)))

        # Left down
        navigation_list.append(("go", get_pose_obj(-1.5, 4.5, 0.57, rc)))
        navigation_list.append(("go", get_pose_obj(-1.0, 3.0, 0.57, rc)))

        # Left corridor
        navigation_list.append(("go", get_pose_obj(0.0, 3.2, 0.57, rc)))
        navigation_list.append(("go", get_pose_obj(0.5, 2.8, 0.57, rc)))
        navigation_list.append(("go", get_pose_obj(1.0, 3.5, 0.57, rc)))

        # Left up
        navigation_list.append(("go", get_pose_obj(1.5, 2.9, 0.57, rc)))
        navigation_list.append(("go", get_pose_obj(2.0,3.0, 0.57, rc)))

        # Back to slightly left slightly up
        navigation_list.append(("go", get_pose_obj(1.5, 2.0, 0.57, rc)))
    
       
    
    









    add_to_navigation_ix = 0

    while len(navigation_list) > 0:

        # Printing if the goal was added on the initioal list.
        if add_to_navigation_ix < len(add_to_navigation):
            print(add_to_navigation[add_to_navigation_ix])
            add_to_navigation_ix += 1


        curr_type, curr_goal = navigation_list[0]
        
        if curr_type == "go":
            rc.goToPose(curr_goal)
        elif curr_type == "spin":
            rc.spin(curr_goal)
        
        while not rc.isTaskComplete():
            rc.info("Waiting for the task to complete...")
            time.sleep(1)
        
        del navigation_list[0]
    
        # input("Enter sth to continue.")
        
    
    input("Navigation list completed, waiting to terminate. Enter anything.")
        



    

    # Example
    if False:    
        # Finally send it a goal to reach
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = rc.get_clock().now().to_msg()

        goal_pose.pose.position.x = 2.6
        goal_pose.pose.position.y = -1.3
        goal_pose.pose.orientation = rc.YawToQuaternion(0.57)

        rc.goToPose(goal_pose)

        while not rc.isTaskComplete():
            rc.info("Waiting for the task to complete...")
            time.sleep(1)

        rc.spin(-0.57)







    rc.destroyNode()

    # And a simple example
if __name__=="__main__":
    main()