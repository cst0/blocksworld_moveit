#!/usr/bin/env python3

from geometry_msgs.msg import Pose
import rospy
import sys
import rospy
from moveit_commander import (
    roscpp_initialize,
    RobotCommander,
    PlanningSceneInterface,
    MoveGroupCommander,
)
from moveit_msgs.msg import DisplayTrajectory

from action_runtime.srv import (
    ActionTrigger,
    ActionTriggerRequest,
    ActionTriggerResponse,
)
from blocksworld_moveit.srv import (
    GetBlocksworldScene,
    GetBlocksworldSceneRequest,
)


class BlocksworldNode:
    def __init__(self):
        roscpp_initialize(sys.argv)
        rospy.init_node("blocksworld_node", anonymous=True)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()

        self.gripper_offset = rospy.get_param("~gripper_offset", 0.1)
        self.dropoff_offset = rospy.get_param("~dropoff_offset", 0.1)
        self.travel_height = rospy.get_param("~travel_height", 0.25)

        self.base_link = rospy.get_param("~base_link", self.robot.get_planning_frame())
        self.group_name = rospy.get_param("~group_name", "manipulator")  # type: ignore

        self.corner1_x = rospy.get_param("~corner1_x", +0.0)  # type: ignore
        self.corner1_y = rospy.get_param("~corner1_y", +0.5)  # type: ignore
        self.corner1_z = rospy.get_param("~corner2_x", +0.0)  # type: ignore
        self.corner2_x = rospy.get_param("~corner2_x", +0.5)  # type: ignore
        self.corner2_y = rospy.get_param("~corner2_y", -0.5)  # type: ignore
        self.corner2_z = rospy.get_param("~corner2_z", +0.5)  # type: ignore

        self.table_center = Pose(
            position={
                "x": (self.corner1_x + self.corner2_x) / 2,
                "y": (self.corner1_y + self.corner2_y) / 2,
                "z": min(self.corner1_z, self.corner2_z),
            }
        )

        self.move_group = MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
        )

        self.srv_pickup = rospy.Service("pickup", ActionTrigger, self._handle_pickup)
        self.srv_putdown = rospy.Service("putdown", ActionTrigger, self._handle_putdown)
        self.srv_stack = rospy.Service("stack", ActionTrigger, self._handle_stack)
        self.srv_unstack = rospy.Service("unstack", ActionTrigger, self._handle_unstack)

        self.get_scene = rospy.ServiceProxy("get_scene", GetBlocksworldScene)
        self.close_gripper = rospy.ServiceProxy("close_gripper", ActionTrigger)
        self.open_gripper = rospy.ServiceProxy("open_gripper", ActionTrigger)

    def goto_pose(self, pose):
        self.move_group.set_pose_target(pose)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return True  # TODO: check success

    def goto_pose_cartesian(self, pose):
        waypoints = []

        current_pose = self.move_group.get_current_pose().pose
        current_pose_travel = current_pose
        current_pose_travel.position.z = self.travel_height

        goal_pose = pose
        goal_pose_travel = goal_pose
        goal_pose_travel.position.z = self.travel_height

        waypoints.append(current_pose)
        waypoints.append(current_pose_travel)
        waypoints.append(goal_pose_travel)
        waypoints.append(goal_pose)

        (plan, _) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return True  # TODO: check success

    def _handle_pickup(self, req: ActionTriggerRequest) -> ActionTriggerResponse:
        blocks = self.get_scene(GetBlocksworldSceneRequest()).blocks
        ids = [b.id for b in blocks]
        if req.args[0] not in ids:
            return ActionTriggerResponse(success=False)
        goal_pose: Pose = blocks[ids.index(req.args[0])].pose
        goal_pose.position.z += self.gripper_offset
        ret = self.goto_pose_cartesian(goal_pose)
        ret = ret and self.close_gripper(ActionTriggerRequest())
        return ActionTriggerResponse(success=ret)

    def _handle_putdown(self, req: ActionTriggerRequest) -> ActionTriggerResponse:
        blocks = self.get_scene(GetBlocksworldSceneRequest()).blocks
        ids = [b.id for b in blocks]
        if req.args[0] not in ids:
            return ActionTriggerResponse(success=False)
        goal_pose: Pose = self.table_center
        goal_pose.position.z += self.gripper_offset + self.dropoff_offset
        ret = self.goto_pose_cartesian(goal_pose)
        ret = ret and self.open_gripper(ActionTriggerRequest())
        return ActionTriggerResponse(success=ret)

    def _handle_stack(self, req: ActionTriggerRequest) -> ActionTriggerResponse:
        blocks = self.get_scene(GetBlocksworldSceneRequest()).blocks
        ids = [b.id for b in blocks]
        if req.args[0] not in ids or req.args[1] not in ids:
            return ActionTriggerResponse(success=False)
        goal_pose: Pose = blocks[ids.index(req.args[1])].pose
        goal_pose.position.z += self.gripper_offset
        ret = self.goto_pose_cartesian(goal_pose)
        ret = ret and self.close_gripper(ActionTriggerRequest())
        return ActionTriggerResponse(success=ret)

    def _handle_unstack(self, req: ActionTriggerRequest) -> ActionTriggerResponse:
        blocks = self.get_scene(GetBlocksworldSceneRequest()).blocks
        ids = [b.id for b in blocks]
        if req.args[0] not in ids or req.args[1] not in ids:
            return ActionTriggerResponse(success=False)
        goal_pose: Pose = blocks[ids.index(req.args[0])].pose
        goal_pose.position.z += self.gripper_offset
        ret = self.goto_pose_cartesian(goal_pose)
        ret = ret and self.open_gripper(ActionTriggerRequest())
        return ActionTriggerResponse(success=ret)
