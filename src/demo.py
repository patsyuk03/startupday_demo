import rospy, sys, moveit_commander, tf
from ar_track_alvar_msgs.msg import AlvarMarkers
import geometry_msgs.msg

boxes_ids = [7, 8]
cubes_ids = [[0, 1], [2, 4]]
marker_ids = boxes_ids+cubes_ids[0]+cubes_ids[1]
found_markers = list()
markers = dict({
    boxes_ids[0]: dict({cubes_ids[0][0]:0,cubes_ids[0][1]:0,boxes_ids[0]:0}),
    boxes_ids[1]: dict({cubes_ids[1][0]:0,cubes_ids[1][1]:0,boxes_ids[1]:0})
})

def callback(data):
    global markers, found_markers
    # found_markers = list()
    for marker in data.markers:
        if marker.id in list(markers.keys()):
            if markers[marker.id][marker.id] == 0:
                markers[marker.id][marker.id] = marker.pose.pose
                found_markers.append(marker.id)
        elif marker.id in list(markers[boxes_ids[0]].keys()):
            if markers[boxes_ids[0]][marker.id] == 0:
                markers[boxes_ids[0]][marker.id] = marker.pose.pose
                found_markers.append(marker.id)
        elif marker.id in list(markers[boxes_ids[1]].keys()):
            if markers[boxes_ids[1]][marker.id] == 0:
                markers[boxes_ids[1]][marker.id] = marker.pose.pose
                found_markers.append(marker.id)
    


class PNPDemo(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('demo_node', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.xarm7 = moveit_commander.MoveGroupCommander("xarm7")
        self.gripper = moveit_commander.MoveGroupCommander("xarm_gripper")

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0.01
        p.pose.position.y = 0.00
        p.pose.position.z = -0.07

        self.scene.add_box("table", p, (1, 1, 0.1))

        p.pose.position.x = -0.44
        p.pose.position.y = 0.00
        p.pose.position.z = 0.48
        p.pose.orientation.y = 0.7068252
        p.pose.orientation.w = 0.7073883

        self.scene.add_box("wall", p, (1, 1, 0.1))
    
    def xArm7Move(self, box_id, cube_id):
        pose_goal = self.xarm7.get_current_pose().pose
        marker_pose = markers[box_id][cube_id]

        marker_orientation = (
        marker_pose.orientation.x,
        marker_pose.orientation.y,
        marker_pose.orientation.z,
        marker_pose.orientation.w)

        xarm7_orientation = (
        pose_goal.orientation.x,
        pose_goal.orientation.y,
        pose_goal.orientation.z,
        pose_goal.orientation.w)

        euler_marker = tf.transformations.euler_from_quaternion(marker_orientation)
        euler_xarm7 = tf.transformations.euler_from_quaternion(xarm7_orientation)

        quaternion = tf.transformations.quaternion_from_euler(euler_xarm7[0], euler_xarm7[1], euler_marker[2])

        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

        pose_goal.position.x = marker_pose.position.x
        pose_goal.position.y = marker_pose.position.y

        self.xarm7.set_pose_target(pose_goal)
        plan_success, traj, planning_time, error_code = self.xarm7.plan()
        self.ExecutePlan(traj)

    def Gripper(self, state):
        gripper_values = self.gripper.get_current_joint_values()
        if state == "open":
            gripper_values[0] = 0
        elif state == "close":
            gripper_values[0] = 0.5
        self.gripper.go(gripper_values, wait=True)
    
    def lineMotion(self, direction, cube):
        current_pose = self.xarm7.get_current_pose().pose
        z = [0.03, 0.07]
        waypoints = []
        if direction == "down1": 
            current_pose.position.z = 0.02
        elif direction == "down2":
            current_pose.position.z = z[cube]
        elif direction == "up": 
            current_pose.position.z = 0.3
        waypoints.append(current_pose)
        (traj, fraction) = self.xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.ExecutePlan(traj)

    def ExecutePlan(self, plan):    
        self.xarm7.execute(plan, wait=True)
        self.xarm7.clear_pose_targets()

    def Xarm7ToStart(self):
        joint_goal = self.xarm7.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -0.7
        joint_goal[2] = 0
        joint_goal[3] = 0.8
        joint_goal[4] = 0
        joint_goal[5] = 1.5
        joint_goal[6] = 0
        self.xarm7.go(joint_goal, wait=True)




def main():
    global markers, found_markers
    move = PNPDemo()
    rospy.loginfo("Subscribing to ar_pose_marker")
    rospy.Subscriber("/ar_tf_markers", AlvarMarkers, callback)
    move.Xarm7ToStart()
    move.Gripper("open")
    inp = input("============ Press `Enter` to start Xarm7 movement")
    while not rospy.is_shutdown():
        move.Xarm7ToStart()
        move.Gripper("open")
        if set(found_markers) == set(marker_ids):
            for box_id in list(markers.keys()):
                rospy.loginfo('Collecting cubes to box: %s', str(box_id))
                cube = 0
                cube_ids = list(markers[box_id].keys())
                cube_ids.remove(box_id)
                for cube_id in cube_ids:
                    rospy.loginfo('Going for cube: %s', str(cube_id))
                    move.Xarm7ToStart()
                    move.Gripper("open")
                    move.xArm7Move(box_id, cube_id)
                    move.lineMotion("down1", cube)
                    move.Gripper("close")
                    move.lineMotion("up", cube)
                    move.xArm7Move(box_id, box_id)
                    move.lineMotion("down2", cube)
                    move.Gripper("open")
                    move.lineMotion("up", cube)
                    move.Xarm7ToStart()
                    cube += 1
                    if rospy.is_shutdown():break
                if rospy.is_shutdown():break
            if rospy.is_shutdown():break
            for box_id in list(markers.keys()):
                rospy.loginfo('Collecting cubes to box: %s', str(box_id))
                cube = 1
                cube_ids = list(markers[box_id].keys())
                cube_ids.remove(box_id)
                cube_ids.reverse()
                for cube_id in cube_ids:
                    rospy.loginfo('Going for cube: %s', str(cube_id))
                    move.Xarm7ToStart()
                    move.Gripper("open")
                    move.xArm7Move(box_id, box_id)
                    move.lineMotion("down2", cube)
                    move.Gripper("close")
                    move.lineMotion("up", cube)
                    move.xArm7Move(box_id, cube_id)
                    move.lineMotion("down1", cube)
                    move.Gripper("open")
                    move.lineMotion("up", cube)
                    move.Xarm7ToStart()
                    cube -= 1
                    if rospy.is_shutdown():break
                if rospy.is_shutdown():break
            if rospy.is_shutdown():break
        else:
            rospy.loginfo('Not all markers are detected. Found markers: %s', str(found_markers))
        found_markers = list()
        markers = dict({
            boxes_ids[0]: dict({cubes_ids[0][0]:0,cubes_ids[0][1]:0,boxes_ids[0]:0}),
            boxes_ids[1]: dict({cubes_ids[1][0]:0,cubes_ids[1][1]:0,boxes_ids[1]:0})
        })

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit(0)