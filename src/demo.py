import rospy, sys, moveit_commander, tf
from ar_track_alvar_msgs.msg import AlvarMarkers

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
    found_markers = list()
    for marker in data.markers:
        if marker.id in list(markers.keys()):
            markers[marker.id][marker.id] = marker.pose.pose
            found_markers.append(marker.id)
        elif marker.id in list(markers[boxes_ids[0]].keys()):
            markers[boxes_ids[0]][marker.id] = marker.pose.pose
            found_markers.append(marker.id)
        elif marker.id in list(markers[boxes_ids[1]].keys()):
            markers[boxes_ids[1]][marker.id] = marker.pose.pose
            found_markers.append(marker.id)
    


class PNPDemo(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('demo_node', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.xarm7 = moveit_commander.MoveGroupCommander("xarm7")
        self.gripper = moveit_commander.MoveGroupCommander("xarm_gripper")
    
    def movexArm7(self, box_id, cube_id):
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
        z = [0.04, 0.07]
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
    move = PNPDemo()
    rospy.loginfo("Subscribing to ar_pose_marker")
    rospy.Subscriber("/ar_tf_markers", AlvarMarkers, callback)

    while not rospy.is_shutdown():
        inp = input("============ Press `Enter` to start Xarm7 movement")
        if inp == "stop": 
            break
        if inp == "start":
            move.Xarm7ToStart()
            move.Gripper("open")
            break

        move.Xarm7ToStart()
        move.Gripper("open")
        if len(found_markers) == len(marker_ids):
            for box_id in list(markers.keys()):
                rospy.loginfo('Collecting cubes to box: %s', str(box_id))
                cube = 0
                cubes_ids = list(markers[box_id].keys())
                cubes_ids.remove(box_id)
                for cube_id in cubes_ids:
                    rospy.loginfo('Going for cube: %s', str(cube_id))
                    move.Xarm7ToStart()
                    move.Gripper("open")
                    move.movexArm7(box_id, cube_id)
                    move.lineMotion("down1", cube)
                    move.Gripper("close")
                    move.lineMotion("up", cube)
                    move.movexArm7(box_id, box_id)
                    move.lineMotion("down2", cube)
                    move.Gripper("open")
                    move.lineMotion("up", cube)
                    move.Xarm7ToStart()
                    cube += 1
                    if rospy.is_shutdown():break
                if rospy.is_shutdown():break
        else:
            rospy.loginfo('Not all markers are detected. Found markers: %s', str(found_markers))

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit(0)