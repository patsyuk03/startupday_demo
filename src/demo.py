import rospy, sys, moveit_commander, tf
from ar_track_alvar_msgs.msg import AlvarMarkers

cubes_ids = [1, 2]
boxes_ids = [3, 4]
marker_ids = [1, 2, 3, 4]
markers = dict()

def callback(data):
    global markers
    for marker in data.markers:
        if marker.id in marker_ids:
            markers[marker.id] = marker.pose.pose

class PNPDemo(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('demo_node', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.xarm7 = moveit_commander.MoveGroupCommander("xarm7")
        self.gripper = moveit_commander.MoveGroupCommander("xarm_gripper")
    
    def Xarm7ToObject(self, id):
        pose_goal = self.xarm7.get_current_pose().pose

        marker_orientation = (
        markers[cubes_ids[id]].orientation.x,
        markers[cubes_ids[id]].orientation.y,
        markers[cubes_ids[id]].orientation.z,
        markers[cubes_ids[id]].orientation.w)

        euler = tf.transformations.euler_from_quaternion(marker_orientation)
        quaternion = tf.transformations.quaternion_from_euler(3.14 + euler[0], euler[1], euler[2])

        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        pose_goal.position.x = markers[cubes_ids[id]].position.x
        pose_goal.position.y = markers[cubes_ids[id]].position.y

        self.xarm7.set_pose_target(pose_goal)
        plan_success, traj, planning_time, error_code = self.xarm7.plan()
        self.ExecutePlan(traj)

    def Xarm7ToStorage(self, id):
        pose_goal = self.xarm7.get_current_pose().pose
        pose_goal.position.x = markers[boxes_ids[id]].position.x
        pose_goal.position.y = markers[boxes_ids[id]].position.y
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
    
    def lineMotion(self, direction):
        current_pose = self.xarm7.get_current_pose().pose
        waypoints = []
        if direction == "down": 
            current_pose.position.z = 0.03
        elif direction == "down2": 
            current_pose.position.z = 0.05
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
    global markers

    move = PNPDemo()
    rospy.loginfo("Subscribing to ar_pose_marker")
    rospy.Subscriber("/ar_tf_markers", AlvarMarkers, callback)

    while not rospy.is_shutdown():
        inp = input("============ Press `Enter` to start Xarm7 movement")
        if inp == "stop": break
        if len(list(markers.keys())) == 4:
            for id in range(len(cubes_ids)):
                move.Xarm7ToStart()
                move.Gripper("open")
                move.Xarm7ToObject(id)
                move.lineMotion("down")
                move.Gripper("close")
                move.lineMotion("up")
                move.Xarm7ToStorage(id)
                move.lineMotion("down")
                move.Gripper("open")
                move.lineMotion("up")
                move.Xarm7ToStart()
            markers = dict()
        else:
            rospy.loginfo('Not all markers are detected.')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit(0)