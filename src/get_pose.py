import rospy, sys
import moveit_commander

class PrintPose(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('print_node', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.xarm7 = moveit_commander.MoveGroupCommander("xarm7")
        self.gripper = moveit_commander.MoveGroupCommander("xarm_gripper")
    
    def printPosition(self):
        pose = self.xarm7.get_current_pose().pose
        print(pose)

        joint = self.xarm7.get_current_joint_values()
        print(joint)


def main():
    pnt = PrintPose()
    rospy.loginfo("Ready to print")
    pnt.printPosition()




if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit(0)