import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

def create_pose(x, y, yaw):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x = float(x)
    goal_pose.pose.position.y = float(y)
    goal_pose.pose.position.z = 0.0
    q = quaternion_from_euler(0, 0, yaw)
    goal_pose.pose.orientation.x = q[0]
    goal_pose.pose.orientation.y = q[1]
    goal_pose.pose.orientation.z = q[2]
    goal_pose.pose.orientation.w = q[3]
    return goal_pose

def main():
    rclpy.init()

    # Define QoS to match your robot's Best Effort subscription
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )

    # Instantiate navigators
    nav1 = BasicNavigator(namespace='robot1')
    nav2 = BasicNavigator(namespace='robot2')

    # Wait for the servers to be fully active
    nav1.waitUntilNav2Active()
    nav2.waitUntilNav2Active()

    # Prepare goals
    goal1 = create_pose(5.0, 3.0, 0.0)
    goal2 = create_pose(-3.0, -6.0, 0.0)

    # Send goals
    nav1.goToPose(goal1)
    nav2.goToPose(goal2)

    # Monitor tasks
    while not nav1.isTaskComplete() or not nav2.isTaskComplete():
        # Small sleep to prevent 100% CPU usage
        rclpy.spin_once(nav1, timeout_sec=0.1)
        rclpy.spin_once(nav2, timeout_sec=0.1)

    print("Tarefas finalizadas!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
