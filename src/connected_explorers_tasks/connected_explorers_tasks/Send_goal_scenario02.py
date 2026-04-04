import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

def create_pose(navigator, x, y, yaw):
    """Função auxiliar apenas para criar o objeto da pose"""
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = 0.0
    
    q = quaternion_from_euler(0, 0, yaw)
    goal_pose.pose.orientation.x = q[0]
    goal_pose.pose.orientation.y = q[1]
    goal_pose.pose.orientation.z = q[2]
    goal_pose.pose.orientation.w = q[3]
    
    return goal_pose

def main():
    rclpy.init()

    nav1 = BasicNavigator(namespace='robot1')


    nav1.waitUntilNav2Active()

    goal1 = create_pose(nav1, 8.0, -7.0, 0.0)

    nav1.goToPose(goal1)


    while not nav1.isTaskComplete():
        pass

    print("Tarefas finalizadas!")
    
    if nav1.getResult() == TaskResult.SUCCEEDED:
        print("Robot 1 chegou!")
    else:
        print("Robot 1 falhou/cancelado")
        

    rclpy.shutdown()

if __name__ == '__main__':
    main()