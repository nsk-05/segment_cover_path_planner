#!/usr/bin/env python3
import matplotlib.pyplot as plt
from collections import deque
from numpy import arange
from occupancy_grid import OccupancyGridManager
from tf.transformations import quaternion_from_euler
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tracking_pid.msg import FollowPathResult,FollowPathActionResult
import actionlib
from shapely.geometry import Point as SPoint, Polygon as SPolygon
from geometry_msgs.msg import PoseStamped,Twist,PointStamped
from nav_msgs.msg import Path
from std_srvs.srv import EmptyResponse,Empty
from std_srvs.srv import Trigger, TriggerResponse

def send_goal(goal_pose):
    # Initialize the move_base client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Create a goal to send to the move_base server
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the goal position
    goal.target_pose.pose.position.x = goal_pose[0]
    goal.target_pose.pose.position.y = goal_pose[1]
    _,_,z,w=quaternion_from_euler(0.0,0.0,goal_pose[2])
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w

    print("sending the intial goal",goal_pose)
    # Send the goal
    client.send_goal(goal)

    # Wait for the result
    res=client.wait_for_result()
    return res

def point_filter(point, polygon_coordinates):
    point = SPoint(point)
    polygon = SPolygon(polygon_coordinates)
    return polygon.contains(point)

def find_path(matrix, start, end):
    rows = len(matrix)
    cols = len(matrix[0])
    visited = set()
    queue = deque([(start, [])])

    while queue:
        current, path = queue.popleft()
        if current == end:
            return path + [end]

        visited.add(current)
        row, col = current

        # Define possible movements: up, down, left, right
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if 0 <= new_row < rows and 0 <= new_col < cols and matrix[new_row][new_col] != 1 and (new_row, new_col) not in visited:
                queue.append(((new_row, new_col), path + [current]))
    return None  # Path not found

def connect_waypoints(waypoints):
    x_coords = [point[0] for point in waypoints]
    y_coords = [point[1] for point in waypoints]

    plt.plot(x_coords, y_coords, marker='o')
    plt.plot(x_coords, y_coords, linestyle='-')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Connect Waypoints')
    plt.grid(True)
    plt.show()

class Segment_coverage_path_planner():
    
    def publish_plan(self,full_plan,row_lenght):
        plan=Path()
        plan.header.frame_id="map"
        plan.header.stamp=rospy.Time.now()
        pre_yaw=None
        pre_y=None
        x,y,z,w=0,0,0,0
        for i in full_plan:
            waypoint_pose=PoseStamped()
            waypoint_pose.header.frame_id='map'
            quad=quaternion_from_euler(0,0,i[2])
            waypoint_pose.pose.position.x=mat[(i[0]*row_lenght)+i[1]][0]
            waypoint_pose.pose.position.y=mat[(i[0]*row_lenght)+i[1]][1]
            waypoint_pose.pose.orientation.z=quad[2]
            waypoint_pose.pose.orientation.w=quad[3]
            plan.poses.append(waypoint_pose)
        self.plan_publisher.publish(plan)

    def __init__(self):
        rospy.init_node("planner_tester")
        self.exploration_point_dist=0.25
        self.ogm = OccupancyGridManager('/move_base_flex/global_costmap/costmap', subscribe_to_updates=False) 
        self.plan_publisher=rospy.Publisher("/SegmentCoverPathPlanner/coverage_plan",Path,queue_size=1)
        self.cmd_pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
        rospy.Subscriber("/follow_path/result",FollowPathActionResult,self.result_callback) 
        rospy.Subscriber("/clicked_point",PointStamped,self.polygon_callback)
        rospy.Service("/clear_segment",Empty,self.close_polygon)
        rospy.Service("/start_planner",Trigger,self.start_planner)
        self.polygon_vertices = [(2.0, 4.0), (3.0, 4.0), (3.0, 4.0), (3.00 ,1.5),(2.0, 4.0)] #[(-0.3, 1.6), (1.7, 1.6), (1.7, -2.8), (-0.3, -2.8)]

    def polygon_callback(self,msg):
        self.polygon_vertices.append([msg.point.x,msg.point.y])

    def close_polygon(self,srv):
        self.polygon_vertices.clear()
        return EmptyResponse()
    
    def start_planner(self,srv):
        mat,cost_mat=self.create_costmap()
        row_len=len(cost_mat[0])
        path=self.find_coverage_plan(mat,cost_mat)
        plan_with_angle=self.find_full_plan(path)
        pose_goal=mat[(plan_with_angle[0][0]*row_len)+plan_with_angle[0][1]]
        pose_goal.append(plan_with_angle[0][2])
        send_goal(pose_goal)
        self.publish_plan(plan_with_angle,row_len)
        print("end_of planning")
        return TriggerResponse(success=True,message="bot started")

    def result_callback(self,msg):
        rospy.sleep(2)
        self.cmd_pub.publish(Twist())
        print("result callback called")

    def create_costmap(self):
        x_values = [vertex[0] for vertex in self.polygon_vertices]
        y_values = [vertex[1] for vertex in self.polygon_vertices]
        min_x,max_x=min(x_values)+0.01,max(x_values)
        min_y,max_y=min(y_values)+0.01,max(y_values)
        matrix=[]
        cost_matrix=[]
        last_row=int((self.polygon_vertices[1][0]-self.polygon_vertices[0][0])/self.exploration_point_dist)
        polygon=SPolygon(self.polygon_vertices)
        for x in arange(min_x,max_x,self.exploration_point_dist):
            #polygon_vertices[0][0],polygon_vertices[1][0],exploration_point_dist):
            _goal_list = list()
            _goal_matrix = list()
            _cost_matrix = list()
            for y in arange(min_y,max_y,0.25):
                try:
                    cost = self.ogm.get_cost_from_world_x_y(x, y)
                    _point=(x,y)
                    if(cost == 0 and point_filter(_point,polygon)):
                        _goal_matrix.append([x,y])
                        _goal_list.append([x,y])
                        _cost_matrix.append(0)
                    else:
                        _cost_matrix.append(1)
                        _goal_matrix.append(None)
                except Exception as ex:
                    pass
            matrix.extend(_goal_matrix)
            cost_matrix.append(_cost_matrix)
        return matrix,cost_matrix
    
    def find_coverage_plan(self,matrix,cost_matrix):
        i=0
        full_path=[]
        column_len=len(cost_matrix[0])
        start_point=None
        while(i<len(matrix)):
            while(True):
                row=i//column_len
                column=i%column_len
                if(row%2!=0):
                    j=(row*column_len)+(column_len-column)
                else:
                    j=i
                if(i>=len(matrix)):
                    break
                if(j>=len(matrix)):
                    i+=1
                    continue
                if(isinstance(matrix[j],list)):
                    break

                i+=1
            row=j//column_len
            column=j%column_len
            if(start_point==None):
                start_point = (row,column)
                i+=1
                continue
            end_point=(row,column)
            path = find_path(cost_matrix, start_point, end_point)
            if(path!=None):
                full_path.extend(path)
                # connect_waypoints(full_path)
            start_point=(row,column)
            i+=1

        return full_path

    def find_full_plan(self,full_path):
        goals_with_angle=[]
        
        for i in range(0,len(full_path)):
            if(i<len(full_path)-1):
                x,y=full_path[i][0]-full_path[i+1][0],full_path[i][1]-full_path[i+1][1]
            if(x==1 and y==0):
                _point=[full_path[i][0],full_path[i][1],1.57+1.57]
            elif(x==-1 and y==0):
                _point=[full_path[i][0],full_path[i][1],-1.57+1.57]
            elif(x==0 and y==1):
                _point=[full_path[i][0],full_path[i][1],0-1.57]
            elif(x==0 and y==-1):
                _point=[full_path[i][0],full_path[i][1],3.14-1.57]
            if(i==0):
                goals_with_angle.append(_point)
                continue
            # goals_with_angle.append(_point)
            else:
                if(goals_with_angle[-1][0]==_point[0] and goals_with_angle[-1][1]==_point[1] and goals_with_angle[-1][2]==_point[2]):
                    continue
                if(goals_with_angle[-1][2]!=_point[2]):
                    goals_with_angle.append([_point[0],_point[1],goals_with_angle[-1][2]])
                    goals_with_angle.append(_point)        
                else:
                    goals_with_angle.append(_point)  
        return goals_with_angle

if __name__=="__main__":
    scpp=Segment_coverage_path_planner()
    mat,cost_mat=scpp.create_costmap()
    row_len=len(cost_mat[0])
    path=scpp.find_coverage_plan(mat,cost_mat)
    plan_with_angle=scpp.find_full_plan(path)
    pose_goal=mat[(plan_with_angle[0][0]*row_len)+plan_with_angle[0][1]]
    pose_goal.append(plan_with_angle[0][2])
    send_goal(pose_goal)


    scpp.publish_plan(plan_with_angle,row_len)
    print("end_of planning")
    rospy.spin()