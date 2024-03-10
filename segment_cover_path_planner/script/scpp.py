#!/usr/bin/env python3
import matplotlib.pyplot as plt
from collections import deque
from numpy import arange
from occupancy_grid import OccupancyGridManager
from tf.transformations import quaternion_from_euler
import rospy
from shapely.geometry import Point as SPoint, Polygon as SPolygon
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

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
            # x,y=mat[(i[0]*row_lenght)+i[1]]
            # if(pre_yaw==None):
            #     pre_yaw=i[2]
            #     pre_y=i[1]
            #     quad=quaternion_from_euler(0,0,pre_yaw)
            #     print(x,y,quad[2],quad[3])
            #     waypoint_pose.pose.position.x=mat[(i[0]*row_lenght)+i[1]][0]
            #     waypoint_pose.pose.position.y=mat[(i[0]*row_lenght)+i[1]][1]
            #     waypoint_pose.pose.orientation.z=quad[2]
            #     waypoint_pose.pose.orientation.w=quad[3]
            #     plan.poses.append(waypoint_pose)
            # elif(pre_yaw==i[2]):
            #     if(pre_y==i[1]):
            #         quad=quaternion_from_euler(0,0,i[2])
            #         waypoint_pose.pose.position.x=mat[(i[0]*row_lenght)+i[1]][0]
            #         waypoint_pose.pose.position.y=mat[(i[0]*row_lenght)+i[1]][1]
            #         waypoint_pose.pose.orientation.z=quad[2]
            #         waypoint_pose.pose.orientation.w=quad[3]
            #         plan.poses.append(waypoint_pose)
            #         pre_yaw=i[2]
            #         pre_y=i[1]
            #     continue
            # else:
            #     quad=quaternion_from_euler(0,0,pre_yaw)
            #     print(x,y,quad[2],quad[3])
            #     waypoint_pose.pose.position.x=mat[(i[0]*row_lenght)+i[1]][0]
            #     waypoint_pose.pose.position.y=mat[(i[0]*row_lenght)+i[1]][1]
            #     waypoint_pose.pose.orientation.z=quad[2]
            #     waypoint_pose.pose.orientation.w=quad[3]
            #     plan.poses.append(waypoint_pose)
            #     quad=quaternion_from_euler(0,0,i[2])
            #     waypoint_pose.pose.position.x=mat[(i[0]*row_lenght)+i[1]][0]
            #     waypoint_pose.pose.position.y=mat[(i[0]*row_lenght)+i[1]][1]
            #     waypoint_pose.pose.orientation.z=quad[2]
            #     waypoint_pose.pose.orientation.w=quad[3]
            #     plan.poses.append(waypoint_pose)
            #     pre_yaw=i[2]
            #     pre_y=i[1]
            # waypoint_pose=PoseStamped()
            # waypoint_pose.header.frame_id='map'
            # x,y=mat[(i[0]*row_lenght)+i[1]]
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
        self.polygon_vertices = [(-0.3, 1.6), (1.7, 1.6), (1.7, -2.8), (-0.3, -2.8)]

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

    print(cost_mat)
    print("=====================================================================")
    print(mat)
    print("=====================================================================")
    print(plan_with_angle)
    print("=====================================================================")

    scpp.publish_plan(plan_with_angle,row_len)
    # print(plan_with_angle)
    # connect_waypoints(path)
    print("end_of planning")
    rospy.spin()