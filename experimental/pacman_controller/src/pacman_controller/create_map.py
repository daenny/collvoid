#!/usr/bin/env python

import roslib; roslib.load_manifest('pacman_controller')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import String
import commands

import rospy
import math

import pacman_controller.game_engine as GE
THRESHOLD = 0.3

def dist(a, b):
    return math.sqrt(math.pow(a['x'] - b['x'], 2) + math.pow(a['y'] - b['y'], 2))

class createMap():
    def __init__(self):
        topic = 'visualization_markers'
        self.publisher = rospy.Publisher(topic, MarkerArray)
        self.sub_click = rospy.Subscriber("click",PoseStamped, self.click_callback)
        self.sub_unclick = rospy.Subscriber("unclick",PoseStamped, self.unclick_callback)
        self.sub_command = rospy.Subscriber("/commands_robot", String, self.commands_robot_callback)

        self.points = []
        self.edges = []
        self.cur = -1
        self.count = 0

        if rospy.has_param("vertices"):
            gameEngine = GE.GameEngine()
            self.points = gameEngine.points
            self.edges = gameEngine.edges
            self.count = len(self.points)

    def commands_robot_callback(self,msg):
        if msg.data == "Save Map":
            self.saveMap()

    def saveMap(self):
        direct = commands.getoutput('rospack find pacman_controller')
        yamlWrite = open(direct + '/map_created.yaml','w')
        
        yamlWrite.write("vertices:\n")
        for point in self.points:
            yamlWrite.write("  point_%d:\n"%point['id'])
            yamlWrite.write("    id: %d\n"%point['id'])
            yamlWrite.write("    x: %f\n"%point['x'])
            yamlWrite.write("    y: %f\n"%point['y'])

        yamlWrite.write("edges:\n")
        for i in range(len(self.edges)):
            yamlWrite.write("  edge_%d:\n"%i)
            yamlWrite.write("    a: %d\n"%self.edges[i]['a'])
            yamlWrite.write("    b: %d\n"%self.edges[i]['b'])

        yamlWrite.close()

   
    def unclick_callback(self,msg):
        point={}
        point['x'] = msg.pose.position.x
        point['y'] = msg.pose.position.y
        
        index = self.get_point(point)
        if (not index == -1):
            del self.points[index]
            self.delete_edges(index)
            
            for i in range(len(self.points)):
                self.points[i]['id'] = i
            
            self.count -= 1
        print str(self.points)
        print str(self.edges)


    def delete_edges(self,index):
        for edge in self.edges:
            if index in edge:
                self.edges.remove(edge)
                continue
            if edge['a']>index:
                edge['a'] -= 1
            if edge['b']>index:
                edge['b'] -= 1


    def click_callback(self,msg):
        point={}
        point['x'] = msg.pose.position.x
        point['y'] = msg.pose.position.y
        
        index = self.get_point(point)
        if not (index  == -1):
            if self.cur == index:
                self.cur = -1
            else:
                if not (self.cur == -1):
                    self.add_neighbor(self.cur, index)
                self.cur = index
        else:
            point['id'] = self.count
            self.points.append(point)
            if not self.cur == -1:
                self.add_neighbor(self.cur, self.count)
            self.cur = self.count;
            self.count +=1
        print str(self.points)
        print str(self.edges)

    def add_neighbor(self, a, b):
        edgeA = {'a':a, 'b':b}
        edgeB = {'a':b, 'b':a}
        if (edgeA in self.edges or edgeB in self.edges):
            return
        else:
            self.edges.append(edgeA)
      
    def get_point(self, position):
        for point in self.points:
            if (abs(dist(point,position)) < THRESHOLD):
                return point['id']
        return -1;

    def get_dir(self, point_a, point_b):
        result_x = point_b['x'] - point_a['x']
        result_y = point_b['y'] - point_a['y']
        dis = dist(point_a,point_b)
        return [result_x / dis, result_y / dis]

    def publish_markers(self):
        #vertices
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.get_rostime()
        marker.type = marker.SPHERE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.id = 0
        for p in self.points:
            point = Point()
            point.x = p['x']
            point.y = p['y']
            if p['id'] == self.cur:
                continue
            marker.points.append(point)
        markerArray.markers.append(marker)


        #names
        for p in self.points:
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = rospy.get_rostime()
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = p['x']
            marker.pose.position.y = p['y']
            marker.pose.position.z = 0.0
            marker.id = len(markerArray.markers)
            marker.text = 'Node: ' + str(p['id'])
            markerArray.markers.append(marker)

        
        # edges
        marker = Marker() 
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.get_rostime()
        marker.type = marker.SPHERE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.id = len(markerArray.markers)
        for edge in self.edges:
            # print str(edge)
            point_a = self.points[edge['a']]
            point_b = self.points[edge['b']]
 
            cur_point = {}
            cur_point['x'] = point_a['x']
            cur_point['y'] = point_a['y']
            direction = self.get_dir(point_a, point_b)
            while (dist(cur_point, point_b) > 0.2):
                point = Point()
                point.x = cur_point['x'] + 0.2 * direction[0]
                point.y = cur_point['y'] + 0.2 * direction[1]
                cur_point['x'] = point.x
                cur_point['y'] = point.y
                marker.points.append(point)
        
        markerArray.markers.append(marker)

        if not (self.cur == -1):
            # clicked
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = rospy.get_rostime()
#            marker.lifetime = 0.1
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = self.points[self.cur]['x']
            marker.pose.position.y = self.points[self.cur]['y']
            marker.pose.position.z = 0.0
            marker.id = len(markerArray.markers)
            markerArray.markers.append(marker)
        else:
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = rospy.get_rostime()
#            marker.lifetime = 0.1
            marker.type = marker.SPHERE
            marker.action = marker.DELETE
            marker.id = len(markerArray.markers)
            markerArray.markers.append(marker)
     

        self.publisher.publish(markerArray)
        

if __name__ == '__main__':
    rospy.init_node('createMap')
    createMapApp = createMap()
    while not rospy.is_shutdown():
        createMapApp.publish_markers()
        rospy.sleep(0.1)
    #system.exit(0)
