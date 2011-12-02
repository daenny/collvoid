#!/usr/bin/env python

import roslib; roslib.load_manifest('pacman_controller')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import String
from collvoid_msgs.msg import PoseTwistWithCovariance

import commands

import rospy
import math

THRESHOLD = 0.20

EAT_POINTS = 10
EAT_SPECIAL = 50
EAT_CHERRY = 100
EAT_GHOST = 200

TIME_FLEEING = 20.0

def dist(a, b):
    return math.sqrt(math.pow(a['x'] - b['x'], 2) + math.pow(a['y'] - b['y'], 2))

class State():
    SETUP = -1
    INIT = 0
    RUNNING = 1
    FLEEING = 2
    PAUSED = 3
    GAME_OVER = 4
    STOPPED = 5
    WON = 6

class GameEngine():
    def __init__(self):
        
        self.points = []
        self.edges = []
        self.initialized = False
        self.pacman = {}
        self.ghosts = {}

        self.scoreLoc = rospy.get_param('/score_loc')
        self.infoLoc = rospy.get_param('/info_loc')
        self.power_ups = rospy.get_param('/power_ups')

        self.readMap()

        
        self.state = State.STOPPED

        self.score = 0;
        self.sendStart = False

        topic = 'pacman_map'
        
        self.pubMap = rospy.Publisher(topic, MarkerArray)
        self.subCommandsRobot= rospy.Subscriber("/commands_robot", String, self.commandsRobotCallback)
        self.subCommonPositions = rospy.Subscriber("/position_share", PoseTwistWithCovariance, self.positionShareCallback)

        self.pubPositions = rospy.Publisher('/game_positions', MarkerArray)
        
        self.pubScore = rospy.Publisher('/score', Marker)

        self.pubInfo = rospy.Publisher('/info', Marker)
        self.pubState = rospy.Publisher('/state', String)
        



    def update(self):
        self.publishMap();
        self.publishState()
        self.publishPositions();
        self.publishScore()
    
        if self.state in [State.GAME_OVER, State.STOPPED, State.PAUSED, State.INIT, State.WON, State.SETUP]:
            return;

        self.eatMapPoints();
        self.checkGhosts()
        if (self.gameWon):
            self.state == State.WON

    def gameWon(self):
        for point in self.mapPoints:
            if not point['eaten']:
                return False
        return True
            
    def publishState(self):
        self.pubState.publish(str(self.state));
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.get_rostime()
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 0.2
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.infoLoc['x']
        marker.pose.position.y = self.infoLoc['y']
        marker.pose.position.z = 0.0
        marker.id = 0
        if (self.state in [State.RUNNING]):
            if not self.sendStart:
                marker.text = 'Start'
                self.sendStart = True
                self.timeSend = rospy.get_rostime()
            elif (rospy.get_rostime()-self.timeSend).to_sec() > 0.2: 
                #marker.type = marker.SPHERE
                marker.action = marker.DELETE
            else:
                marker.text = 'Start'

        if (self.state == State.FLEEING):
            timeFleeing = TIME_FLEEING-(rospy.get_rostime()-self.timeStartFleeing).to_sec()
            if timeFleeing > 0:
                marker.text = str(timeFleeing)
            else:
                marker.action = marker.DELETE
                self.state = State.RUNNING
  
        if (self.state in [State.GAME_OVER, State.STOPPED, State.SETUP]):
            marker.text = 'GAME OVER!'

        if (self.state in [State.INIT]):
            marker.text = 'Initializing Game'  

        if (self.state in [State.PAUSED]):
            marker.text = 'Game Paused'

        if (self.state in [State.WON]):
            marker.text = 'Congratulations, you have won the Game! Now get back to you real job!'

        self.pubInfo.publish(marker)

    def publishScore(self):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.scoreLoc['x']
        marker.pose.position.y = self.scoreLoc['y']
        marker.pose.position.z = 0.0
        marker.id = 0
        marker.text = 'Score: ' + str(self.score)
                   
        self.pubScore.publish(marker)

    def reset(self):
        self.score = 0
        self.sendStart = False
        for point in self.mapPoints:
            point['eaten'] = False
        
    def checkGhosts(self):
        for ghost in self.ghosts:
            if (dist(self.pacman, self.ghosts[ghost]) < self.pacman['radius'] + self.ghosts[ghost]['radius'] + 1.5*THRESHOLD):
                if self.state == State.RUNNING and not self.ghosts[ghost]['eaten']:
                    self.state = State.GAME_OVER
                    return
                elif self.state == State.FLEEING:
                    if not self.ghosts[ghost]['eaten']:
                        self.score += EAT_GHOST
                        self.ghosts[ghost]['eaten'] = True

    def eatMapPoints(self):
        for point in self.mapPoints:
            if dist(self.pacman, point) < self.pacman['radius'] + THRESHOLD:
                if (not point['eaten']):
                    self.score += EAT_POINTS
                    point['eaten'] = True
                    if (point['powerup']):
                        self.state = State.FLEEING
                        self.timeStartFleeing = rospy.get_rostime()
            
                
    def publishPositions(self):
        if not self.initialized:
            return
        nr = 0
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.MESH_RESOURCE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.mesh_use_embedded_materials = True
        marker.mesh_resource = "package://pacman_controller/meshes/pacman.dae"
     

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation = self.pacman['orientation']
        marker.pose.position.x = self.pacman['x']
        marker.pose.position.y = self.pacman['y']
        marker.pose.position.z = 0.0
        marker.id = nr
        markerArray.markers.append(marker)
    
        for ghost in self.ghosts:
            curGhost = self.ghosts[ghost]
            if not curGhost["initialized"]:
                continue
            nr += 1
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.MESH_RESOURCE
            marker.action = marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            marker.mesh_use_embedded_materials = True
            if curGhost["eaten"]:
                marker.mesh_resource = "package://pacman_controller/meshes/dead.dae"
            elif self.state == State.FLEEING:
                marker.mesh_resource = "package://pacman_controller/meshes/ghost_catchable.dae"
            else:
                marker.mesh_resource = "package://pacman_controller/meshes/%s.dae"%ghost
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation = curGhost['orientation']
            marker.pose.position.x = curGhost['x']
            marker.pose.position.y = curGhost['y']
            marker.pose.position.z = 0.0
            marker.id = nr
            markerArray.markers.append(marker)


        self.pubPositions.publish(markerArray)
        return

    def positionBlocked(self, position, agent):
        if not agent == self.pacman:
            if (dist(position, self.pacman) < self.pacman['radius'] + THRESHOLD):
                return True
        for ghost in self.ghosts:
            if self.ghosts[ghost] == agent:
                continue
            if (dist(position, self.ghosts[ghost]) < self.ghosts[ghost]['radius'] + THRESHOLD):
                return True
       

        return False

    def readMap(self):
        vertices = rospy.get_param("/vertices");
        edgesRead = rospy.get_param("/edges");
        self.points = []
        self.edges = []
        for i in range(len(vertices)):
            #print str(self.vertices['point_%d'%i])
            point = {}
            point['x'] = vertices['point_%d'%i]['x']
            point['y'] = vertices['point_%d'%i]['y']
            point['id'] = vertices['point_%d'%i]['id']
            point['neighbors'] = []
            self.points.append(point)
        
        for i in range(len(edgesRead)):
            #print str(self.edgesRead['edge_%d'%i])
            self.addEdge(edgesRead['edge_%d'%i])
            self.edges.append(edgesRead['edge_%d'%i])

        self.createMapPoints();

      
    def addEdge(self, edge):
        if not edge['a'] in self.points[edge['b']]['neighbors']:
            self.points[edge['b']]['neighbors'].append(edge['a'])
        if not edge['b'] in self.points[edge['a']]['neighbors']:
            self.points[edge['a']]['neighbors'].append(edge['b'])

    def commandsRobotCallback(self, msg):
        if (msg.data == "Start") and self.state == State.INIT:
            self.state = State.RUNNING

        if (msg.data == "Setup"): 
            self.state = State.SETUP

            
        if (msg.data == "Stop"):
            self.state = State.STOPPED


        if (msg.data == "Init") and self.state in [State.STOPPED, State.GAME_OVER, State.SETUP]:
            self.state = State.INIT
            self.reset()
        
        if self.state in [State.STOPPED, State.GAME_OVER]:
            return

        if (msg.data == "Pause") and self.state in [State.RUNNING, State.FLEEING]:
            self.old_state = self.state
            self.state = State.PAUSED

        if (msg.data == "Resume") and self.state == State.PAUSED:
            self.state = self.old_state
            

    def positionShareCallback(self, msg):
        if (msg.robot_id == "pacman"):
            self.pacman['x'] = msg.pose.pose.position.x
            self.pacman['y'] = msg.pose.pose.position.y
            self.pacman['orientation'] = msg.pose.pose.orientation
            self.pacman['radius'] = msg.radius
            self.initialized = True

        else:
            if not msg.robot_id in self.ghosts:
                self.ghosts[msg.robot_id] = {}
                self.ghosts[msg.robot_id]['initialized'] = False
              
                self.ghosts[msg.robot_id]['eaten'] = False
                self.ghosts[msg.robot_id]['home'] = self.points[rospy.get_param('/%s/home/'%msg.robot_id)]
            self.ghosts[msg.robot_id]['x'] = msg.pose.pose.position.x
            self.ghosts[msg.robot_id]['y'] = msg.pose.pose.position.y
            self.ghosts[msg.robot_id]['orientation'] = msg.pose.pose.orientation
                  
            self.ghosts[msg.robot_id]['radius'] = msg.radius
            if self.ghosts[msg.robot_id]['eaten'] and dist(self.ghosts[msg.robot_id]['home'], self.ghosts[msg.robot_id]) < self.ghosts[msg.robot_id]['radius'] + THRESHOLD:
                self.ghosts[msg.robot_id]['eaten'] = False
            self.ghosts[msg.robot_id]['initialized'] = True

    def createMapPoints(self):
        self.mapPoints = [];
        for edge in self.edges:
#            print str(edge)
            point_a = self.points[edge['a']]
            point_b = self.points[edge['b']]
            cur_point = {}
            cur_point['x'] = point_a['x']
            cur_point['y'] = point_a['y']
            direction = self.get_dir(point_a, point_b)
            while (dist(cur_point, point_b) > 0.2):
                point = {}
                point['x'] = cur_point['x'] + 0.2 * direction[0]
                point['y'] = cur_point['y'] + 0.2 * direction[1]
                point['neighbors'] = [point_a['id'], point_b['id']]
                point['eaten'] = False
                point['powerup'] = False
                cur_point = point
                self.mapPoints.append(point)
        for node in self.power_ups['nodes']:
            point = self.points[node]
            point['eaten'] = False
            point['powerup'] = True
            self.mapPoints.append(point)
            

    def findClosestMapPoint(self,position):
       minDist = dist(self.mapPoints[0], position)
       minPoint = self.mapPoints[0]
       for p in self.mapPoints:
           if p["powerup"]:
               continue;
           distNew = dist(p, position)
           if distNew < minDist:
               minDist = distNew
               minPoint = p
       return minPoint
        

    def findClosestWP(self, position):
        minPoint = self.findClosestMapPoint(position)
        distA = dist(minPoint, self.points[minPoint['neighbors'][0]])
        distB = dist(minPoint, self.points[minPoint['neighbors'][1]])
        if (distA > distB):
            return minPoint['neighbors'][1]
        else: 
            return minPoint['neighbors'][0]
        
    def publishMap(self):
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.id = 0
        for p in self.mapPoints:
            if p['eaten']:
                continue
            if p['powerup']:
                continue
            point = Point()
            point.x = p['x']
            point.y = p['y']
            point.z = 0.15
            marker.points.append(point)
        markerArray.markers.append(marker)

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE_LIST
        marker.action = marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.id = 1
        for p in self.mapPoints:
            if p['eaten']:
                continue
            if not p['powerup']:
                continue
            point = Point()
            point.x = p['x']
            point.y = p['y']
            point.z = 0.2
            marker.points.append(point)
        markerArray.markers.append(marker)
        
        self.pubMap.publish(markerArray)



    def publishMarkers(self):
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "/map"
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
            marker.points.append(point)
        #markerArray.markers.append(marker)

        for p in self.points:
            marker = Marker()
            marker.header.frame_id = "/map"
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
            marker.id = p['id']+1
            marker.text = 'Node: ' + str(p['id'])
            markerArray.markers.append(marker)



        
        nr = len(self.points)+1
        for edge in self.edges:
#            print str(edge)
            point_a = self.points[edge['a']]
            point_b = self.points[edge['b']]
            marker = Marker() 
            marker.header.frame_id = "/map"
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
            marker.id = nr
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
            nr +=1
        self.pubMap.publish(markerArray)

    def get_dir(self, point_a, point_b):
        result_x = point_b['x'] - point_a['x']
        result_y = point_b['y'] - point_a['y']
        dis = dist(point_a,point_b)
        return [result_x / dis, result_y / dis]


    def find_shortest_path(self,start, end, path = [], pathIds = []):
        if pathIds == []:
            pathIds = pathIds + [-1]
        else:
            pathIds = pathIds + [start['id']]
        path = path + [start]
        
        if start == end:
            return path
        shortest = None
        for node in start['neighbors']:
#            print str(node)
            if node not in pathIds:
                newpath = self.find_shortest_path(self.points[node], end, path, pathIds)
                if newpath:
                    if not shortest or len(newpath) < len(shortest):
                        shortest = newpath
        return shortest



if __name__ == '__main__':
    rospy.init_node('game_engine')
    gameEngine = GameEngine()
    while not rospy.is_shutdown():
        gameEngine.update()
        rospy.sleep(0.05)


