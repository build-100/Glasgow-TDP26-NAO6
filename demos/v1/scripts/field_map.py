#!/usr/bin/env python3
"""Fixed Figure 3 pitch: semantic map plus green/white display in map frame."""
import math
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from scenario import centre, edges

def marker(mid,kind,color=(1,1,1,1),ns='fixed_pitch'):
    m=Marker();m.header.frame_id='map';m.ns=ns;m.id=mid;m.type=kind
    m.action=Marker.ADD;m.pose.orientation.w=1
    m.color.r,m.color.g,m.color.b,m.color.a=color
    return m

def make_map():
    grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.05
    grid.info.width,grid.info.height=220,160
    grid.info.origin.position.x=grid.info.origin.position.y=-1
    grid.info.origin.orientation.w=1;grid.data=[0]*(220*160)
    return grid

def pitch_markers():
    ground=marker(0,Marker.CUBE,(0,0.6,0,1))
    ground.pose.position=Point(4.5,3,-.04)
    ground.scale.x,ground.scale.y,ground.scale.z=11,8,.02
    lines=marker(1,Marker.LINE_LIST);lines.scale.x=.035
    def seg(a,b):lines.points.extend([Point(*a,.015),Point(*b,.015)])
    for a,b in [((0,0),(9,0)),((9,0),(9,6)),((9,6),(0,6)),((0,6),(0,0)),((4.5,0),(4.5,6))]:seg(a,b)
    for side in [-1,1]:
        x=0 if side==-1 else 9
        for depth,half in [(1,1.5),(2,2.5)]:
            inside=x-side*depth
            seg((x,3-half),(inside,3-half));seg((inside,3-half),(inside,3+half));seg((inside,3+half),(x,3+half))
        penalty=x-side*1.5
        seg((penalty-.075,3),(penalty+.075,3));seg((penalty,2.925),(penalty,3.075))
    for i in range(96):
        a,b=2*math.pi*i/96,2*math.pi*(i+1)/96
        seg((4.5+.75*math.cos(a),3+.75*math.sin(a)),(4.5+.75*math.cos(b),3+.75*math.sin(b)))
    goals=marker(2,Marker.LINE_LIST);goals.scale.x=.06
    for x,depth in [(0,-.6),(9,.6)]:
        points=[(x,1.7),(x+depth,1.7),(x+depth,4.3),(x,4.3)]
        for a,b in zip(points[:-1],points[1:]):goals.points.extend([Point(*a,.025),Point(*b,.025)])
    return MarkerArray(markers=[ground,lines,goals])

def graph_markers():
    graph=marker(0,Marker.LINE_LIST,(.65,.85,.7,.35),'planning_graph');graph.scale.x=.012
    for a,b in edges():
        for n in [a,b]:
            x,y=centre(n);graph.points.append(Point(x+4.5,y+3,.035))
    cells=marker(1,Marker.LINE_LIST,(.6,.8,.65,.25),'planning_graph');cells.scale.x=.01
    for x in [3,6]:cells.points.extend([Point(x,0,.03),Point(x,6,.03)])
    for y in [2,4]:cells.points.extend([Point(0,y,.03),Point(9,y,.03)])
    markers=[graph,cells]
    for n in range(9):
        x,y=centre(n);m=marker(10+n,Marker.TEXT_VIEW_FACING,(.85,.95,.85,.9),'planning_graph')
        m.pose.position=Point(x+4.5,y+3,.05);m.scale.z=.17;m.text='N'+str(n);markers.append(m)
    return MarkerArray(markers=markers)

def main():
    rospy.init_node('fixed_pitch_map')
    pubs=[rospy.Publisher('/map',OccupancyGrid,queue_size=1,latch=True),
          rospy.Publisher('/field_markings',MarkerArray,queue_size=1,latch=True),
          rospy.Publisher('/planning_graph',MarkerArray,queue_size=1,latch=True)]
    for publisher,message in zip(pubs,[make_map(),pitch_markers(),graph_markers()]):publisher.publish(message)
    rospy.spin()

if __name__=='__main__':main()
