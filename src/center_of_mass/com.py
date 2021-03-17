#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs as tf_geo
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
import numpy
import math

class CoMCalculator: 
    
    def __init__(self):
        rospy.init_node('com_calculation', anonymous=True)
        self.base_link_frame = rospy.get_param("~base_link_frame", "base_link_frame")
        self.wheel_substr = rospy.get_param("~wheel_substr", "wheel")
        self.tf_prefix = rospy.get_param("~tf_prefix", "")
        self.Mass = 0
        #get robot description from URDF
        robot = URDF.from_parameter_server()
        self.links = robot.link_map
        
        #Delete links, which contain no mass description
        unnecessary_links = []
        for link in self.links:
            if self.links[link].inertial == None:
                unnecessary_links.append(link)

        for link in unnecessary_links:
            del self.links[link]
        
        #Calculate the total mass of the robot
        for link in self.links:
            self.Mass += self.links[link].inertial.mass

        rospy.loginfo("Mass of robot is %f", self.Mass)
        self.calculator()
    
    def calculator(self):
        #initialisations for tf and marker
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        zuTransformieren = geometry_msgs.msg.PointStamped()
        marker = Marker()
        marker.header.frame_id = self.base_link_frame
        marker.header.stamp = rospy.Time()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        pub = rospy.Publisher('com', Marker, queue_size=1)        
        pub_wl = rospy.Publisher('wheel_load', JointState, queue_size=1)
        wl_msg=JointState()
        wl_dict=dict()

        
        rate = rospy.Rate(1)
        rospy.sleep(1)
        
        #loop for calculating the CoM while robot is not shutdown
        while not rospy.is_shutdown():
            x = 0
            y = 0
            z = 0
            for link in self.links:
                iswheel= self.wheel_substr in link
                try:
                    #get transformation matrix of link
                    trans = tfBuffer.lookup_transform(self.base_link_frame, self.tf_prefix + link, rospy.Time())
                    #transform CoM of link
                    zuTransformieren.point.x = self.links[link].inertial.origin.xyz[0]
                    zuTransformieren.point.y = self.links[link].inertial.origin.xyz[1]
                    zuTransformieren.point.z = self.links[link].inertial.origin.xyz[2]
                    zuTransformieren.header.frame_id = self.tf_prefix + link
                    zuTransformieren.header.stamp = rospy.get_rostime()                    
                    transformiert = tf_geo.do_transform_point(zuTransformieren, trans)
                    #calculate part of CoM equation depending on link
                    x += self.links[link].inertial.mass * transformiert.point.x
                    y += self.links[link].inertial.mass * transformiert.point.y
                    z += self.links[link].inertial.mass * transformiert.point.z
                    if iswheel:
                        wl_dict[self.tf_prefix + link]=transformiert
                except tf2_ros.TransformException as err:
                    rospy.logerr("TF error in COM computation %s", err)

                
            #finish CoM calculation
            x = x/self.Mass
            y = y/self.Mass
            z = z/self.Mass

            #send CoM position to RViZ
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            pub.publish(marker)


            X=[]
            for wheel in wl_dict.keys(): 
                X.append([wl_dict[wheel].point.x, wl_dict[wheel].point.y, wl_dict[wheel].point.z]) 
            X=numpy.transpose(numpy.array(X))            
            com=numpy.array([[x],[y],[z]])                        
            mi=numpy.linalg.lstsq(X,com)
            wl_msg.name=wl_dict.keys()
            mi_list=numpy.transpose(mi).tolist()[0]

            
            wl_msg.effort=[self.Mass*m/sum(mi_list) for m in mi_list]
            pub_wl.publish(wl_msg)

            try:
                # catch exeption of moving backwarts in time, when restarting simulator
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                rospy.logwarn("We moved backwards in time. I hope you just resetted the simulation. If not there is something wrong")
            
            
if __name__ == '__main__':
    calc = CoMCalculator()