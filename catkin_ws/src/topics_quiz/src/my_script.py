#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist

def callback(msg): 
    print msg.ranges[360] # affiche la distance de l'obstacle devant le robot
    print msg.ranges[719] # affiche la distance de l'obstacle a droite du robot
    print msg.ranges[0]   # affiche la distance de l'obstacle a gauche du robot

    # S'il n'y a pas d'obstacle a moins d'un metre devant le robot, le robot 
    # avance.
    if msg.ranges[360] > 1:
        move.linear.x = 0.3
        move.angular.z = 0.0
      
    # S'il y a un obstacle a moins d'un metre devant le robot, le robot tourne a 
    # gauche.
    if msg.ranges[360] < 1: 
        move.linear.x = 0.0
        move.angular.z = 0.3

    # S'il y a un obstacle a moins d'un metre sur la droite du robot, le robot 
    # tourne a gauche.
    if msg.ranges[719] < 1:     # car 720 hors des limites
        move.linear.x = 0.0
        move.angular.z = -0.3

    # S'il y a un obstacle a moins d'un metre sur la gauche du robot, le robot 
    # tourne a droite.
    if msg.ranges[0] < 1:
        move.linear.x = 0.0
        move.angular.z = 0.3
    
    pub.publish(move) # envoi des mouvements (publish)

# initialisation du noeud topics_quiz_node
rospy.init_node('topics_quiz_node')

# lecture sur le topic du laser
sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, callback) 

# ecriture sur le topic de mouvements du robot
pub = rospy.Publisher('/cmd_vel', Twist)

# lancement du message Twist
move = Twist()

# maintient du node jusqu'au shutdown
rospy.spin()
