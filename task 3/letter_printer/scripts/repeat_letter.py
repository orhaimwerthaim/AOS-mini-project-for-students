#!/usr/bin/env python3
# license removed for brevity
# from https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
import rospy
import actionlib
from geometry_msgs.msg import Point
from letter_printer.srv import * 
from std_msgs.msg import Int8,Bool, String
 
def handle_repeat_letter_print(req):
    print('start print letter service call----------------------------------------------------------------------')
    if len(req.letter) > 0:
        for i in range(req.times):
            print(req.letter[0])
        
        print('end print letter service call----------------------------------------------------------------------')
        return repeat_letter_printResponse(True)
    print('end print letter service call----------------------------------------------------------------------')
    return repeat_letter_printResponse(False)



def single_letter_server():
    rospy.init_node('single_letter_server_node')
    s = rospy.Service('/print_repeat_letter', repeat_letter_print, handle_repeat_letter_print)
    rospy.spin()

if __name__ == '__main__':
    try:
        single_letter_server() 
    except rospy.ROSInterruptException:
        rospy.loginfo("single_letter finished.")
