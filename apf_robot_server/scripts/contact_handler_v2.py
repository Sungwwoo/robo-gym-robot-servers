#!/usr/bin/env python3

import rospy
import message_filters
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Bool


class ContactHandler:
    def __init__(self):
        self.pub_collision = rospy.Publisher("base_collision", Bool, queue_size=10)

        self.sub_collision_0 = rospy.Subscriber("base_collision_0", ContactsState, self.cbCollision0)
        self.sub_collision_1 = rospy.Subscriber("base_collision_1", ContactsState, self.cbCollision1)
        self.sub_collision_2 = rospy.Subscriber("base_collision_2", ContactsState, self.cbCollision2)

        self.coll_0 = False
        self.coll_1 = False
        self.coll_2 = False
        return

    def cbCollision0(self, data):
        if not data.states == []:
            self.coll_0 = True
        else:
            self.coll_0 = False

    def cbCollision1(self, data):
        if not data.states == []:
            self.coll_1 = True
        else:
            self.coll_1 = False

    def cbCollision2(self, data):
        if not data.states == []:
            self.coll_2 = True
        else:
            self.coll_2 = False

    def publishCollision(self):
        info = Bool()
        if self.coll_0 or self.coll_1 or self.coll_2:
            info.data = True
            for i in range(0, 10):
                self.pub_collision.publish(info)
        else:
            info.data = False
            self.pub_collision.publish(info)


if __name__ == "__main__":
    rospy.init_node("jackal_contact_hander")
    p = ContactHandler()
    while not rospy.is_shutdown():
        p.publishCollision()
        rospy.sleep(0.05)
        rospy.spin()
