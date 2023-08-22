#!/usr/bin/env python3

import rospy
import message_filters
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Bool


class ContactHandler:
    def __init__(self):
        self.pub_collision = rospy.Publisher("base_collision", Bool, queue_size=10)

        # self.sub_collision_0 = message_filters.Subscriber("base_collision_0", ContactsState)
        self.sub_collision_1 = message_filters.Subscriber("base_collision_1", ContactsState)
        self.sub_collision_2 = message_filters.Subscriber("base_collision_2", ContactsState)

        ts = message_filters.TimeSynchronizer(
            [
                # self.sub_collision_0,
                self.sub_collision_1,
                self.sub_collision_2,
            ],
            10,
        )
        ts.registerCallback(self.cbCollision)

        return

    def cbCollision(self, data2, data3):
        info = Bool()
        if not (data2.states == [] and data3.states == []):
            info.data = True
            for i in range(0, 20):
                self.pub_collision.publish(info)
        else:
            info.data = False

        self.pub_collision.publish(info)


if __name__ == "__main__":
    rospy.init_node("jackal_contact_hander")
    p = ContactHandler()
    while not rospy.is_shutdown():
        rospy.spin()
