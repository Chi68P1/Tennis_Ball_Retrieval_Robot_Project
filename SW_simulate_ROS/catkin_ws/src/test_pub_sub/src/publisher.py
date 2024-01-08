#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from pynput import keyboard

message1 = Float64()
message2 = Float64()
message3 = Float64()
message4 = Float64()
move_forward = False
move_backward = False
turn_left = False
turn_right = False
spacebar_pressed = False  # Variable to track spacebar state

def on_key_release(key):
    global message1, message2, move_forward, move_backward, turn_left, turn_right

    if key == keyboard.Key.esc:
        # Stop the program when the 'esc' key is pressed
        return False
    elif key == keyboard.KeyCode.from_char('w'):
        move_forward = False
    elif key == keyboard.KeyCode.from_char('s'):
        move_backward = False
    elif key == keyboard.KeyCode.from_char('a'):
        turn_left = False
    elif key == keyboard.KeyCode.from_char('d'):
        turn_right = False
    elif key == keyboard.Key.space:
        spacebar_pressed = False


def on_key_press(key):
    global message1, message2, move_forward, move_backward, turn_left, turn_right

    if key == keyboard.KeyCode.from_char('w'):
        move_forward = True
    elif key == keyboard.KeyCode.from_char('s'):
        move_backward = True
    elif key == keyboard.KeyCode.from_char('a'):
        turn_left = True
    elif key == keyboard.KeyCode.from_char('d'):
        turn_right = True
    elif key == keyboard.Key.space:
        spacebar_pressed = True

def publisher():
    global message1, message2, message3, message4, move_forward, move_backward, turn_left, turn_right

    # Initialize the ROS node
    rospy.init_node("continuous_publisher")

    # Initialize publishers
    joint1 = rospy.Publisher('/my_TRR/joint1_velocity_controller/command', Float64, queue_size=10)
    joint2 = rospy.Publisher('/my_TRR/joint2_velocity_controller/command', Float64, queue_size=10)
    joint3 = rospy.Publisher('/my_TRR/joint3_velocity_controller/command', Float64, queue_size=10)
    joint4 = rospy.Publisher('/my_TRR/joint4_velocity_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(5)  # Publishing rate: 5 Hz

    # Start listening to keyboard input
    with keyboard.Listener(on_release=on_key_release, on_press=on_key_press) as listener:
        while not rospy.is_shutdown():
            message3.data = -110
            message4.data = -110
            if move_forward:
                message1.data = -11.5
                message2.data = -11.5
            elif move_backward:
                message1.data = 11.5
                message2.data = 11.5
            elif turn_left:
                message1.data = -3
                message2.data = 3
            elif turn_right:
                message1.data = 3
                message2.data = -3
            else:
                message1.data = 0.0
                message2.data = 0.0

            # Log and publish the values
            rospy.loginfo("Publishing value for joint1: %s" % message1.data)
            rospy.loginfo("Publishing value for joint2: %s" % message2.data)
            rospy.loginfo("Publishing value for joint3: %s" % message3.data)
            rospy.loginfo("Publishing value for joint4: %s" % message4.data)
            joint1.publish(message1)
            joint2.publish(message2)
            joint3.publish(message3)
            joint4.publish(message4)

            rate.sleep()

if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
