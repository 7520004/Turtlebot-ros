#! /usr/bin/env python3

"""
タスク

・1秒間に10回の頻度で動作コマンドを送り続ける
・3秒ごとに移動と停止を切り替える
・移動：0.5 m/sの前進
・停止：0 m/s
"""

import rospy
from geometry_msgs.msg import Twist #速度コマンドをもつ

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1) #Twist型で'cmd_vel'というトピックを生成
#queueは一時的に保存できるメッセージ数．これを超える頻度でメッセージを送ると，rospyでは超える分を破棄する

rospy.init_node('red_light_green_light') #'red_light_green_light'という名前のノードを作成

red_light_twist = Twist()   #Twistのインスタンスを作成
green_light_twist = Twist() #こうすることで，Twistのフィールドにアクセス可能

green_light_twist.linear.x = 0.5    #green_lightのTwistでx方向に0.5 m/s


driving_forward = False
light_change_time = rospy.Time.now()
rate = rospy.Rate(10)   #10Hz → 1秒間に10回のコマンドを送る

while not rospy.is_shutdown():
    if driving_forward:
        cmd_vel_pub.publish(green_light_twist)  #green_light_twistでトピック配信
    else:
        cmd_vel_pub.publish(red_light_twist)    #red_light_twistでトピック配信


    if light_change_time < rospy.Time.now():
        driving_forward = not driving_forward   #driving_forwardのT/F切り替え
        light_change_time = rospy.Time.now() + rospy.Duration(3)

    rate.sleep()    #必要以上に膨大なメッセージを送ってしまいCPUを使い果たしてしまうことを防ぐ
