#! /usr/bin/env python3

"""rospy.Rate()を使うことで安定した周期でのループを実現"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

#キーの割当：[angular.z, linear.x]
key_mapping = {'w':[ 0, 1], 'x':[0, -1],
               'a':[-1, 0], 'd':[1,  0],
               's':[ 0, 0]}

last_twist = None   #何も押されなかった場合，直前のtwistを配信するための記録用変数

def keys_callback(msg, twist_pub):
    global last_twist

    if len(msg.data) == 0 or msg.data[0] not in key_mapping.keys():
        return  #データがないもしくはキーマッピングにないデータの場合，何もせずに終了

    velocity = key_mapping[msg.data[0]] #キーマッピングからキーに合わせて抽出
    last_twist = Twist()    #Twistインスタンス生成(0に初期化)
    last_twist.angular.z, last_twist.linear.x = velocity    #配列の要素をそれぞれz, xに代入
    twist_pub.publish(last_twist)   #トピック配信

if __name__ == '__main__':
    rospy.init_node('keys_to_twist')    #ノードの初期化
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)   #cmd_vel配信準備
    rospy.Subscriber('keys', String, keys_callback, twist_pub)  #keysを購読し，コールバック関数を呼び出す．引数はさらに後ろで指定する

    rate = rospy.Rate(10)   #10Hz(100ミリ秒ごと)で出力するため
    last_twist = Twist()    #0に初期化

    while not rospy.is_shutdown():
        twist_pub.publish(last_twist)
        rate.sleep()
