#! /usr/bin/env python3

"""コマンドラインでのパラメータ設定を利用して，速度のスケーリングを実装"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

#キーの割当：[angular.z, linear.x]
key_mapping = {'w':[ 0, 1], 'x':[0, -1],
               'a':[-1, 0], 'd':[1,  0],
               's':[ 0, 0]}

last_twist = None   #何も押されなかった場合，直前のtwistを配信するための記録用変数
vel_scales = [0.1, 0.1] #デフォルトの速度（非常に遅い）

def keys_callback(msg, twist_pub):
    global last_twist, vel_scale

    if len(msg.data) == 0 or msg.data[0] not in key_mapping.keys():
        return  #データがないもしくはキーマッピングにないデータの場合，何もせずに終了

    velocity = key_mapping[msg.data[0]] #キーマッピングからキーに合わせて抽出
    last_twist = Twist()    #Twistインスタンス生成(0に初期化)
    last_twist.angular.z = velocity[0] * vel_scales[0]  #配列の要素に速度スケール値をかけて代入
    last_twist.linear.x = velocity[1] * vel_scales[1]   #配列の要素に速度スケール値をかけて代入
    twist_pub.publish(last_twist)   #トピック配信

if __name__ == '__main__':
    rospy.init_node('keys_to_twist')    #ノードの初期化
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)   #cmd_vel配信準備
    rospy.Subscriber('keys', String, keys_callback, twist_pub)  #keysを購読し，コールバック関数を呼び出す．引数はさらに後ろで指定する

    rate = rospy.Rate(10)   #10Hz(100ミリ秒ごと)で出力するため
    last_twist = Twist()    #0に初期化

    if rospy.has_param('~linear_scale'):    #コマンドラインで'linear_scale'というパラメータが指定されているかチェック
        vel_scales[1] = rospy.get_param('~linear_scale')    #指定された値を取得
    else:
        #パラメータが指定されておらず，デフォルト値を使うことを警告する
        rospy.logwarn(f"linear scale not provided; using {vel_scales[1]:.1f}")

    if rospy.has_param('~angular_scale'):   #コマンドラインで'angular_scale'というパラメータが指定されているかチェック
        vel_scales[0] = rospy.get_param('~angular_scale')   #指定された値を取得
    else:
        #パラメータが指定されておらず，デフォルト値を使うことを警告する
        rospy.logwarn(f"angular scale not provided; using {vel_scales[0]:.1f}")

    while not rospy.is_shutdown():
        twist_pub.publish(last_twist)
        rate.sleep()
