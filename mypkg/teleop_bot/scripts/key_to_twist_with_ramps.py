#! /usr/bin/env python3

"""瞬間的に加速するのを防ぐために速度を一定時間で増減させる"""

import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

#キーの割当：[angular.z, linear.x]
key_mapping = {'w':[ 0, 1], 'x':[0, -1],
               'a':[-1, 0], 'd':[1,  0],
               's':[ 0, 0]}

twist_pub      = None       #twist配信用の変数
target_twist   = None       #目標の速度インスタンス用変数
last_twist     = None       #何も押されなかった場合，直前のtwistを配信するための記録用変数
last_send_time = None       #配信した時刻を記録する変数
vel_scales     = [0.1, 0.1] #デフォルトの速度（非常に遅い）
vel_ramps      = [1, 1]     #単位はm/s　増減の度合い 

def ramped_vel(vel_prev, vel_target, time_prev, time_now, ramp_rate):
    """ 最大の速度ステップを計算する """

    step = ramp_rate * (time_now - time_prev).to_sec()

    if vel_target > vel_prev:   #目標値に達していなければsignを1.0に
        sign = 1.0
    else:                       #目標値に達しているならばsignを-1.0に
        sign = -1.0

    error = math.fabs(vel_target - vel_prev)    #絶対値計算　abs()と違って，引数がintでもfloatで返す

    if error < step:    #この時間ステップ内にそこに到達できる --> 到達した
        return vel_target
    else:
        return vel_prev + sign * step   #ターゲットに向けてステップを進める

def ramped_twist(prev, target, time_prev, time_now, ramps):
    """ 計算した速度ステップをtwistに適用 """
    twist = Twist()
    twist.angular.z = ramped_vel(prev.angular.z, target.angular.z, time_prev, time_now, ramps[0])
    twist.linear.x  = ramped_vel(prev.linear.x, target.linear.x, time_prev, time_now, ramps[1])

    return twist

def send_twist():
    """ twistを送る """
    global last_send_time, target_twist, last_twist, vel_scales, vel_ramps, twist_pub

    time_now = rospy.Time.now()

    last_twist = ramped_twist(last_twist, target_twist, last_send_time, time_now, vel_ramps)

    last_send_time = time_now
    twist_pub.publish(last_twist)       #twist配信

def keys_callback(msg):
    global target_twist, last_twist, vel_scales

    if len(msg.data) == 0 or msg.data[0] not in key_mapping.keys():
        return  #データがないもしくはキーマッピングにないデータの場合，何もせずに終了

    velocity = key_mapping[msg.data[0]] #キーマッピングからキーに合わせて抽出
    target_twist = Twist()  #Twistインスタンス生成(0に初期化)
    target_twist.angular.z = velocity[0] * vel_scales[0]    #配列の要素に速度スケール値をかけて目標角速度に代入
    target_twist.linear.x = velocity[1] * vel_scales[1]     #配列の要素に速度スケール値をかけて目標速度に代入

def fetch_param(name, default):
    """ コマンドラインでの入力の受付"""
    if rospy.has_param(name):   #コマンドラインでパラlast_send_timeメータnameが指定されているかチェック
        return rospy.get_param(name)    #指定された値を取得
    else:
        #パラメータが指定されておらず，デフォルト値を使うことを警告する
        rospy.logwarn(f"parameter {name} not defined. Defaulting to {default:.1f}")
        return default


if __name__ == '__main__':
    rospy.init_node('keys_to_twist')    #ノードの初期化
    last_send_time = rospy.Time.now()
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)   #cmd_vel配信準備
    rospy.Subscriber('keys', String, keys_callback) #keysを購読し，コールバック関数を呼び出す．引数はさらに後ろで指定する

    target_twist = Twist()  #0に初期化
    last_twist = Twist()    #0に初期化

    vel_scales[0] = fetch_param('~angular_scale', 0.1)
    vel_scales[1] = fetch_param('~linear_scale', 0.1)
    vel_ramps[0]  = fetch_param('~angular_accel', 1.0)
    vel_ramps[1]  = fetch_param('~angular_accel', 1.0)


    rate = rospy.Rate(20)   #10Hz(100ミリ秒ごと)で出力するため
    while not rospy.is_shutdown():
        send_twist()
        rate.sleep()
