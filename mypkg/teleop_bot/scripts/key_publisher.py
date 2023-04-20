#! /usr/bin/env python3

import sys, select, tty, termios	#ターミナルでの作業に必要なPythonのライブラリ
import rospy
from std_msgs.msg import String		#文字列を扱うメッセージ


if __name__ == '__main__':
	key_pub = rospy.Publisher('keys', String, queue_size = 1)	#トピック配信の準備
	rospy.init_node("keyboard_driver")	#ノードの初期化
	rate = rospy.Rate(100)	#100Hz(0.01s)

	old_attr = termios.tcgetattr(sys.stdin)	#現在のコンソール設定を保存
	tty.setcbreak(sys.stdin.fileno())		#この2行により，ユーザがキーを押したときにすぐに標準入力として受け取れるようになった

	print("Publishing keystrokes. Press Ctrl-C to exit ...")

	while not rospy.is_shutdown():
		"""selectはすぐにリターンする．また0秒でタイムアウトするように設定している"""
		if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
			key_pub.publish(sys.stdin.read(1))	#読み込んだものをStringとして配信
		rate.sleep()

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)	#コンソールを標準モードに戻してからプログラムを抜ける必要がある
