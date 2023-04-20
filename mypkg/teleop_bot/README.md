################################################################
★	teleop_botのマニュアル
################################################################
[説明]	キーボード操作用pkg
[内容]	①キー入力を取得
		②キー入力をTwistへ変換

#①key_publisher
	$ rosrun teleop_bot key_publisher.py
	
#②key_to_twist
	#normal
	$ rosrun teleop_bot key_to_twist.py
	#with_ramps&parm
	$ rosrun teleop_bot key_to_twist_with_ramps.py _angular_scale:=0.7 _linear_scale:=0.7

[参考文献]
Qiita - https://qiita.com/Yuya-Shimizu/items/bf58f6fa0925b5460022
