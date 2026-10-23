# D1-T が current_servo_angle で返すサーボ角(度)。7軸ぶん。
# **このPCの個体は rt/ を付けずに配信している**ため ROS2 からは購読できない
# (ROS2 は必ず rt/ を付ける)。型定義だけ置いてある。詳細は README を読むこと。
float32 servo0_data
float32 servo1_data
float32 servo2_data
float32 servo3_data
float32 servo4_data
float32 servo5_data
float32 servo6_data
