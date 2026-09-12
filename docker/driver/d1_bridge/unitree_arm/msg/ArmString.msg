# D1-T が rt/arm_Command で受け取るコマンド。中身はJSON文字列。
# **フィールド名・パッケージ名・メッセージ名を変えないこと。**
# DDS の型名 unitree_arm::msg::dds_::ArmString_ が SDK 側
# (docker/driver/d1_sdk/src/msg/ArmString_.hpp) と一致しなくなり、
# 機体のサブスクライバとマッチしなくなる。
string data
