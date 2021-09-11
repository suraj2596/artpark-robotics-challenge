rostopic pub -1 /artpark/spine_ctrl/command std_msgs/Float64 "data: 0.6"
rostopic pub -1 /artpark/scissor_1_ctrl/command std_msgs/Float64 "data: -0.4" & rostopic pub -1 /artpark/scissor_2_ctrl/command std_msgs/Float64 "data: 0.4"
rostopic pub -1 /artpark/disc_ctrl/command std_msgs/Float64 "data: 0.8"
rostopic pub -1 /artpark/stem_ctrl/command std_msgs/Float64 "data: 0.2"
rostopic pub -1 /artpark/stem_ctrl/command std_msgs/Float64 "data: -0.2"
rostopic pub -1 /artpark/disc_ctrl/command std_msgs/Float64 "data: -0.8"
rostopic pub -1 /artpark/stem_ctrl/command std_msgs/Float64 "data: -0.0"
rostopic pub -1 /artpark/disc_ctrl/command std_msgs/Float64 "data: -0.0"
rostopic pub -1 /artpark/scissor_1_ctrl/command std_msgs/Float64 "data: -0.2" & rostopic pub -1 /artpark/scissor_2_ctrl/command std_msgs/Float64 "data: -0.2"
rostopic pub -1 /artpark/scissor_1_ctrl/command std_msgs/Float64 "data: 0.2" & rostopic pub -1 /artpark/scissor_2_ctrl/command std_msgs/Float64 "data: 0.2"

rostopic pub -1 /artpark/init_signal std_msgs/Float64 "data: -0.0"

