## 用途
* 一些转换工具
* 结合darknet_ros和r3live测量三维坐标

## 透视投影用法
`roslaunch learning_image_transport image_transport.launch`

## 图像转录
`rosrun image_transport republish  raw in:=/zed2i/zed_node/right/image_rect_color  compressed out:=/zed2i/zed_node/right/image_rect_color`

## PointCloud转化为livxo_ros_driver/CustomMsg
`rosrun learning_image_transport point_trans`

## 从txt读取2D导航点
`rosparam load yaml/param.yaml`
`rosrun learning_image_transport nav_goal`

