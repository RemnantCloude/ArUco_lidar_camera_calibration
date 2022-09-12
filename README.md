# LiDAR-Camera Calibration using 3D-3D Point correspondences

## 简介

原项目来自[LiDAR-Camera Calibration using 3D-3D Point correspondences](https://github.com/ankitdhall/lidar_camera_calibration)，修改用于对相机和Robosense LiDAR联合标定。

### 参考资料

- [lidar_camera_calibration项目——激光雷达和相机联合标定](https://blog.csdn.net/zhanghm1995/article/details/87802656)
- [SLAM学习笔记（二十一）3D雷达与相机的标定方法详细教程](https://blog.csdn.net/zkk9527/article/details/121330088)
- [Velodyne-ZED-Calibration标定记录](https://sunjiadai.xyz/blog/2019/04/12/Velodyne-ZED-calibration%E6%A0%87%E5%AE%9A%E8%AE%B0%E5%BD%95/)
- [Aruco图案生成](https://chev.me/arucogen/https://blog.csdn.net/lixujie666/article/details/80246909)

### 其他依赖

- [RoboSense（速腾聚创）转Velodyne](https://github.com/HViktorTsoi/rs_to_velodyne)

## 标定步骤

### camera_calibration

1. 启动相机节点
2. roslaunch capture_img run.launch 获取图片，使用MATLAB进行标定

### lidar_camera_calibration

1. create dir "/home/user/image_data/"
2. 启动相机和激光雷达节点
   - roslaunch lidar_camera_calibration start_lidar_camera.launch
3. 启动aruco节点，检查是否能识别到标志
   - roslaunch lidar_camera_calibration aruco.launch
4. 启动标定节点
   - roslaunch lidar_camera_calibration calibration.launch

### Attention

0. 注意Aruco标志的方向，要和原项目中保持一致。
1. sudo apt-get install ros-xxx-velodyne
2. 将文件夹lidar_camera_calibration下的dependencies路径下的两个目录aruco_mapping和aruco_ros拷贝到ROS工作空间的src路径下，再进行编译。
3. find_transform.launch启动ArUco mapping节点并修改其中的重映射命令使得获取正确的相机图片话题
   
   ```<remap from="/image_raw" to="xxx"/>```
4. 修改参数设置文件，主要是相机内参设置，分别在（TODO：需要弄清楚aruco里的参数如何设置）
    a. (1.57 -1.57 0)是初始旋转欧拉角，保留默认设置，方便激光雷达坐标系和像素坐标系进行对齐。
5. 注意一些文件被修改过

    1. aruco_mapping.cpp

    ```C++
    if(vis_marker.id == 26)
    {
        vis_marker.scale.x = 0.23;
        vis_marker.scale.y = 0.23;
    }
    else if(vis_marker.id == 582)
    {
        vis_marker.scale.x = 0.24;
        vis_marker.scale.y = 0.24;
    }
    else
    {
        vis_marker.scale.x = marker_size_;
        vis_marker.scale.y = marker_size_;
    }
    ```
    2. markerdetector.cpp
    ```C++
    if(detectedMarkers[i].id == 26)
    {
        //std::cout << "ID26" << std::endl;
        detectedMarkers[i].calculateExtrinsics ( 0.23,camMatrix,distCoeff,setYPerpendicular );
    }
    else if(detectedMarkers[i].id == 582)
    {
        //std::cout << "ID582" << std::endl;
        detectedMarkers[i].calculateExtrinsics ( 0.24,camMatrix,distCoeff,setYPerpendicular );
    }
    else
    {
        //std::cout << "wrong ID!!!!!!!!!!!!!!!!" << std::endl;
        detectedMarkers[i].calculateExtrinsics ( markerSizeMeters,camMatrix,distCoeff,setYPerpendicular );
    }
    ```
5. 启动标定节点前，需要保证aruco标记能够在相机画面内可见。
6. 注意PreprocessUtils.h的lidar线数设置
   
    ```std::vector <std::vector<myPointXYZRID *>> rings(16);```

7. 标定时注意如果是VNC远程连接工控机，需要外接键鼠直接标定，否则需要修改**Corners.cpp**代码。

    ```C++
    while (collected != 3*LINE_SEGMENTS[q]) {

        cv::setMouseCallback("cloud", onMouse, &_point_);

        cv::imshow("cloud", image_edge_laser);
        cv::waitKey(0);
        ++collected;
        std::cout << "collected:" << collected << std::endl;
        if(collected%3==1) //for VNC connect
            polygon.push_back(_point_);
    }
    ```

## 标定后使用

主要用到Final Rotation和Average translation作为[R|t]，其中final_rotation = rotation_avg * lidarToCamera
