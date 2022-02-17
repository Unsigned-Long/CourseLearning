

# chp4-lee-group

___Author : csl___

___E-Mail : 3079625093@qq.com___

[TOC]

## 1. Project Tree

```cpp
.
├── build
├── CMakeLists.txt
├── img
│   ├── color
│   │   ├── 1.png
│   │   ├── 2.png
│   │   ├── 3.png
│   │   ├── 4.png
│   │   └── 5.png
│   ├── depth
│   │   ├── 1.pgm
│   │   ├── 2.pgm
│   │   ├── 3.pgm
│   │   ├── 4.pgm
│   │   └── 5.pgm
│   ├── distorted.png
│   ├── left.png
│   ├── output
│   │   ├── pcl.png
│   │   ├── room.png
│   │   └── undistorted.png
│   └── right.png
├── pose.txt
├── readme.md
└── src
    ├── depthPointCloud.cpp
    ├── rgbd.cpp
    ├── undistorted.cpp
    └── useopencv.cpp

21 directories, 87 files
```

## 2. Items

### 1. use opencv

source code

```cpp
void useOpenCV() {
    cv::Mat img = cv::imread("../img/distorted.png", cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      std::cout << "image load failed" << std::endl;
      return;
    }
    std::cout << "here are the details of this image:\n";
    std::cout << "{'rows': " << img.rows
              << ", 'cols': " << img.cols
              << ", 'channels': " << img.channels()
              << ", 'type': " << img.type() << "}\n";
    cv::imshow("win", img);
    cv::waitKey(0);

    long long sum = 0;
    ns_timer::Timer<> timer;
    for (int i = 0; i != img.rows; ++i) {
      auto ptr = img.ptr<uchar>(i);
      for (int j = 0; j != img.cols; ++j) {
        sum += ptr[j];
      }
    }
    std::cout << "{sum value of pixels: " << sum
              << "}\n"
              << timer.last_elapsed("cost time") << std::endl;

    cv::Mat img2 = img;

    std::cout << "deep copy result: img not affected\n";
    cv::Mat img3 = img2.clone();
    img3(cv::Rect(0, 0, img.cols / 4, img.rows / 4)) = cv::Mat::zeros(cv::Size2i(img.cols / 4, img.rows / 4), CV_8UC1);
    cv::imshow("win", img);
    cv::waitKey(0);

    img2(cv::Rect(0, 0, img.cols / 4, img.rows / 4)) = cv::Mat::zeros(cv::Size2i(img.cols / 4, img.rows / 4), CV_8UC1);
    std::cout << "sample agsin result: img affected\n";
    cv::imshow("win", img);
    cv::waitKey(0);

    return;
  }
```

output

```cpp
here are the details of this image:
{'rows': 480, 'cols': 752, 'channels': 1, 'type': 0}
{sum value of pixels: 18980449}
{cost time: 0.222164(ms)}
deep copy result: img not affected
sample agsin result: img affected
```

### 2. undistorted

source code

```cpp
void undistorted() {
    cv::Mat img = cv::imread("../img/distorted.png", cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      std::cout << "image load failed" << std::endl;
      return;
    }
    std::cout << "here are the details of this image:\n";
    std::cout << "{'rows': " << img.rows
              << ", 'cols': " << img.cols
              << ", 'channels': " << img.channels()
              << ", 'type': " << img.type() << "}\n";

    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;

    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    cv::Mat dst = cv::Mat(img.size(), CV_8UC1, cv::Scalar(0));
    ns_timer::Timer<> timer;
    for (int i = 0; i != img.rows; ++i) {
      auto dstPtr = dst.ptr<uchar>(i);
      for (int j = 0; j != img.cols; ++j) {
        double u = j, v = i;
        double x = (u - cx) / fx, y = (v - cy) / fy;
        double r_2 = x * x + y * y, r_4 = r_2 * r_2;
        double x_dis = x * (1 + k1 * r_2 + k2 * r_4) + 2 * p1 * x * y + p2 * (r_2 + 2 * x * x);
        double y_dis = y * (1 + k1 * r_2 + k2 * r_4) + 2 * p2 * x * y + p1 * (r_2 + 2 * y * y);

        int u_dis = fx * x_dis + cx, v_dis = fy * y_dis + cy;
        if (u_dis >= 0 && u_dis < img.cols && v_dis >= 0 && v_dis < img.rows) {
          dstPtr[j] = img.ptr<uchar>(v_dis)[u_dis];
        } else {
          dstPtr[j] = 0;
        }
      }
    }

    std::cout << "here is the result " << timer.last_elapsed("cost time")
              << std::endl;

    cv::imshow("win_src", img);
    cv::imshow("win_dst", dst);
    cv::waitKey(0);

    cv::imwrite("../img/undistorted.png", dst);
    return;
  }
```

images

___Before___

<img src="./img/distorted.png">

___After___

<img src="./img/output/undistorted.png">

### 3. RGB-D

source code

```cpp
std::vector<std::string> filesInDir(const std::string &directory) {
    std::vector<std::string> files;
    for (const auto &elem : std::filesystem::directory_iterator(directory))
      if (elem.status().type() != std::filesystem::file_type::directory)
        files.push_back(std::filesystem::canonical(elem.path()).c_str());
    return files;
  }

  void rgbd() {
    auto colorFiles = ns_chp5::filesInDir("../img/color");
    std::sort(colorFiles.begin(), colorFiles.end());
    auto depthFiles = ns_chp5::filesInDir("../img/depth");
    std::sort(depthFiles.begin(), depthFiles.end());
    auto pose = CSV_READ_FILE("../pose.txt", ' ', Item, double, double, double, double, double, double, double);

    std::cout << "color files:\n";
    for (const auto &elem : colorFiles)
      std::cout << elem << std::endl;

    std::cout << "\ndepth files:\n";
    for (const auto &elem : depthFiles)
      std::cout << elem << std::endl;

    std::cout << "\npose:\n";
    for (const auto &elem : pose)
      std::cout << elem << std::endl;

    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    ns_timer::Timer<> timer;
    for (int i = 0; i != 5; ++i) {
      cv::Mat color = cv::imread(colorFiles.at(i), cv::IMREAD_UNCHANGED);
      cv::Mat depth = cv::imread(depthFiles.at(i), cv::IMREAD_UNCHANGED);
      auto &cam_pose = pose.at(i);
      for (int j = 0; j != color.rows; ++j) {
        for (int k = 0; k != color.cols; ++k) {

          auto d = depth.at<ushort>(j, k);
          if (d == 0)
            continue;

          double u = k, v = j;
          double scalar = double(d) / depthScale;
          Eigen::Vector3d p((u - cx) * scalar / fx, (v - cy) * scalar / fy, scalar);

          Eigen::Quaterniond q(cam_pose.qw(), cam_pose.qx(), cam_pose.qy(), cam_pose.qz());
          Eigen::Vector3d t(cam_pose.tx(), cam_pose.ty(), cam_pose.tz());

          Eigen::Vector3d p_w = q.toRotationMatrix() * p + t;

          auto c = color.at<cv::Vec3b>(j, k);

          cloud->push_back(pcl::PointXYZRGB(p_w(0), p_w(1), p_w(2), c[2], c[1], c[0]));
        }
      }
    }

    std::cout << timer.last_elapsed("cost time") << std::endl;

    pcl::visualization::CloudViewer viewer("win");

    while (!viewer.wasStopped()) {
      viewer.showCloud(cloud);
    }
    return;
  }
```

___[image](./img/output/pcl.png)___

<img src="./img/output/pcl.png">

### 4. depth point cloud

source code

```cpp
void depth() {
    cv::Mat left = cv::imread("../img/left.png", CV_8UC1);
    cv::Mat right = cv::imread("../img/right.png", CV_8UC1);

    std::cout << "here are the details of images:\n";
    std::cout << "'left': {'rows': " << left.rows
              << ", 'cols': " << left.cols
              << ", 'channels': " << left.channels()
              << ", 'type': " << left.type() << "}\n";
    std::cout << "'right': {'rows': " << right.rows
              << ", 'cols': " << right.cols
              << ", 'channels': " << right.channels()
              << ", 'type': " << right.type() << "}\n";

    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    double b = 0.573;

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);
    cv::Mat dis_sgbm, dis;
    sgbm->compute(left, right, dis_sgbm);
    dis_sgbm.convertTo(dis, CV_32F, 1.0 / 16.0f);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    for (int i = 0; i != left.rows; ++i) {
      for (int j = 0; j != left.cols; ++j) {
        float d = dis.at<float>(i, j);
        if (d <= 1.0f || d >= 96.0f)
          continue;
        double u = j, v = i;
        double x = (u - cx) / fx, y = (v - cy) / fy;
        double i = d / 255.0;
        d = fx * b / d;
        pcl::PointXYZI p(x * d, y * d, d, i);
        cloud->push_back(p);
      }
    }

    pcl::visualization::CloudViewer viewer("win");

    while (!viewer.wasStopped()) {
      viewer.showCloud(cloud);
    }
    return;
  }
```

<img src="./img/output/room.png">

### 5. cmake

```cmake
cmake_minimum_required(VERSION 3.10)

project(cameramodel VERSION 1.0)

find_package(OpenCV)
find_package(Sophus)
find_package(Eigen3)
find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)

set (CMAKE_DISABLE_FIND_PACKAGE_fmt ON)

include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Release")

add_executable(${CMAKE_PROJECT_NAME}_useopencv ${CMAKE_SOURCE_DIR}/src/useopencv.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME}_useopencv PRIVATE ${OpenCV_LIBS})

add_executable(${CMAKE_PROJECT_NAME}_undistorted ${CMAKE_SOURCE_DIR}/src/undistorted.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME}_undistorted PRIVATE ${OpenCV_LIBS})


add_executable(${CMAKE_PROJECT_NAME}_depthPointCloud ${CMAKE_SOURCE_DIR}/src/depthPointCloud.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME}_depthPointCloud PRIVATE 
                      ${OpenCV_LIBS}
                      ${PCL_LIBRARIES})

 add_executable(${CMAKE_PROJECT_NAME}_rgbd ${CMAKE_SOURCE_DIR}/src/rgbd.cpp)

target_include_directories(${CMAKE_PROJECT_NAME}_rgbd PRIVATE ${SOPHUS_INCLUDE_DIRS})

target_link_libraries(${CMAKE_PROJECT_NAME}_rgbd PRIVATE 
                      ${OpenCV_LIBS}
                      ${PCL_LIBRARIES}
                      Sophus::Sophus)
```

