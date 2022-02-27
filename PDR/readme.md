# Pedestrian Dead Reckoning

___Author: ShouLong Chen___

___School: WHU___

___E-Mail: 3079625093@qq.com___

[TOC]

## 1. OverView

PDR is a passive location method. It can obtain the pedestrian step frequency and step size according to sensors such as accelerometers, and then estimate the relative position of travelers according to the orientation.

Based on the Android operating system, this project uses Java as the development language and Android studio as the development platform to realize the PDR algorithm.

## 2. Main Parts

### 1. Coordinate system

![image-20220227162648402](./imgs/image-20220227162648402.png)

### 2. Calculate azimuth

The calculation of azimuth mainly depends on the accelerometer and magnetometer inside the mobile phone:

![image-20220227162605911](./imgs/image-20220227162605911.png)

![image-20220227162728437](./imgs/image-20220227162728437.png)

### 3. Smoothing of acceleration

We use HMA algorithm for smoothing：

![image-20220227162911094](./imgs/image-20220227162911094.png)

![image-20220227163156318](./imgs/image-20220227163156318.png)

![image-20220227163220965](./imgs/image-20220227163220965.png)

### 4. Azimuth smoothing

We use the EMA algorithm：

![image-20220227163002138](./imgs/image-20220227163002138.png)

![image-20220227163311276](./imgs/image-20220227163311276.png)

### 5. SF-SL Model

![image-20220227163353243](./imgs/image-20220227163353243.png)

## 3. Result

![image-20220227163120486](./imgs/image-20220227163120486.png)