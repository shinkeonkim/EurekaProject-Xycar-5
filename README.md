# EurekaProject-Xycar-5

- 국민대학교 창업연계공학설계입문, 유레카프로젝트 강의의 실습, 과제 내용입니다.

## 유레카 프로젝트 5조
||학번 |이름|github username|
|--|--|--|--|
|조장|20191564|김신건|shinkeonkim|
|조원|20191550|곽다윗|dawit0905|
|조원|20191549|고현성|cokwa|
|조원|20191545|강경수|gomtang3274|
|조원|20191565|김예리|yeringii|

## How to make ros package

### github 환경 구축이 안 되는 경우

SSH or VNC로 Xycar에 접속한 후,
```
$cd ~xycar/src
$catkin_create_pkg [package_name] rospy std_msgs [.. depends]

$cd [package_name]
$gedit [code_file_name] // node code copy and paste

$chmod +x [code_file_name]
$cd ~/xycar
$cm  //or $catkin_make

$cd ~/xycar/src/[package_name]
$mkdir launch
$cd launch
$gedit [lauch_file_name].launch // launch file copy and paste

$roslaunch [package_name] [launch_file_name].launch
```
