# Local octomap
Создает node для выделения небольшой области.

## Необходимые библиотеки
* open3d
* numpy
* std_msgs
* sensor_msgs
* geometry_msgs
* rospy

## Входные топики:

`/octomap_point_cloud_centers` PointCloud2 - облако точек центров кубиков от octomap_server

`/point` Point - точка-запрос, возле которой необходимо выделить область

## Выходные топики:

`/center_points_local_map` PointCloud2 - облако точек центров кубиков выделенной области

`/marker_array_local_map` MarkerArray - массив кубиков выделенной области

## Запуск
```
cd ~/catkin_ws/src
git clone 
cd ~/catkin_ws
catkin_make
source devel/setup.sh
python src/local_octomap/local_octomap.py
```