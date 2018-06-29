# octomap_tutor
A tutorial about octomap. Please check my blog for detail: cnblogs.com/gaoxiang12

解决PCD转octomap时未更新free node，导致后面使用octomap_server得到的2D occupancy地图没有可用区域的问题

Installation:
Dependency: 
  opencv - sudo apt-get install libopencv-dev
  boost - sudo apt-get install libboost-all-dev
  pcl - follow instructions on http://pointclouds.org/
  octomap - https://github.com/OctoMap/octomap

Build:
  change the octomap path in cmake_modules/octomap-config.cmake
  ./build.sh
  
Usage:
  bin/pcd2octomap <input-file> <output-file>
  bin/pcd2colorOctomap <input-file> <output-file>
  bin/joinmap
  
Example:
  bin/pcd2octomap data/sample.pcd data/sample.bt
    -convert the sample pcd file to octomap file
  bin/pcd2colorOctomap data/sample.pcd data/sample.bt
    -convert the sample pcd file to color octomap file
  bin/joinmap
    - join the maps defined in data/keyframe.txt
