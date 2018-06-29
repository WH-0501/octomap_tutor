/*************************************************************************
	> File Name: src/joinMap.cpp
	> Author: Gao Xiang
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年12月13日 星期日 14时37分05秒
 ************************************************************************/

#include <iostream>
#include <vector>

// octomap 
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/math/Pose6D.h>
//#include <octomap/Pointcloud.h>

// 使用Eigen的Geometry模块处理3d运动
#include <Eigen/Core>
#include <Eigen/Geometry> 

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

using namespace std;

int main( int argc, char** argv )
{
    // 读关键帧编号
    ifstream fin( "/home/ros/pcd_map/map_generate/id.txt" );
    vector<int> keyframes;
    vector< Eigen::Isometry3d > poses;
    // 把文件 ./data/keyframe.txt 里的数据读取到vector中
    while( fin.peek() != EOF ) // peek：预读取下一个字符
    {
        int index_keyframe;
        fin>>index_keyframe;
        if (fin.fail()) break;
        keyframes.push_back( index_keyframe );
    }
    fin.close();

    cout<<"load total "<<keyframes.size()<<" keyframes. "<<endl;

    // 读关键帧姿态
    // 我的代码中使用了Eigen来存储姿态，类似的，也可以用octomath::Pose6D来做这件事
    fin.open( "/home/ros/pcd_map/map_generate/gps_optimization_pose.txt" );
    while( fin.peek() != EOF )
    {
        int index_keyframe;
        float data[6]; // x, y, z, roll, pitch, yaw
        fin>>index_keyframe;
        for ( int i=0; i<6; i++ )
        {
            fin>>data[i];
            cout<<data[i]<<" ";
        }
        cout<<endl;
        if (fin.fail()) break;
        // 注意这里的顺序。g2o文件四元数按 qx, qy, qz, qw来存
        // 但Eigen初始化按照qw, qx, qy, qz来做
        Eigen::Quaterniond q( data[6], data[3], data[4], data[5] );
        Eigen::Isometry3d t(q);
        t(0,3) = data[0]; t(1,3) = data[1]; t(2,3) = data[2];
        poses.push_back( t );
    }
    fin.close();

    // 拼合全局地图
    octomap::OcTree tree( 0.2 ); //全局map

    // 注意我们的做法是先把图像转换至pcl的点云，进行姿态变换，最后存储成octomap
    // 因为octomap的颜色信息不是特别方便处理，所以采用了这种迂回的方式
    // 所以，如果不考虑颜色，那不必转成pcl点云，而可以直接使用octomap::Pointcloud结构
    char pcd_name[1024];
    for ( size_t i=0; i < /*keyframes.size()*/10; i++ )
    {
        
        int k = keyframes[i];
        Eigen::Isometry3d& pose = poses[i];

		sprintf(pcd_name, "/home/ros/pcd_map/map_generate/gps_global/%dglobal.pcd", k);
		cout<<"converting "<<i<<"th keyframe :" << pcd_name << "..." <<endl;
		
		// 读取cloud数据
		pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    	pcl::io::loadPCDFile<pcl::PointXYZRGBA> ( pcd_name, cloud );

        // 将cloud旋转之后插入全局地图
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_temp( new pcl::PointCloud<pcl::PointXYZRGBA>() );
        pcl::transformPointCloud( cloud, *cloud_temp, pose.matrix() );
		
        octomap::Pointcloud cloud_octo;
        for (auto p:cloud_temp->points)
            cloud_octo.push_back( p.x, p.y, p.z );
        
        tree.insertPointCloud( &cloud_octo, 
        				octomap::point3d( pose(0,3), pose(1,3), pose(2,3) ) );

        //for (auto p:cloud_temp->points)
        //    tree.integrateNodeColor( p.x, p.y, p.z, p.r, p.g, p.b );

		memset(pcd_name, 0, sizeof(pcd_name));
    }
    
    //tree.updateInnerOccupancy();
    tree.write( "/home/ros/pcd_map/map_generate/join_map.bt" );

    cout<<"done."<<endl;
    
    return 0;

}
