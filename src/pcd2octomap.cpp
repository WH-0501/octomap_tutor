/*************************************************************************
	> File Name: src/pcd2octomap.cpp
	> Author: Gao Xiang
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年12月12日 星期六 15时51分45秒
 ************************************************************************/

#include <iostream>
#include <assert.h>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//octomap 
#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>
using namespace std;

int main( int argc, char** argv )
{
    if (argc != 3)
    {
        cout<<"Usage: pcd2octomap <input_file> <output_file>"<<endl;
        return -1;
    }

    string input_file = argv[1], output_file = argv[2];
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::io::loadPCDFile<pcl::PointXYZRGBA> ( input_file, cloud );

    cout<<"point cloud loaded, piont size = "<<cloud.points.size()<<endl;

    //声明octomap变量
    cout<<"copy data into octomap..."<<endl;
    // 创建八叉树对象，参数为分辨率，这里设成了0.05
    octomap::OcTree tree( 0.2 );
	
    octomap::Pointcloud scan;
    printf("=======?");
    for (auto p:cloud.points)
    {
        // 将点云里的点插入到octomap中 
        //tree.updateNode( octomap::point3d(p.x, p.y, p.z), true ); // true 只插入 occupied
	
	/* 
	 * 上面的updateNode只考虑了占用区域，即实际障碍物，未写入未free区域
	 * insertPointCloud(const Pointcloud& scan, 
			    const octomap::point3d& sensor_origin,
                            double maxrange=-1., 
                            bool lazy_eval = false, 
                            bool discretize = false) 
	 */
	scan.push_back(p.x,p.y,p.z);
        //printf("=======?");
    }
    tree.insertPointCloud(&scan,octomap::point3d(0,0,0));
    // 更新octomap
    //tree.updateInnerOccupancy();
    // 存储octomap
    tree.writeBinary( output_file );
    cout<<"done."<<endl;

    return 0;
}
