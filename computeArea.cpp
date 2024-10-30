#include <pcl/point_types.h>          //PCL中所有点类型定义的头文件
#include <pcl/io/pcd_io.h>            //打开关闭pcd文件的类定义的头文件
#include <pcl/io/ply_io.h>            //打开关闭ply文件的类定义的头文件
#include <pcl/kdtree/kdtree_flann.h>  //kd-tree搜索对象的类定义的头文件
#include <pcl/surface/mls.h>          //最小二乘法平滑处理类定义头文件
#include <pcl/visualization/pcl_visualizer.h> //其实都有就是路径不明确,所以用cmake构建项目更好一点

//pca算法要用的包
#include <pcl/common/pca.h>

//ranrsc算法要用的包
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <Eigen/Dense>
#include <pcl/filters/project_inliers.h>//投影
#include <pcl/ModelCoefficients.h>

//凹凸包算法要用的包
#include <pcl/surface/concave_hull.h>

//无序点云的快速三角化要用的包
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <boost/thread/thread.hpp>

//vtk的包
#include <vtk-7.1/vtkSmartPointer.h>
#include <vtk-7.1/vtkTriangleFilter.h>
#include <vtk-7.1/vtkPLYReader.h>
#include <vtk-7.1/vtkMassProperties.h>

//各种滤波器
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波器

//文件的读取
#include <fstream>
#include <sstream>
#include <vector>

//语句的读入、输出
#include <iostream>
using namespace std;

//开根运算
#include <cmath>


// //PCA+贪婪三角+二维凸包
// void rebuildSurface()
// {
//     string filePath;//定义文件路径
//     cout<<"请输入pcd格式的点云文件地址:"<<endl;
//     cin>>filePath;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//暂时忽略强度信息
//     if (pcl::io::loadPCDFile<pcl::PointXYZ> (filePath, *cloud) == -1) 
//     {
//       cout<<"读取失败"<<endl;
//     }
//     int numA=cloud->points.size();
//     cout<<"原始点云数量:"<<numA<<endl;


//     double mlsSearchRadius;//定义搜索半径
//     cout<<"请输入移动最小二乘的搜索距离:"<<endl;
//     cin>>mlsSearchRadius;
//     // 创建一个KD树
// 	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMls (new pcl::PointCloud<pcl::PointXYZ>);
// 	// 定义对象 (第二种定义类型是为了存储法线, 即使用不到也需要定义出来)
// 	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
// 	mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计
// 	//设置参数
// 	mls.setInputCloud(cloud);
// 	mls.setSearchMethod(tree);
// 	mls.setSearchRadius(mlsSearchRadius);
//     mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);     //对点云进行上采样
//     mls.setUpsamplingRadius(50);    //设置采样半径大小，2cm
//     mls.setUpsamplingStepSize(40);  //设置采样步长大小，1cm
//     mls.process(*cloudMls);
//     pcl::io::savePCDFile ("mls.pcd", *cloudMls);
//     int numB=cloudMls->points.size();
//     cout<<"mls平滑后点云数量:"<<numB<<endl;





//     // 法线估计
//     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
//     tree2->setInputCloud (cloudMls);
//     n.setInputCloud (cloudMls);
//     n.setSearchMethod (tree2);
//     n.setKSearch (10);
//     n.compute (*normals);
//     //* normals should not contain the point normals + surface curvatures

//     // 连接字段
//     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//     pcl::concatenateFields (*cloudMls, *normals, *cloud_with_normals);
//     //* cloud_with_normals = cloud + normals

//     // 创造搜索树
//     pcl::search::KdTree<pcl::PointNormal>::Ptr tree3 (new pcl::search::KdTree<pcl::PointNormal>);
//     tree3->setInputCloud (cloud_with_normals);

    
//     // 初始化对象
//     pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//定义三角化对象
//     pcl::PolygonMesh triangles;//存储最终三角化的网络模型
    
//     // 设置三角形边长的最大值
//     gp3.setSearchRadius (30);         //设置搜索半径radius，来确定三角化时k邻近的球半径。把它设置为输入参数
    
//     // 设置参数
//     gp3.setMu (2.5);                     //设置样本点到最近邻域距离的乘积系数 mu 来获得每个样本点的最大搜索距离，这样使得算法自适应点云密度的变化
//     gp3.setMaximumNearestNeighbors (100);//设置样本点最多可以搜索的邻域数目100 。
//     gp3.setMaximumSurfaceAngle(M_PI/4);  //45 degrees，设置连接时的最大角度 eps_angle ，当某点法线相对于采样点的法线偏离角度超过该最大角度时，连接时就不考虑该点。
//     gp3.setMinimumAngle(M_PI/18);        //10 degrees，设置三角化后三角形的最小角，参数 minimum_angle 为最小角的值。
//     gp3.setMaximumAngle(2*M_PI/3);       //120 degrees，设置三角化后三角形的最大角，参数 maximum_angle 为最大角的值。
//     gp3.setNormalConsistency(false);     //设置一个标志 consistent ，来保证法线朝向一致，如果设置为 true 则会使得算法保持法线方向一致，如果为 false 算法则不会进行法线一致性检查。
    
//     // Get result
//     gp3.setInputCloud (cloud_with_normals);//设置输入点云为有向点云
//     gp3.setSearchMethod (tree3);           //设置搜索方式tree3
//     gp3.reconstruct (triangles);           //重建提取三角化

//     // 保存为PLY文件
//     pcl::io::savePLYFile("result.ply", triangles);
//     // std::cout << triangles;
//     // Additional vertex information
//     std::vector<int> parts = gp3.getPartIDs();//获得重建后每点的 ID, Parts 从 0 开始编号， a-1 表示未连接的点。
//     std::vector<int> states = gp3.getPointStates();
    
//     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("RebuildSurface"));
//     viewer->setBackgroundColor (0, 0, 0);
//     viewer->addPolygonMesh(triangles,"my");



//     vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
 
// 	// mesh  ply 格式的模型
// 	reader->SetFileName("result.ply");
// 	reader->Update();

//     // 三角化， vtkMassProperties 这个只能计算三角化的模型
// 	vtkSmartPointer<vtkTriangleFilter> triangle = vtkSmartPointer<vtkTriangleFilter>::New();
// 	triangle->SetInputData(reader->GetOutput());
// 	triangle->Update();
// 	//  vtkMassProperties  来计算体积和表面积
// 	vtkSmartPointer<vtkMassProperties>  ploy = vtkSmartPointer<vtkMassProperties>::New();
// 	ploy->SetInputData(triangle->GetOutput());
// 	ploy->Update();
 

//     // 计算三角形的面积 并求和
//     double surface_area = ploy->GetSurfaceArea(); // 表面积
//     cout << "vtk面积:" << surface_area << endl;

//     // 主循环
//      while(!viewer->wasStopped()) 
//     {
//         viewer->spinOnce();
//     }
// }

//RANRSC+二维凸包
// void rebuildSurface()
// {
//     string filePath;//定义文件路径
//     cout<<"请输入pcd格式的点云文件地址:"<<endl;
//     cin>>filePath;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//暂时忽略强度信息
//     pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);

//     if (pcl::io::loadPCDFile<pcl::PointXYZ> (filePath, *cloud) == -1) 
//     {
//       cout<<"读取失败"<<endl;
//     }

//     //MLS提高点密度
//     int numA=cloud->points.size();
//     cout<<"原始点云数量:"<<numA<<endl;

//     double mlsSearchRadius;//定义搜索半径
//     cout<<"请输入移动最小二乘的搜索距离:"<<endl;
//     cin>>mlsSearchRadius;

//     double searchRadius;//定义搜索半径
//     cout<<"请输入连接点的最大距离:"<<endl;
//     cin>>searchRadius;
//     // 创建一个KD树
// 	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
// 	// 输出文件中有PointNormal类型，用来存储移动最小二乘法算出的法线
// 	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
// 	// 定义对象 (第二种定义类型是为了存储法线, 即使用不到也需要定义出来)
// 	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
// 	mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计

// 	//设置参数
// 	mls.setInputCloud(cloud);
// 	mls.setPolynomialFit(true);
// 	mls.setSearchMethod(tree);
// 	mls.setSearchRadius(mlsSearchRadius);
//     mls.process(*cloud_with_normals);
//     pcl::io::savePCDFile ("mls.pcd", *cloud_with_normals);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);//暂时忽略强度信息
//     pcl::io::loadPCDFile<pcl::PointXYZ> ("mls.pcd", *cloudB);

//     std::vector<int> inliers;
//     pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloudB));

//     pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
//     ransac.setDistanceThreshold (6); //6毫米
//     ransac.computeModel();
//     ransac.getInliers(inliers);

//     // 创建一个Eigen::VectorXf对象，用于存储平面模型的系数
//     Eigen::VectorXf sacCoefficients;
//     // 获取平面模型的系数
//     ransac.getModelCoefficients(sacCoefficients);
//     // // 打印平面模型的系数
//     // std::cout << "平面模型的系数: " << sacCoefficients << std::endl;
 
//     //定义模型系数对象，并填充对应的数据
//     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//     coefficients->values.resize(4);
//     coefficients->values[0] = sacCoefficients[0];
//     coefficients->values[1] = sacCoefficients[1];
//     coefficients->values[2] = sacCoefficients[2];
//     coefficients->values[3] = sacCoefficients[3];


//     pcl::ProjectInliers<pcl::PointXYZ> proj;
// 	proj.setInputCloud(cloudB);
// 	proj.setModelCoefficients(coefficients);
// 	// proj.setModelType(pcl::SACMODEL_PLANE);
// 	proj.filter(*cloud_projected);

//     float x=coefficients->values[0];
//     float y=coefficients->values[1];
//     float z=coefficients->values[2];    //法向量的Z值
    
//     //归一化
//     float nx= x/sqrt(x*x+y*y+z*z);      
//     float ny= y/sqrt(x*x+y*y+z*z);   
//     float nz= z/sqrt(x*x+y*y+z*z);  
//     Eigen::Vector3f  planeNorm(nx,ny,nz);

//     Eigen::Vector3f  xoyNorm(0.0,0.0,1.0);//xOy平面法向量

//     //计算夹角
//     float theta=acos(nz);

//     Eigen::Vector3f axis = planeNorm.cross(xoyNorm); //叉乘获取旋转轴

//     //对旋转向量进行归一化处理
//     float v1v2= sqrt(axis.x()*axis.x() + axis.y()*axis.y()+ axis.z()*axis.z());
//     axis = axis/v1v2;

//     Eigen::Affine3f ro_matrix = Eigen::Affine3f::Identity();
//     ro_matrix.rotate(Eigen::AngleAxisf(theta,axis));//theta为正时 逆时针旋转，theta为负时 顺时针旋转

//     // Eigen::AngleAxisd ro_vector(-theta, Eigen::Vector3d(axis.x(),axis.y(),axis.z()));
//     // Eigen::Matrix3d ro_matrix = ro_vector.toRotationMatrix(); //旋转矩阵
//     pcl::transformPointCloud(*cloudB, *cloud_transformed, ro_matrix);

//     // pcl::copyPointCloud (*cloud, inliers, *final);
//     //保存结果
//     pcl::io::savePCDFile("projectResult.pcd", *cloud_projected);
//     pcl::io::savePCDFile("roateResult.pcd", *cloud_transformed);

//     //保存凸包中的面要素
//     pcl::PolygonMesh mesh;  
//     pcl::ConvexHull<pcl::PointXYZ> convexHull;  
//     convexHull.setInputCloud(cloud_transformed);
//     convexHull.setDimension(2); 
//     convexHull.setComputeAreaVolume(true);
//     convexHull.reconstruct(mesh);

//     //保存为PLY文件
//     pcl::io::savePLYFile("result.ply", mesh);
//     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("RebuildSurface"));
//     viewer->addPolygonMesh(mesh, "polyline");

//     vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
 
// 	// mesh  ply 格式的模型
// 	reader->SetFileName("/home/ubuntu/Desktop/C++Build/test/pclTest/computeArea/build/result.ply");
// 	reader->Update();

//     // 三角化， vtkMassProperties 这个只能计算三角化的模型
// 	vtkSmartPointer<vtkTriangleFilter> triangle = vtkSmartPointer<vtkTriangleFilter>::New();
// 	triangle->SetInputData(reader->GetOutput());
// 	triangle->Update();
// 	//  vtkMassProperties  来计算体积和表面积
// 	vtkSmartPointer<vtkMassProperties>  ploy = vtkSmartPointer<vtkMassProperties>::New();
// 	ploy->SetInputData(triangle->GetOutput());
// 	ploy->Update();
 

//     // 计算三角形的面积 并求和
//     double surface_area = ploy->GetSurfaceArea(); // 表面积
//     cout << "vtk面积: " << surface_area << endl;

//     // 主循环
//      while(!viewer->wasStopped()) 
//     {
//         viewer->spinOnce();
//     }   

// }


//mls+PCA+vtk面积计算
int rebuildSurface()
{
    string filePath;//定义文件路径
    cout<<"请输入pcd格式的点云文件地址:"<<endl;
    cin>>filePath;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//暂时忽略强度信息
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filePath, *cloud) == -1) 
    {
      cout<<"读取失败"<<endl;
      return 0;
    }

    int numA=cloud->points.size();
    cout<<"原始点云数量:"<<numA<<endl;

    double mlsSearchRadius;//定义搜索半径
    cout<<"请输入移动最小二乘的搜索距离:"<<endl;
    cin>>mlsSearchRadius;

    double searchRadius;//定义搜索半径
    cout<<"请输入连接点的最大距离:"<<endl;
    cin>>searchRadius;
    // 创建一个KD树
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	// 输出文件中有PointNormal类型，用来存储移动最小二乘法算出的法线
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);

	// 定义对象 (第二种定义类型是为了存储法线, 即使用不到也需要定义出来)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true);  //设置在最小二乘计算中需要进行法线估计

	//设置参数
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(mlsSearchRadius);
    mls.process(*cloud_with_normals);
    pcl::io::savePCDFile ("mls.pcd", *cloud_with_normals);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);//暂时忽略强度信息
    pcl::io::loadPCDFile<pcl::PointXYZ> ("mls.pcd", *cloudB);

    int numB=cloudB->points.size();
    cout<<"mls平滑后点云数量:"<<numB<<endl;

    //主成分分析
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloudB);
    pca.project(*cloudB, *cloudPCAprojection);
    //保存投影后结果
    pcl::io::savePCDFile("pcaResult.pcd", *cloudPCAprojection);

    pcl::ConvexHull<pcl::PointXYZ> convexHull;
    convexHull.setInputCloud(cloudPCAprojection);
    convexHull.setDimension(2); //二维凸包
    convexHull.setComputeAreaVolume(true);
    //保存凸包中的面要素
    std::vector<pcl::Vertices> polygons;
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PolygonMesh mesh;  
    convexHull.reconstruct(mesh);
    //保存为PLY文件
    pcl::io::savePLYFile("result.ply", mesh);


    // // 创造搜索树
    // pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);//定义搜索树对象
    // tree2->setInputCloud (cloud_with_normalsB);//利用有向点云构造tree
    
    // // 初始化对象
    // pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;//定义三角化对象
    // pcl::PolygonMesh triangles;//存储最终三角化的网络模型
    
    // // 设置三角形边长的最大值
    // gp3.setSearchRadius (searchRadius);         //设置搜索半径radius，来确定三角化时k邻近的球半径。把它设置为输入参数
    
    // // 设置参数
    // gp3.setMu (2.5);                     //设置样本点到最近邻域距离的乘积系数 mu 来获得每个样本点的最大搜索距离，这样使得算法自适应点云密度的变化
    // gp3.setMaximumNearestNeighbors (100);//设置样本点最多可以搜索的邻域数目100 。
    // gp3.setMaximumSurfaceAngle(M_PI/4);  //45 degrees，设置连接时的最大角度 eps_angle ，当某点法线相对于采样点的法线偏离角度超过该最大角度时，连接时就不考虑该点。
    // gp3.setMinimumAngle(M_PI/18);        //10 degrees，设置三角化后三角形的最小角，参数 minimum_angle 为最小角的值。
    // gp3.setMaximumAngle(2*M_PI/3);       //120 degrees，设置三角化后三角形的最大角，参数 maximum_angle 为最大角的值。
    // gp3.setNormalConsistency(false);     //设置一个标志 consistent ，来保证法线朝向一致，如果设置为 true 则会使得算法保持法线方向一致，如果为 false 算法则不会进行法线一致性检查。
    
    // // Get result
    // gp3.setInputCloud (cloud_with_normalsB);//设置输入点云为有向点云
    // gp3.setSearchMethod (tree2);           //设置搜索方式tree2
    // gp3.reconstruct (triangles);           //重建提取三角化

    // // 保存为PLY文件
    // pcl::io::savePLYFile("result.ply", triangles);
    // // std::cout << triangles;
    // // Additional vertex information
    // std::vector<int> parts = gp3.getPartIDs();//获得重建后每点的 ID, Parts 从 0 开始编号， a-1 表示未连接的点。
    // std::vector<int> states = gp3.getPointStates();
    
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("RebuildSurface"));
    // viewer->setBackgroundColor (0, 0, 0);
    // viewer->addPolygonMesh(triangles,"my");
    
        
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("RebuildSurface"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPolygonMesh(mesh,"my");
    

    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
 
	// mesh  ply 格式的模型
	reader->SetFileName("/home/ubuntu/Desktop/C++Build/test/pclTest/computeArea/build/result.ply");
	reader->Update();

    // 三角化， vtkMassProperties 这个只能计算三角化的模型
	vtkSmartPointer<vtkTriangleFilter> triangle = vtkSmartPointer<vtkTriangleFilter>::New();
	triangle->SetInputData(reader->GetOutput());
	triangle->Update();
	//  vtkMassProperties  来计算体积和表面积
	vtkSmartPointer<vtkMassProperties>  ploy = vtkSmartPointer<vtkMassProperties>::New();
	ploy->SetInputData(triangle->GetOutput());
	ploy->Update();
 
    // 计算三角形的面积 并求和
    double surface_area = ploy->GetSurfaceArea(); // 表面积
    cout << "vtk面积: " << surface_area << endl;
    // double maxArea = ploy->GetMaxCellArea();//最大单元面积
    // double minArea = ploy->GetMinCellArea();//最小单元面积
    // cout << "单元最大面积：   " << maxArea << endl;
    // cout << "单元最小面积：   " << minArea << endl;

    // 主循环
     while(!viewer->wasStopped()) 
    {
        viewer->spinOnce();
    }       
}





int main(int argc, char** argv)
{
    rebuildSurface();
	return 0;
}