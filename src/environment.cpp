/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <typeinfo>

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2); 
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud) {

    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block ---------
    // ----------------------------------------------------

    // ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer, inputCloud, "inputCloud");

    // preprocessing PCD - downsampling and selecting ROI

    inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.2, Eigen::Vector4f(-20, -6, -2, 1), Eigen::Vector4f(30, 7, 1, 1));
    //renderPointCloud(viewer, inputCloud, "filterCloud");

    // bounding box : car's roof
    // Box carTop;
    // carTop.x_min = -1.5;
    // carTop.y_min = -1.7;
    // carTop.z_min = -1.2;
    // carTop.x_max = 2.5;
    // carTop.y_max = 1.7;
    // carTop.z_max = 0;

    // Color carTopColor = Color(75, 0, 130);
    // renderBox(viewer, carTop, 666, carTopColor);

    // using RANSAC to perform segmentation 

    std::unordered_set<int> inliers = pointProcessorI->RansacPlane(inputCloud, 25, 0.3);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

    for(int index = 0; index < inputCloud->points.size(); index++) {
        pcl::PointXYZI point = inputCloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    if(inliers.size()) {
        renderPointCloud(viewer, cloudInliers, "planeCloud", Color(0, 1, 0));
        renderPointCloud(viewer, cloudOutliers, "obstCloud", Color(1, 0, 0));
    }
    else {
        std::cout<<"Could not estimate a planar model for the given dataset!!" << std::endl;
        renderPointCloud(viewer, inputCloud, "Filtered Cloud");
    }

    // perform euclidean clustering
    KdTree* tree = new KdTree;

    // std::cout<<typeid(cloudOutliers->points).name()<<std::endl;
    int i = 0;
    std::vector<std::vector<float>> cloudPoints;
    for (auto point : cloudOutliers->points) {
        const std::vector<float> obsPoints {point.x, point.y, point.z};
        tree->insert(obsPoints, i++);
        cloudPoints.push_back(obsPoints);
    }

    // for (int i = 0; i< cloudOutliers->points.size(); i++)
    //     tree->insert(cloudOutliers->points[i])

    std::vector<std::vector<int>> clusters = pointProcessorI->euclideanCluster(cloudPoints, tree, 0.53, 10, 500);

    // std::cout<<"Found Clusters : "<<clusters.size()<<std::endl;

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
    ProcessPointClouds<pcl::PointXYZ> pointProcessorBox;

    for(std::vector<int> cluster : clusters){

        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud (new pcl::PointCloud<pcl::PointXYZ>());
        for(int indice : cluster)
            clusterCloud->points.push_back(pcl::PointXYZ(cloudPoints[indice][0], cloudPoints[indice][1], cloudPoints[indice][2]));
        renderPointCloud(viewer, clusterCloud, "cluster "+std::to_string(clusterId), colors[clusterId%3]);

        Box box = pointProcessorBox.BoundingBox(clusterCloud);
        renderBox(viewer, box, clusterId, Color(1, 0, 0));
        ++clusterId;
    }

    if(clusters.size() == 0)
        renderPointCloud(viewer, inputCloud, "Filtered Cloud");
}

//testing 
// void cityBlock2(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud) {
//     inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.2, Eigen::Vector4f(-20, -6, -2, 1), Eigen::Vector4f(30, 7, 1, 1));
//     // renderPointCloud(viewer, inputCloud, "filterCloud");

//     // bounding box : car's roof
//     // Box carTop;
//     // carTop.x_min = -1.5;
//     // carTop.y_min = -1.7;
//     // carTop.z_min = -1.2;
//     // carTop.x_max = 2.5;
//     // carTop.y_max = 1.7;
//     // carTop.z_max = 0;

//     // Color carTopColor = Color(75, 0, 130);
//     // renderBox(viewer, carTop, 666, carTopColor);

//     std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(inputCloud, 100, 0.2);
//     renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
//     renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

//     std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.53, 10, 500);

//     int clusterId = 0;
//     std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
//     for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster :  cloudClusters){
//         std::cout<<"Cluster Size: ";
//         pointProcessorI->numPoints(cluster);
//         renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);

//         Box box = pointProcessorI->BoundingBox(cluster);
//         renderBox(viewer, box, clusterId);
//         ++clusterId;
//     }
// }

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer, lidar->position, inputCloud);
    // renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;   // stack
    //ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>(); // heap
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster :  cloudClusters){
        std::cout<<"Cluster Size: ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0); 
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    // cityBlock(viewer);
    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce ();
    // } 

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ()){

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);
        // cityBlock2(viewer, pointProcessorI, inputCloudI);
        streamIterator++;
        if(streamIterator == stream.end())
        streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}