#pragma once

#include<string>
#include<iostream>
#include<fstream>
#include <vector>
#include<memory> // NEW

#include <json/value.h> // read detected bounding box coordianates
#include <json/json.h>// read detected bounding box coordianates

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <pcl/visualization/pcl_visualizer.h>

#include<pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>




#include "data_type.h"



    void writeDetectionandInspection(const std::string& path, const std::vector<drillTarget>& allDrillTargets ){
           
        std::ofstream file (path);
        file.precision(15);
        file  <<" cluster " <<" ID "<<" x " << " y "<<" z " <<" qw "<<" qx " <<" qy " <<" qz " <<" position_edge.x " <<" position_edge.y " << " position_edge.z " 
        <<" position_neigbour_center.x " <<" position_neigbour_center.y " << " position_neigbour_center.z " <<" unpiloted "<<" piloted " <<" fullSized " << " EdgeTolerance " << " Edge Distance " <<" Center Tolerance" << " Center Distance " <<"misprediction" << "differentCluster" << std::endl;
        for(const auto dt : allDrillTargets){
            if(dt.differentCluster == false && dt.misPrediction == false && dt.unpiloted  == false){
                file  << (int)dt.cluster << ";" << dt.position_aligned.x() <<";"<< dt.position_aligned.y() <<";"<<dt.position_aligned.z() <<
                 ";" << dt.quartenion_cad.w() <<";"<< dt.quartenion_cad.x() <<";"<< dt.quartenion_cad.y()  <<";"<< dt.quartenion_cad.z() << 
                 //";" << dt.position_edge.x() << ";" << dt.position_edge.y() << ";" << dt.position_edge.z() << 
                // ";" << dt.position_neigbour_center.x() << ";" << dt.position_neigbour_center.y() << ";" << dt.position_neigbour_center.z() << 
                // ";" << dt.unpiloted << ";"  << dt.piloted << ";" << dt.fullSized << ";" << dt.withinEdgeTolerance  << ";" << dt.distanceToEdge  << 
                 //";" << dt.withinCenterTolerance << ";" << dt.distanceToCenter << ";" << dt.misPrediction << ";" << dt.differentCluster << 
                 std::endl;
            }
            
        }

      file.close();

    //            std::string points_new =  "/home/oguz/Desktop/dataset_pointclouds/audit_15/drillTargets_cluster_15_newQuartenion.txt";
    //     std::ofstream file_new (points_new);
    //     file_new  <<" cluster " <<" x " << " y "<<" z "<< " qw " <<" qx " <<" qy "<<" qz " <<" unpiloted "<<" piloted " 
    //     <<" fullSized "<< " edgeTolerance " 
    //     <<" upperCenterTolerance "<<" lowerCenterTolerance" 
    //     <<" misPrediction" <<" differentCluster" <<std::endl;
    //     for(const auto dt : allDrillTargets_inspected){
    //     file_new  << (int)dt.cluster <<";" << dt.position_aligned.x() <<";"<< dt.position_aligned.y() <<";"<<dt.position_aligned.z() <<";"<<
    //     dt.quartenion.w() <<";"<< dt.quartenion.x()<<";"<< dt.quartenion.y() <<";"<< dt.quartenion.z() << 
    //     ";"<< dt.unpiloted <<";"<< dt.piloted <<";"<< dt.fullSized <<";"<< dt.withinEdgeTolerance <<
    //     ";"<< dt.withinCenterToleranceUpper <<";"<< dt.withinCenterToleranceLower <<";"<< dt.misPrediction << ";"<< dt.differentCluster  << std::endl;
    //     }
    //     file_new.close();

    }


    //       void showCircles(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_all,const std::string& cloudPath,const std::vector<pcl::ModelCoefficients>& coefficient_all){
                      
    //     // Show bounding boxes, center and normals of the detected holes
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //     if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
    //         throw std::runtime_error("Couldn't read cloud pcd file \n");}
    //     pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Circle Holes"));
    //     viewer->setBackgroundColor (0, 0, 0);
    //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud_BB_all, 255, 255, 0);
        
    //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cloud, 144, 144, 255);
    //     viewer->addPointCloud<pcl::PointXYZ> (cloud, color3, "cloud");
    //     viewer->addPointCloud<pcl::PointXYZ> (cloud_BB_all, color1, "bb");
    //     std::vector<pcl::ModelCoefficients> circle_coeffs;
    //     for (size_t i = 0; i < coefficient_all.size(); i++){
    //         pcl::ModelCoefficients circle;
    //         circle.values.emplace_back(coefficient_all.at(i).values.at(0));
    //         circle.values.emplace_back(coefficient_all.at(i).values.at(1));
    //         circle.values.emplace_back(coefficient_all.at(i).values.at(2));
    //         circle_coeffs.emplace_back(circle);
    //     }
        
    //     for (int i = 0; i < circle_coeffs.size(); i++){
    //         viewer->addCircle(circle_coeffs.at(i),cv::format("circle%d",i));
    //         //viewer->addLine(add_lines.at(i-1),add_lines.at(i),122.0,133.0,250.0,cv::format("line%d",i));
    //         }
        
    //     while (!viewer->wasStopped ()) {viewer->spinOnce ();}    

    // }

    void showExtractedHoles(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_all,const std::string& cloudPath,const std::vector<pcl::PointXYZ>& add_lines){
                      
        // Show bounding boxes, center and normals of the detected holes
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
            throw std::runtime_error("Couldn't read cloud pcd file \n");}
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Extracted Holes"));
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud_BB_all, 255, 255, 0);
        
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cloud, 144, 144, 255);
        viewer->addPointCloud<pcl::PointXYZ> (cloud, color3, "cloud");
        viewer->addPointCloud<pcl::PointXYZ> (cloud_BB_all, color1, "bb");
        for (int i = 1; i < add_lines.size(); i+=2){
            viewer->addLine(add_lines.at(i-1),add_lines.at(i),122.0,133.0,250.0,cv::format("line%d",i));}
        
        while (!viewer->wasStopped ()) {viewer->spinOnce ();}    

    }

    void showCorrespendences(const std::vector<drillTarget>& detectedHoles_matched, const std::string& cloudPath){

         //##################
            // Show Results of matching
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
                throw std::runtime_error("Couldn't read cloud pcd file \n");}

            pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Correspondences"));
            viewer->setBackgroundColor (0, 0, 0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cloud, 144, 144, 255);
            viewer->addPointCloud<pcl::PointXYZ> (cloud, color3, "cloud_test");
            
            std::vector<pcl::PointXYZ> add_lines;
            for (size_t i = 0; i < detectedHoles_matched.size(); i++){
                if(detectedHoles_matched.at(i).misPrediction == false &&
                 detectedHoles_matched.at(i).differentCluster == false && 
                 detectedHoles_matched.at(i).unpiloted == false){
                    pcl::PointXYZ p_detected{detectedHoles_matched.at(i).position_aligned[0],detectedHoles_matched.at(i).position_aligned[1],detectedHoles_matched.at(i).position_aligned[2]};
                    pcl::PointXYZ p_cad{detectedHoles_matched.at(i).position[0],detectedHoles_matched.at(i).position[1],detectedHoles_matched.at(i).position[2]};
                    add_lines.emplace_back(p_detected);
                    add_lines.emplace_back(p_cad);
                }
            }
            for (int i = 1; i < add_lines.size(); i+=2){
                viewer->addLine(add_lines.at(i-1),add_lines.at(i),90.0,4.0,90.0,cv::format("line_%d",i));
            }
        
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_different_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t i = 0; i < detectedHoles_matched.size(); i++){
                if(detectedHoles_matched.at(i).differentCluster == true ){
                    pcl::PointXYZ p{detectedHoles_matched.at(i).position_aligned[0],detectedHoles_matched.at(i).position_aligned[1],detectedHoles_matched.at(i).position_aligned[2]};
                    cloud_different_cluster->points.emplace_back(p);
            }
            }
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_different(cloud_different_cluster, 12, 0, 255);
            viewer->addPointCloud<pcl::PointXYZ> (cloud_different_cluster, color_different, "cloud_different");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud_different");


            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_misprediction (new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t i = 0; i < detectedHoles_matched.size(); i++){
                if(detectedHoles_matched.at(i).misPrediction == true ){
                    pcl::PointXYZ p{detectedHoles_matched.at(i).position_aligned[0],detectedHoles_matched.at(i).position_aligned[1],detectedHoles_matched.at(i).position_aligned[2]};
                    cloud_misprediction->points.emplace_back(p);
            }
            }
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_misPrediction(cloud_misprediction, 255, 0, 0);
            viewer->addPointCloud<pcl::PointXYZ> (cloud_misprediction, color_misPrediction, "color_misPrediction");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "color_misPrediction");
            while (!viewer->wasStopped ()) {viewer->spinOnce ();}   
            
            // Show Results of matching
            //##################
    }



    void showEdgeDistances(const std::vector<drillTarget>& res, const std::string& cloudPath){
                                        //##################
                // Show Results of matching

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
                if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
                    throw std::runtime_error("Couldn't read cloud pcd file \n");}

                pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Edge Distance"));
                viewer->setBackgroundColor (0, 0, 0);
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cloud, 144, 144, 255);
                viewer->addPointCloud<pcl::PointXYZ> (cloud, color3, "cloud_test");
                


                std::vector<pcl::PointXYZ> add_lines;
                for(const auto p: res){
                    if(p.misPrediction == false && p.differentCluster == false){   
                    pcl::PointXYZ p1{p.position_aligned.x(),p.position_aligned.y(),p.position_aligned.z()};
                    pcl::PointXYZ p2{p.position_edge.x(),p.position_edge.y(),p.position_edge.z()};
                    add_lines.emplace_back(p1);
                    add_lines.emplace_back(p2);
                   }
                }
                
                for (int i = 1; i < add_lines.size(); i+=2){
                    viewer->addLine(add_lines.at(i-1),add_lines.at(i),90.0,4.0,90.0,cv::format("line_%d",i));
                }


            while (!viewer->wasStopped ()) {viewer->spinOnce ();}   
            
            // Show Results of matching
            //##################
    }




    void showCenterDistances(const std::vector<drillTarget>& res, const std::string& cloudPath){
                        //##################
                // Show Results of matching
             
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
                if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
                    throw std::runtime_error("Couldn't read cloud pcd file \n");}

                pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Center Distance"));
                viewer->setBackgroundColor (0, 0, 0);
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cloud, 144, 144, 255);
                viewer->addPointCloud<pcl::PointXYZ> (cloud, color3, "cloud_test");
                


                std::vector<pcl::PointXYZ> add_lines_first;
                for(const auto p: res){
                    if(p.misPrediction == false && p.differentCluster == false){
                    pcl::PointXYZ p1{p.position_aligned.x(),p.position_aligned.y(),p.position_aligned.z()};
                    pcl::PointXYZ p2{p.position_neigbour_center.x(),p.position_neigbour_center.y(),p.position_neigbour_center.z()};
                    add_lines_first.emplace_back(p1);
                    add_lines_first.emplace_back(p2);
                }
                }
                for (int i = 1; i < add_lines_first.size(); i+=2){
                    viewer->addLine(add_lines_first.at(i-1),add_lines_first.at(i),1.0f,0.0f,0.0f,cv::format("line_first_%d",i));
                }
                
                
                
        
                while (!viewer->wasStopped ()) {viewer->spinOnce ();}   
                
                // Show Results of matching
                //##################
    }

    void showNormals(const std::vector<drillTarget>& allDrillTargets, const std::string& cloudPath){
                   //##################
            // Show Results of matching
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
                throw std::runtime_error("Couldn't read cloud pcd file \n");}

            pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Normals"));
            viewer->setBackgroundColor (0, 0, 0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cloud, 144, 144, 255);
            viewer->addPointCloud<pcl::PointXYZ> (cloud, color3, "cloud_test");
            
            std::vector<pcl::PointXYZ> add_lines;
            for (size_t i = 0; i < allDrillTargets.size(); i++){
                if(allDrillTargets.at(i).misPrediction == false &&
                 allDrillTargets.at(i).differentCluster == false){

                    Eigen::Vector3f position_{allDrillTargets.at(i).position_aligned[0],allDrillTargets.at(i).position_aligned[1],allDrillTargets.at(i).position_aligned[2]};
                    Eigen::Vector3f position_shift{0,0,-25};
                    Eigen::Vector3f position_second = allDrillTargets.at(i).rotation_matrix * position_shift;
                    // EulerAngles ea = ToEulerAngles(allDrillTargets.at(i).quartenion_cad);
                    // Eigen::Matrix3f rotation_matrix = rotation_from_euler(ea.roll,ea.pitch,ea.yaw);  
                    // Eigen::Vector3f position_second = rotation_matrix * position_shift;
                    Eigen::Vector3f position_second_= position_ + position_second;
                    pcl::PointXYZ p_second{position_second_[0],position_second_[1],position_second_[2]};
                    pcl::PointXYZ p_first{allDrillTargets.at(i).position_aligned[0],allDrillTargets.at(i).position_aligned[1],allDrillTargets.at(i).position_aligned[2]};
                    add_lines.emplace_back(p_first);
                    add_lines.emplace_back(p_second);
                    
                }
            }
            for (int i = 1; i < add_lines.size(); i+=2){
                viewer->addLine(add_lines.at(i-1),add_lines.at(i),90.0,4.0,90.0,cv::format("line_%d",i));
            }
            while (!viewer->wasStopped ()) {viewer->spinOnce ();}   
            // Show Results of matching
            //##################
    }

    void showBorderPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr border_points){
          pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Border Points"));
        viewer->setBackgroundColor(0, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_border(border_points, 250, 144, 60);

        viewer->addPointCloud<pcl::PointXYZ>(border_points, color_border,"point");
        while(!viewer->wasStopped()) {viewer->spinOnce ();};

    }
    void  showCenters(const std::vector<Eigen::Vector3f>& pos_extracted,const std::vector<Eigen::Vector3f>& pos_cent, const std::string& cloudPath, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_BB_real ){
         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
                throw std::runtime_error("Couldn't read cloud pcd file \n");
            }
         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extracted (new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t i = 0; i < pos_extracted.size(); i++){
            pcl::PointXYZ p{pos_extracted.at(i).x(),pos_extracted.at(i).y(),pos_extracted.at(i).z()};
            cloud_extracted->points.emplace_back(p);
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_centroid (new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t i = 0; i < pos_cent.size(); i++){
            pcl::PointXYZ p{pos_cent.at(i).x(),pos_cent.at(i).y(),pos_cent.at(i).z()};
            cloud_centroid->points.emplace_back(p);
        }

        
        
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("DrillTargets"));
    viewer->setBackgroundColor(0, 0, 0);

    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_(cloud, 250, 144, 60);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "point");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_(cloud_BB_real, 250, 144, 60);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_BB_real, color_,"bb");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_extracted(cloud_extracted, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_extracted, color_extracted,"cloud_extracted");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_extracted");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_centroid(cloud_centroid, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_centroid, color_centroid,"cloud_centroid");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_centroid");
    
 
    while(!viewer->wasStopped()) {viewer->spinOnce ();};
        // ########VISUALIZATION########
    }



void showDrillTargets(const std::vector<drillTarget>& allDrillTargets,const std::string& cloudPath){
         // ########VISUALIZATION########
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
        throw std::runtime_error("Couldn't read cloud pcd file \n");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_piloted (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unpiloted (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fullSized (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_misprediction (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_differentCluster (new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto dT : allDrillTargets){
        if(dT.differentCluster == false && dT.misPrediction == false){
            if(dT.piloted == true ){
               cloud_piloted->points.emplace_back(dT.position_aligned.x(),dT.position_aligned.y(),dT.position_aligned.z());
            }
            else if (dT.unpiloted == true){
                cloud_unpiloted->points.emplace_back(dT.position_aligned.x(),dT.position_aligned.y(),dT.position_aligned.z());

            }else{
                cloud_fullSized->points.emplace_back(dT.position_aligned.x(),dT.position_aligned.y(),dT.position_aligned.z());
            }
        }else{
            if(dT.differentCluster == true && dT.misPrediction == false){
               cloud_differentCluster->points.emplace_back(dT.position_aligned.x(),dT.position_aligned.y(),dT.position_aligned.z());
            }
            else{
                cloud_misprediction->points.emplace_back(dT.position_aligned.x(),dT.position_aligned.y(),dT.position_aligned.z());

            }
            
        }
    }

    
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("DrillTargets"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_(cloud, 250, 144, 60);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, color_,"cloud");


     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_unpiloted(cloud_unpiloted, 144, 144, 144);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_unpiloted, color_unpiloted,"color_unpiloted");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "color_unpiloted");

     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_piloted(cloud_piloted,0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_piloted, color_piloted,"color_piloted");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "color_piloted");


     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_fullSized(cloud_fullSized, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_fullSized, color_fullSized,"color_fullSized");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "color_fullSized");


     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_misprediction(cloud_misprediction, 255, 0,0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_misprediction, color_misprediction,"color_misprediction");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "color_misprediction");


     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_differentCluster(cloud_differentCluster, 255, 144, 60);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_differentCluster, color_differentCluster,"color_differentCluster");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "color_differentCluster");


    while(!viewer->wasStopped()) {viewer->spinOnce ();};
        // ########VISUALIZATION########
}

void extractedDrillTargets(pcl::PointCloud<pcl::PointXYZ>::Ptr pilot_holes_locations_cad,const std::string& cloudPath){
         // ########VISUALIZATION########
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloudPath, *cloud) == -1){
        throw std::runtime_error("Couldn't read cloud pcd file \n");
    }
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("ExtractedCenters"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_(cloud, 250, 144, 60);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, color_,"point");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_main(pilot_holes_locations_cad, 30, 144, 60);
    viewer->addPointCloud<pcl::PointXYZ>(pilot_holes_locations_cad, color_main,"pilot_holes_locations_cad");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "pilot_holes_locations_cad");
    while(!viewer->wasStopped()) {viewer->spinOnce ();};
        // ########VISUALIZATION########
}
    // void normalsCalculation(){

    //     // std::vector<pcl::PointXYZ> add_lines;
    //     // for (size_t i = 0; i < allDrillTargets_inspected.size(); i++){
    //     //     if(allDrillTargets_inspected.at(i).misPrediction == false && allDrillTargets_inspected.at(i).differentCluster == false){

    //     //     Eigen::Vector3f position_center{allDrillTargets_inspected.at(i).position.x(),allDrillTargets_inspected.at(i).position.y(),allDrillTargets_inspected.at(i).position.z()};
    //     //     Eigen::Vector3f position_shift{0,0,-25};
    //     //     // Calculate rotation matrix
    //     //     EulerAngles ea = ToEulerAngles(allDrillTargets_inspected.at(i).quartenion_cad);
    //     //      Eigen::Matrix3f rotation_matrix_Cad = rotation_from_euler(ea.roll,ea.pitch,ea.yaw);
            


    //     //     Eigen::Vector3f position_second = rotation_matrix_Cad * position_shift;
    //     //     Eigen::Vector3f position_second_= position_center + position_second;
    //     //     pcl::PointXYZ p_first{position_center.x(),position_center.y(),position_center.z()};
    //     //     pcl::PointXYZ p_second{position_second_.x(),position_second_.y(),position_second_.z()};
    //     //     add_lines.emplace_back(p_first);
    //     //     add_lines.emplace_back(p_second);

    //     //     Eigen::Vector3f p_dif = p_first.getArray3fMap() - p_second.getArray3fMap();
    //     //     p_dif.normalize();
    //     //     Eigen::Vector3f normals{p_dif.x(),p_dif.y(),p_dif.z()};
    //     //     allDrillTargets_inspected.at(i).normals_cad = normals;
    //     //     //pcl::Normal n{p_dif.x(),p_dif.y(),p_dif.z()};
            
    //     //     }
    //     // }
    // }