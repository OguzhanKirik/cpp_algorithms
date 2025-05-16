#pragma once


struct params{
    float edgeToleranceMin =0.38f;
    float edgeToleranceMax = 0.57f;
    float centerToleranceMin =0.75f;
    float centerToleranceMax = 1.32f;

    int fullSizeHole = 40; // pixel

    float angular_resolution = 0.01f;
    float angular_resolution_border = 0.05f; 
    float sensorDistanceOffset = 500.0f;
    float maxCorrespondenceDistance= 10.0f;
    float diameteFullSized = 2.5f;
    float marginBBXaxis = 200.0f;
    float marginBBYaxis = 200.0f;
    float marginBBZaxis = 10.0f;
    //   float marginBBXaxis = 20.0f;
    //   float marginBBYaxis = 20.0f;
    //   float marginBBZaxis = 10.0f;
    float radiusBorderFiltering = 10.0f;
    int numberOfBorderNeighbour = 10;
    Eigen::Vector3f profilometerCalibTranslation{49.2808571, -13.7755957,-73.2848663};
    Eigen::Vector3f profilometerCalibRotation{(float) (179.947632f * (M_PI/180.0f)),(float) (-0.0595062114f * (M_PI/180.0f)),(float) (0.763690412f * (M_PI/180.0f))};
    // Eigen::Vector3f profilometerCalibTranslation{89.75, 55 , 40.097};
    // Eigen::Vector3f profilometerCalibRotation{(float) (-0.85f  * (M_PI/180.0f)),(float) ( -0.2f * (M_PI/180.0f)),(float) (183.5f * (M_PI/180.0f))};
    std::string auditFolder; 
    std::string cloudPath; 

    bool modeOutLierRemovalActive = true;
    bool modeOutLierRemovalActiveForBorder =false;
    float setStddevMulThresh = 1.0f; // outlier removal
    //float setStddevMulThresh = 1.5f; // outlier removal
    float setStddevMulThreshBorder = 0.5f; // outlier removal border

    int meanK = 30;
    //int meanK = 10;
    int meanKBorder = 50;  // outlier removal

    int borderSize = 100; // range image
    float maxAngleWidth = 180.0f; // range image
    float maxAngleHeight = 180.0f; // range image
    
};


struct Quaternion {
    float w, x, y, z;
};

struct EulerAngles {
    float roll, pitch, yaw;
};


struct boundingBox{
    int cluster;
    cv::Point topLeft;
    cv::Point bottomRight;
    cv::Point topRight;
    cv::Point bottomLeft;
    cv::Point center;
    std::string ROI;

    void addMargin(const int& margin){
        topLeft.x-=margin;
        topLeft.y-=margin;
        bottomRight.x+=margin ;
        bottomRight.y+=margin;

    }
    
   
    bool isFullSized() const{
        int width =  topRight.x - bottomLeft.x;
        int height = bottomLeft.y - topRight.y;
        if(width > 40 || height > 40)
            return true;
        else
            return false;
    }
    
};

struct range_image{
    cv::Vec3f orientation;
    cv::Vec3f translation;
    cv::Affine3f sensorPose;
    float image_offset_x;
    float image_offset_y;
    float min;
    float max;
};


struct drillTarget{
    float cluster; // to read correctly
    Eigen::Vector3f  position;
    Eigen::Vector3f  position_aligned;
    Eigen::Vector3f  position_edge;
    Eigen::Vector3f position_neigbour_center;

    Eigen::Vector3f  normals;
    Eigen::Vector3f  normals_cad;

    Eigen::Quaternionf quartenion;
    Eigen::Quaternionf quartenion_cad;
    Eigen::Matrix3f rotation_matrix;
    Eigen::Affine3f homogeneousMat;

    bool piloted = false;
    bool fullSized =false;
    bool unpiloted = false;
    bool withinEdgeTolerance = false;

    bool withinCenterTolerance = false;
    bool misPrediction = false;
    bool differentCluster= false;
    float distanceToEdge= 0.0f;
    float distanceToCenter = 0.0f;
    float detectedHoleDiameter = 0;
    int ID;
    boundingBox BB;
    std::string ROI;

};
