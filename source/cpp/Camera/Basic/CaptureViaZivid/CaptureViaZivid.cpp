#include <Zivid/Zivid.h>
#include <halconcpp/HalconCpp.h>

#include <chrono>
#include <iostream>

void savePointCloud(const HalconCpp::HObjectModel3D &model, const std::string &fileName)
{
    model.WriteObjectModel3d(HalconCpp::HString{ "ply" },
                             HalconCpp::HString{ fileName.c_str() },
                             HalconCpp::HString{ "invert_normals" },
                             HalconCpp::HString{ "false" });
}

HalconCpp::HObjectModel3D zividToHalconPointCloud(const Zivid::PointCloud &pointCloud)
{
    const auto width = pointCloud.width();
    const auto height = pointCloud.height();

    int numberOfValidPoints = std::count_if(pointCloud.dataPtr(),
                                            pointCloud.dataPtr() + pointCloud.size(),
                                            [](const Zivid::Point &point) { return (!point.isNaN()); });

    // Initializing HTuples which are later filled with data from the Zivid point cloud.
    // tupleXYZMapping is of shape [width, height, rows[], cols[]], and is used for creating xyz mapping.
    // See more at: https://www.mvtec.com/doc/halcon/13/en/set_object_model_3d_attrib.html

    HalconCpp::HTuple tupleX, tupleY, tupleZ, tupleR, tupleB, tupleG, tupleRow, tupleCol, tupleXYZMapping;
    tupleX[numberOfValidPoints - 1] = (float)0.0;
    tupleY[numberOfValidPoints - 1] = (float)0.0;
    tupleZ[numberOfValidPoints - 1] = (float)0.0;
    tupleR[numberOfValidPoints - 1] = (Hlong)0;
    tupleG[numberOfValidPoints - 1] = (Hlong)0;
    tupleB[numberOfValidPoints - 1] = (Hlong)0;

    tupleXYZMapping[2 * numberOfValidPoints + 2 - 1] = (Hlong)0;
    tupleXYZMapping[0] = (Hlong)width;
    tupleXYZMapping[1] = (Hlong)height;

    double *arrayX = tupleX.DArr();
    double *arrayY = tupleY.DArr();
    double *arrayZ = tupleZ.DArr();
    Hlong *arrayR = tupleR.LArr();
    Hlong *arrayG = tupleG.LArr();
    Hlong *arrayB = tupleB.LArr();
    Hlong *arrayXYZMapping = tupleXYZMapping.LArr();

    int validPointIndex = 0;
    size_t idx = 0;
    const auto *pointCloudData = pointCloud.dataPtr();

    for(size_t i = 0; i < width; ++i)
    {
        for(size_t j = 0; j < height; ++j)
        {
            idx = width * j + i;
            if(!isnan(pointCloudData[idx].x))
            {
                arrayX[validPointIndex] = pointCloudData[idx].x;
                arrayY[validPointIndex] = pointCloudData[idx].y;
                arrayZ[validPointIndex] = pointCloudData[idx].z;
                arrayR[validPointIndex] = pointCloudData[idx].red();
                arrayG[validPointIndex] = pointCloudData[idx].green();
                arrayB[validPointIndex] = pointCloudData[idx].blue();
                arrayXYZMapping[2 + validPointIndex] = j;
                arrayXYZMapping[2 + numberOfValidPoints + validPointIndex] = i;

                validPointIndex++;
            }
        }
    }

    std::cout << "Constructing ObjectModel3D based on XYZ data" << std::endl;
    HalconCpp::HObjectModel3D objectModel3D(tupleX, tupleY, tupleZ);

    std::cout << "Mapping ObjectModel3D data" << std::endl;
    HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "xyz_mapping", "object", tupleXYZMapping);

    std::cout << "Adding RGB to ObjectModel3D" << std::endl;
    HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "red", "points", tupleR);
    HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "green", "points", tupleG);
    HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "blue", "points", tupleB);

    return objectModel3D;
}

int main()
{
    try
    {
        std::cout << "Connecting to camera" << std::endl;
        Zivid::Application zivid;
        auto camera = zivid.connectCamera();

        std::cout << "Configuring camera settings" << std::endl;
        camera << Zivid::Settings::Iris{ 21 } << Zivid::Settings::ExposureTime{ std::chrono::microseconds{ 10000 } }
               << Zivid::Settings::Filters::Outlier::Enabled::yes << Zivid::Settings::Filters::Outlier::Threshold{ 5 };

        std::cout << "Capturing frame" << std::endl;
        auto frame = camera.capture();
        const auto zividPointCloud = frame.getPointCloud();

        std::cout << "Converting to Halcon point cloud" << std::endl;
        const auto halconPointCloud = zividToHalconPointCloud(zividPointCloud);

        const auto fileName = "Zivid3D.ply";
        std::cout << "Saving point cloud to: " << fileName << std::endl;
        savePointCloud(halconPointCloud, fileName);
    }

    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
