#include <Zivid/Zivid.h>
#include <halconcpp/HalconCpp.h>

#include <algorithm>
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

    const auto pointsXYZ = pointCloud.copyPointsXYZ();
    const auto colorsRGBA = pointCloud.copyColorsRGBA();

    int numberOfValidPoints = std::count_if(pointsXYZ.data(),
                                            pointsXYZ.data() + pointsXYZ.size(),
                                            [](const Zivid::PointXYZ &point) { return (!point.isNaN()); });

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

    int validPointIndex = 0;

    for(size_t i = 0; i < height; ++i)
    {
        for(size_t j = 0; j < width; ++j)
        {
            const auto &point = pointsXYZ(i, j);
            const auto &color = colorsRGBA(i, j);
            if(!isnan(point.x))
            {
                tupleX.DArr()[validPointIndex] = point.x;
                tupleY.DArr()[validPointIndex] = point.y;
                tupleZ.DArr()[validPointIndex] = point.z;
                tupleR.LArr()[validPointIndex] = color.r;
                tupleG.LArr()[validPointIndex] = color.g;
                tupleB.LArr()[validPointIndex] = color.b;
                tupleXYZMapping.LArr()[2 + validPointIndex] = i;
                tupleXYZMapping.LArr()[2 + numberOfValidPoints + validPointIndex] = j;

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

        std::cout << "Configuring settings" << std::endl;
        const auto settings =
            Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{
                                 Zivid::Settings::Acquisition::Aperture{ 5.66 },
                                 Zivid::Settings::Acquisition::ExposureTime{ std::chrono::microseconds{ 8333 } } } },
                             Zivid::Settings::Processing::Filters::Outlier::Removal::Enabled::yes,
                             Zivid::Settings::Processing::Filters::Outlier::Removal::Threshold{ 5 },
                             Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
                             Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 } };

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture(settings);
        const auto zividPointCloud = frame.pointCloud();

        std::cout << "Converting to Halcon point cloud" << std::endl;
        const auto halconPointCloud = zividToHalconPointCloud(zividPointCloud);

        const auto pointCloudFile = "Zivid3D.ply";
        std::cout << "Saving point cloud to file: " << pointCloudFile << std::endl;
        savePointCloud(halconPointCloud, pointCloudFile);
    }

    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
