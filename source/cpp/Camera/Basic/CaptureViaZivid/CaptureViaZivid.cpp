/*
This example shows how to capture a point cloud, with colors, using Zivid SDK, transform it to a Halcon point cloud and save it using Halcon C++ SDK.
*/

#include <Zivid/Zivid.h>
#include <halconcpp/HalconCpp.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <iostream>
#include <numeric>

#include "omp.h"

using namespace std::chrono;

void savePointCloud(const HalconCpp::HObjectModel3D &model, const std::string &fileName)
{
    model.WriteObjectModel3d(HalconCpp::HString{ "ply" },
                             HalconCpp::HString{ fileName.c_str() },
                             HalconCpp::HString{ "invert_normals" },
                             HalconCpp::HString{ "false" });
}

std::vector<size_t> getRowOffsets(const Zivid::Array2D<Zivid::PointXYZ> &pointsXYZ,
                                  size_t width,
                                  size_t height,
                                  size_t &numberOfValidPoints)
{
    std::vector<size_t> numberOfValidPointsInRow(height, 0);

#pragma omp parallel for
    for(size_t i = 0; i < height; ++i)
    {
        size_t validInRow = 0;
        for(size_t j = 0; j < width; ++j)
        {
            if(!pointsXYZ(i, j).isNaN())
            {
                ++validInRow;
            }
        }
        numberOfValidPointsInRow[i] = validInRow;
    }

    numberOfValidPoints = std::accumulate(numberOfValidPointsInRow.begin(), numberOfValidPointsInRow.end(), 0);

    std::vector<size_t> pointOffset(height, 0);

    for(size_t i = 1; i < height; ++i)
    {
        pointOffset[i] = pointOffset[i - 1] + numberOfValidPointsInRow[i - 1];
    }

    return pointOffset;
}

HalconCpp::HObjectModel3D zividToHalconPointCloud(const Zivid::PointCloud &pointCloud)
{
    const auto width = pointCloud.width();
    const auto height = pointCloud.height();

    const auto pointsXYZ = pointCloud.copyPointsXYZ();
    const auto colorsRGBA = pointCloud.copyColorsRGBA();
    const auto normalsXYZ = pointCloud.copyNormalsXYZ();

    auto t1 = steady_clock::now();

    size_t numberOfValidPoints;

    std::vector<size_t> pointOffset = getRowOffsets(pointsXYZ, width, height, numberOfValidPoints);

    // int numberOfValidPoints = std::count_if(pointsXYZ.data(),
    //                                         pointsXYZ.data() + pointsXYZ.size(),
    //                                         [](const Zivid::PointXYZ &point) { return (!point.isNaN()); });

    std::cout << "Valid points: " << numberOfValidPoints << std::endl;

    auto t2 = steady_clock::now();

    // Initializing HTuples which are later filled with data from the Zivid point cloud.
    // tupleXYZMapping is of shape [width, height, rows[], cols[]], and is used for creating xyz mapping.
    // See more at: https://www.mvtec.com/doc/halcon/13/en/set_object_model_3d_attrib.html

    HalconCpp::HTuple tuplePointsX, tuplePointsY, tuplePointsZ, tupleNormalsX, tupleNormalsY, tupleNormalsZ,
        tupleColorsR, tupleColorsB, tupleColorsG, tupleRow, tupleCol, tupleXYZMapping;

    tuplePointsX[numberOfValidPoints - 1] = (float)0.0;
    tuplePointsY[numberOfValidPoints - 1] = (float)0.0;
    tuplePointsZ[numberOfValidPoints - 1] = (float)0.0;
    tupleNormalsX[numberOfValidPoints - 1] = (float)0.0;
    tupleNormalsY[numberOfValidPoints - 1] = (float)0.0;
    tupleNormalsZ[numberOfValidPoints - 1] = (float)0.0;
    tupleColorsR[numberOfValidPoints - 1] = (Hlong)0;
    tupleColorsG[numberOfValidPoints - 1] = (Hlong)0;
    tupleColorsB[numberOfValidPoints - 1] = (Hlong)0;

    tupleXYZMapping[2 * numberOfValidPoints + 2 - 1] = (Hlong)0;
    tupleXYZMapping[0] = (Hlong)width;
    tupleXYZMapping[1] = (Hlong)height;

    auto t3 = steady_clock::now();

#pragma omp parallel for
    for(size_t i = 0; i < height; ++i)
    {
        int validPointIndex = pointOffset[i];
        for(size_t j = 0; j < width; ++j)
        {
            const auto &point = pointsXYZ(i, j);
            const auto &normal = normalsXYZ(i, j);
            const auto &color = colorsRGBA(i, j);

            if(!isnan(point.x))
            {
                tuplePointsX.DArr()[validPointIndex] = point.x;
                tuplePointsY.DArr()[validPointIndex] = point.y;
                tuplePointsZ.DArr()[validPointIndex] = point.z;
                tupleColorsR.LArr()[validPointIndex] = color.r;
                tupleColorsG.LArr()[validPointIndex] = color.g;
                tupleColorsB.LArr()[validPointIndex] = color.b;
                tupleXYZMapping.LArr()[2 + validPointIndex] = i;
                tupleXYZMapping.LArr()[2 + numberOfValidPoints + validPointIndex] = j;

                if(!isnan(normal.x))
                {
                    tupleNormalsX.DArr()[validPointIndex] = normal.x;
                    tupleNormalsY.DArr()[validPointIndex] = normal.y;
                    tupleNormalsZ.DArr()[validPointIndex] = normal.z;
                }

                validPointIndex++;
            }
        }
    }

    auto t4 = steady_clock::now();

    std::cout << "Counting Nans: " << duration_cast<milliseconds>(t2 - t1).count() << "ms" << std::endl;
    std::cout << "Init Halcon mem: " << duration_cast<milliseconds>(t3 - t2).count() << "ms" << std::endl;
    std::cout << "Filter nans: " << duration_cast<milliseconds>(t4 - t3).count() << "ms" << std::endl;

    std::cout << "Constructing ObjectModel3D based on XYZ data" << std::endl;
    HalconCpp::HObjectModel3D objectModel3D(tuplePointsX, tuplePointsY, tuplePointsZ);

    std::cout << "Mapping ObjectModel3D data" << std::endl;
    HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "xyz_mapping", "object", tupleXYZMapping);

    std::cout << "Adding normals to ObjectModel3D" << std::endl;
    HalconCpp::HTuple normalsAttribNames, normalsAttribValues;
    normalsAttribNames.Append("point_normal_x");
    normalsAttribNames.Append("point_normal_y");
    normalsAttribNames.Append("point_normal_z");

    normalsAttribValues.Append(tupleNormalsX);
    normalsAttribValues.Append(tupleNormalsY);
    normalsAttribValues.Append(tupleNormalsZ);

    HalconCpp::SetObjectModel3dAttribMod(objectModel3D, normalsAttribNames, "points", normalsAttribValues);

    std::cout << "Adding RGB to ObjectModel3D" << std::endl;
    HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "red", "points", tupleColorsR);
    HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "green", "points", tupleColorsG);
    HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "blue", "points", tupleColorsB);

    auto t5 = steady_clock::now();

    std::cout << "Finalize Halcon: " << duration_cast<milliseconds>(t5 - t4).count() << "ms" << std::endl;

    return objectModel3D;
}

int main()
{
    try
    {
        std::cout << "Connecting to camera" << std::endl;
        Zivid::Application zivid;
        const auto dataFile = "/home/frontier/Zivid3D.zdf";

        std::cout << "Reading ZDF frame from file: " << dataFile << std::endl;
        const auto frame = Zivid::Frame(dataFile);
        const auto zividPointCloud = frame.pointCloud();

        std::cout << "Converting to Halcon point cloud" << std::endl;
        const auto halconPointCloud = zividToHalconPointCloud(zividPointCloud);

        const auto pointCloudFile = "Zivid3D.ply";
        std::cout << "Saving point cloud to file: " << pointCloudFile << std::endl;
        savePointCloud(halconPointCloud, pointCloudFile);
    }

    catch(HalconCpp::HException &except)
    {
        std::cerr << "Error: " << except.ErrorMessage() << std::endl;
        return EXIT_FAILURE;
    }

    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
