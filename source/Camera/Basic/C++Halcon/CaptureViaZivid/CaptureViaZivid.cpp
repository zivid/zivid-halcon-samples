#include <Zivid/Zivid.h>
#include <halconcpp/HalconCpp.h>

#include <chrono>
#include <iostream>


void savePointCloud(const HalconCpp::HObjectModel3D &model, const std::string &filename)
{
	model.WriteObjectModel3d(HalconCpp::HString{ "ply" },
		HalconCpp::HString{ filename.c_str() },
		HalconCpp::HString{ "invert_normals" },
		HalconCpp::HString{ "false" });
}


HalconCpp::HObjectModel3D zividToHalconPointCloud(const Zivid::PointCloud &pointCloud)
{
	const auto width = pointCloud.width();
	const auto height = pointCloud.height();

	std::vector<float> pointsX(width * height);
	std::vector<float> pointsY(width * height);
	std::vector<float> pointsZ(width * height);
	std::vector<float> colorsR(width * height);
	std::vector<float> colorsG(width * height);
	std::vector<float> colorsB(width * height);
	std::vector<int> rowMapping(width*height);
	std::vector<int> colMapping(width*height);

	int numValidPoints = 0;
	for (size_t i = 0; i < width*height; ++i)
	{
		const auto &point = pointCloud(i);
		if (!isnan(point.x))
		{
			pointsX[numValidPoints] = point.x;
			pointsY[numValidPoints] = point.y;
			pointsZ[numValidPoints] = point.z;
			colorsR[numValidPoints] = point.red();
			colorsG[numValidPoints] = point.green();
			colorsB[numValidPoints] = point.blue();
			rowMapping[numValidPoints] = i / width;
			colMapping[numValidPoints] = i % width;
			numValidPoints++;
		}
	}

	std::cout << "Constructing ObjectModel3D based on XYZ data" << std::endl;
	HalconCpp::HObjectModel3D objectModel3D(
		HalconCpp::HTuple{ pointsX.data(), static_cast<long>(numValidPoints) },
		HalconCpp::HTuple{ pointsY.data(), static_cast<long>(numValidPoints) },
		HalconCpp::HTuple{ pointsZ.data(), static_cast<long>(numValidPoints) }
	);

	std::cout << "Mapping ObjectModel3D data" << std::endl;
	std::vector<int> attribValues = { static_cast<int>(width), static_cast<int>(height) };
	attribValues.insert(attribValues.end(), rowMapping.begin(), rowMapping.begin() + static_cast<int>(numValidPoints));
	attribValues.insert(attribValues.end(), colMapping.begin(), colMapping.begin() + static_cast<int>(numValidPoints));
	HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "xyz_mapping", "object", HalconCpp::HTuple{ attribValues.data(), static_cast<long>(attribValues.size()) });

	std::cout << "Adding RGB to ObjectModel3D" << std::endl;
	HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "red", "points", HalconCpp::HTuple{ colorsR.data(), static_cast<long>(numValidPoints) });
	HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "green", "points", HalconCpp::HTuple{ colorsG.data(), static_cast<long>(numValidPoints) });
	HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "blue", "points", HalconCpp::HTuple{ colorsB.data(), static_cast<long>(numValidPoints) });

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
		camera << Zivid::Settings::Iris{ 21 }
			<< Zivid::Settings::ExposureTime{ std::chrono::microseconds{ 10000 } }
			<< Zivid::Settings::Filters::Outlier::Enabled::yes
			<< Zivid::Settings::Filters::Outlier::Threshold{ 5 };

		std::cout << "Capturing frame" << std::endl;
		auto frame = camera.capture();
		const auto zividPointCloud = frame.getPointCloud();

		std::cout << "Converting to Halcon point cloud" << std::endl;
		const auto halconPointCloud = zividToHalconPointCloud(zividPointCloud);

		const auto fileName = "Zivid3D.ply";
		std::cout << "Saving point cloud to: " << fileName << std::endl;
		savePointCloud(halconPointCloud, fileName);
	}
	catch (const std::exception &e)
	{
		std::cerr << "Error: " << Zivid::toString(e) << std::endl;
		return EXIT_FAILURE;
	}
}