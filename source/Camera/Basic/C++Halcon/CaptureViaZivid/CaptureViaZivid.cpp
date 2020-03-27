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

	std::vector<float> array1dX(width * height, 0.0);
	std::vector<float> array1dY(width * height, 0.0);
	std::vector<float> array1dZ(width * height, 0.0);
	std::vector<float> array1dR(width * height, 0.0);
	std::vector<float> array1dG(width * height, 0.0);
	std::vector<float> array1dB(width * height, 0.0);

	for (size_t i = 0; i < width*height; ++i)
	{
		const auto &point = pointCloud(i);
		if (!isnan(point.x))
		{
			array1dX[i] = point.x;
			array1dY[i] = point.y;
			array1dZ[i] = point.z;
			array1dR[i] = point.red();
			array1dG[i] = point.green();
			array1dB[i] = point.blue();
		}
	}

	std::cout << "Constructing ObjectModel3D based on XYZ data" << std::endl;
	const auto X = HalconCpp::HImage{ "real", static_cast<long>(width), static_cast<long>(height), array1dX.data() };
	const auto Y = HalconCpp::HImage{ "real", static_cast<long>(width), static_cast<long>(height), array1dY.data() };
	const auto Z = HalconCpp::HImage{ "real", static_cast<long>(width), static_cast<long>(height), array1dZ.data() };
	HalconCpp::HObjectModel3D objectModel3D(X, Y, Z);

	std::cout << "Adding RGB to ObjectModel3D" << std::endl;
	HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "red", "points", HalconCpp::HTuple{ array1dR.data(), static_cast<long>(width) * static_cast<long>(height) });
	HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "green", "points", HalconCpp::HTuple{ array1dG.data(), static_cast<long>(width) * static_cast<long>(height) });
	HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "blue", "points", HalconCpp::HTuple{ array1dB.data(), static_cast<long>(width) * static_cast<long>(height) });

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
		camera << Zivid::Settings::Iris{ 20 }
			<< Zivid::Settings::ExposureTime{ std::chrono::microseconds{ 8333 } }
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