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


void setColorsInObjectModel3D(HalconCpp::HObjectModel3D &objectModel3D, const HalconCpp::HImage &RGB)
{
	const HalconCpp::HRegion domain = RGB.AccessChannel(1).GetDomain();
	HalconCpp::HTuple rows, cols;
	domain.GetRegionPoints(&rows, &cols);

	const HalconCpp::HTuple Rtup = RGB.AccessChannel(1).GetGrayval(rows, cols);
	const HalconCpp::HTuple Gtup = RGB.AccessChannel(2).GetGrayval(rows, cols);
	const HalconCpp::HTuple Btup = RGB.AccessChannel(3).GetGrayval(rows, cols);

	std::cout << "Adding RGB to ObjectModel3D" << std::endl;
	objectModel3D.SetObjectModel3dAttribMod(HalconCpp::HTuple("red"), "points", Rtup);
	objectModel3D.SetObjectModel3dAttribMod(HalconCpp::HTuple("green"), "points", Gtup);
	objectModel3D.SetObjectModel3dAttribMod(HalconCpp::HTuple("blue"), "points", Btup);
}


HalconCpp::HString getFirstAvailableZividDevice(const HalconCpp::HTuple &devices)
{
	auto zividDevices = devices.TupleRegexpSelect("Zivid");
	if (zividDevices.Length() == 0) {
		throw std::runtime_error("No Zivid devices found. Please check your setup.");
	}
	return zividDevices[0];
}


int main()
{
	try
	{
		auto availableDevices = HalconCpp::HTuple();
		auto information = HalconCpp::HTuple();
		HalconCpp::InfoFramegrabber("GenICamTL", "device", &information, &availableDevices);
		const auto zividDevice = getFirstAvailableZividDevice(availableDevices);

		std::cout << "Connecting to camera" << std::endl;
		auto Framegrabber = HalconCpp::HTuple();
		HalconCpp::OpenFramegrabber("GenICamTL", 1, 1, 0, 0, 0, 0, "progressive", -1, "default", -1, "false", "default", zividDevice, 0, 0, &Framegrabber);

		std::cout << "Configuring 3D-settings" << std::endl;
		HalconCpp::SetFramegrabberParam(Framegrabber, "create_objectmodel3d", "enable");
		HalconCpp::SetFramegrabberParam(Framegrabber, "add_objectmodel3d_overlay_attrib", "enable");
		HalconCpp::SetFramegrabberParam(Framegrabber, "AcquisitionMode", "SingleFrame");

		std::cout << "Configuring camera settings" << std::endl;
		HalconCpp::SetFramegrabberParam(Framegrabber, "Iris", 20);
		HalconCpp::SetFramegrabberParam(Framegrabber, "ExposureTime", 8333);
		HalconCpp::SetFramegrabberParam(Framegrabber, "Gain", 2);
		HalconCpp::SetFramegrabberParam(Framegrabber, "Brightness", 1.0);
		HalconCpp::SetFramegrabberParam(Framegrabber, "OutlierFilterEnabled", 1);
		HalconCpp::SetFramegrabberParam(Framegrabber, "GaussianFilterEnabled", 1);
		HalconCpp::SetFramegrabberParam(Framegrabber, "GaussianFilterSigma", 1.5);
		HalconCpp::SetFramegrabberParam(Framegrabber, "OutlierFilterThreshold", 5);

		std::cout << "Capturing frame" << std::endl;
		auto region = HalconCpp::HRegion();
		auto contours = HalconCpp::HXLDCont();
		auto data = HalconCpp::HTuple();
		auto frame = HalconCpp::HImage();
		HalconCpp::GrabData(&frame, &region, &contours, Framegrabber, &data);

		std::cout << "Constructing ObjectModel3D based on XYZ data" << std::endl;
		const auto X = frame.SelectObj(1);
		const auto Y = frame.SelectObj(2);
		const auto Z = frame.SelectObj(3);
		const auto Contrast = frame.SelectObj(4);
		const auto RGB = frame.SelectObj(5);
		HalconCpp::HObjectModel3D objectModel3D(X, Y, Z);

		setColorsInObjectModel3D(objectModel3D, RGB);

		const auto fileName = "Zivid3D.ply";
		std::cout << "Saving point cloud to: " << fileName << std::endl;
		savePointCloud(objectModel3D, fileName);

	}
	catch (const std::exception &e)
	{
		std::cerr << "Error: " << Zivid::toString(e) << std::endl;
		return EXIT_FAILURE;
	}
}