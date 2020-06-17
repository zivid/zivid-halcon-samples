using System;

class Program
{
    static void Main()
    {
        try
        {
            Console.WriteLine("Connecting to camera");
            var zividDevice = GetFirstAvailableZividDevice();
            var framegrabber = new HalconDotNet.HTuple();
            HalconDotNet.HOperatorSet.OpenFramegrabber("GenICamTL",
                                    1,
                                    1,
                                    0,
                                    0,
                                    0,
                                    0,
                                    "progressive",
                                    -1,
                                    "default",
                                    -1,
                                    "false",
                                    "default",
                                    zividDevice,
                                    0,
                                    0,
                                    out framegrabber);

            Console.WriteLine("Configuring 3D-settings");
            HalconDotNet.HOperatorSet.SetFramegrabberParam(framegrabber, "create_objectmodel3d", "enable");
            HalconDotNet.HOperatorSet.SetFramegrabberParam(framegrabber, "add_objectmodel3d_overlay_attrib", "enable");
            HalconDotNet.HOperatorSet.SetFramegrabberParam(framegrabber, "AcquisitionMode", "SingleFrame");

            Console.WriteLine("Configuring camera settings");
            HalconDotNet.HOperatorSet.SetFramegrabberParam(framegrabber, "Aperture", 5.66);
            HalconDotNet.HOperatorSet.SetFramegrabberParam(framegrabber, "ExposureTime", 8333);
            HalconDotNet.HOperatorSet.SetFramegrabberParam(framegrabber, "Gain", 1);
            HalconDotNet.HOperatorSet.SetFramegrabberParam(framegrabber, "ProcessingFiltersOutlierRemovalEnabled", 1);
            HalconDotNet.HOperatorSet.SetFramegrabberParam(framegrabber, "ProcessingFiltersOutlierRemovalThreshold", 5);

            Console.WriteLine("Capturing frame");
            var frame = new HalconDotNet.HObject();
            var region = new HalconDotNet.HObject();
            var contours = new HalconDotNet.HObject();
            var data = new HalconDotNet.HTuple();
            HalconDotNet.HOperatorSet.GrabData(out frame, out region, out contours, framegrabber, out data);

            var x = frame.SelectObj(1);
            var y = frame.SelectObj(2);
            var z = frame.SelectObj(3);
            var snr = frame.SelectObj(4);
            var rgb = frame.SelectObj(5);

            Console.WriteLine("Removing invalid 3D points (zeroes)");
            var reducedRegion = new HalconDotNet.HObject();
            var zReduced = new HalconDotNet.HObject();
            HalconDotNet.HOperatorSet.Threshold(z, out reducedRegion, 0.0001, 10000);
            HalconDotNet.HOperatorSet.ReduceDomain(z, reducedRegion, out zReduced);

            Console.WriteLine("Constructing ObjetModel3D based on XYZ data");
            var objectModel3D = new HalconDotNet.HTuple();
            HalconDotNet.HOperatorSet.XyzToObjectModel3d(x, y, zReduced, out objectModel3D);

            Console.WriteLine("Adding RGB to ObjectModel3D");
            SetColorsInObjectModel3D(objectModel3D, rgb, zReduced);

            string pointCloudFile = "Zivid3D.ply";
            Console.WriteLine("Saving point cloud to file: " + pointCloudFile);
            SaveHalconPointCloud(objectModel3D, pointCloudFile);
        }
        catch (Exception ex)
        {
            Console.WriteLine("Error: " + ex.Message);
            Environment.ExitCode = 1;
        }
    }

    private static void SaveHalconPointCloud(HalconDotNet.HTuple model, string fileName)
    {
        HalconDotNet.HOperatorSet.WriteObjectModel3d(model, "ply", fileName, "invert_normals", "false");
    }

    private static void SetColorsInObjectModel3D(HalconDotNet.HTuple objectModel3D, HalconDotNet.HObject RGB, HalconDotNet.HObject zReduced)
    {
        var domain = new HalconDotNet.HObject();
        var rows = new HalconDotNet.HTuple();
        var cols = new HalconDotNet.HTuple();

        HalconDotNet.HOperatorSet.GetDomain(zReduced, out domain);
        HalconDotNet.HOperatorSet.GetRegionPoints(domain, out rows, out cols);

        var objectRed = new HalconDotNet.HObject();
        var objectGreen = new HalconDotNet.HObject();
        var objectBlue = new HalconDotNet.HObject();

        HalconDotNet.HOperatorSet.AccessChannel(RGB, out objectRed, 1);
        HalconDotNet.HOperatorSet.AccessChannel(RGB, out objectGreen, 1);
        HalconDotNet.HOperatorSet.AccessChannel(RGB, out objectBlue, 1);

        var tupleRed = new HalconDotNet.HTuple();
        var tupleGreen = new HalconDotNet.HTuple();
        var tupleBlue = new HalconDotNet.HTuple();

        HalconDotNet.HOperatorSet.GetGrayval(objectRed, rows, cols, out tupleRed);
        HalconDotNet.HOperatorSet.GetGrayval(objectGreen, rows, cols, out tupleGreen);
        HalconDotNet.HOperatorSet.GetGrayval(objectBlue, rows, cols, out tupleBlue);

        HalconDotNet.HOperatorSet.SetObjectModel3dAttribMod(objectModel3D, "red", "points", tupleRed);
        HalconDotNet.HOperatorSet.SetObjectModel3dAttribMod(objectModel3D, "green", "points", tupleGreen);
        HalconDotNet.HOperatorSet.SetObjectModel3dAttribMod(objectModel3D, "blue", "points", tupleBlue);
    }

    private static HalconDotNet.HTuple GetFirstAvailableZividDevice()
    {
        var devices = new HalconDotNet.HTuple();
        var information = new HalconDotNet.HTuple();
        HalconDotNet.HOperatorSet.InfoFramegrabber("GenICamTL", "device", out information, out devices);

        var zividDevices = devices.TupleRegexpSelect("Zivid");
        if (zividDevices.Length == 0)
        {
            throw new System.InvalidOperationException("No Zivid devices found. Please check your setup.");
        }
        return zividDevices[0];
    }
}
