using System;
using Duration = Zivid.NET.Duration;

class Program
{
    static void Main()
    {
        try
        {
            var zivid = new Zivid.NET.Application();

            Console.WriteLine("Connecting to camera");
            var camera = zivid.ConnectCamera();

            Console.WriteLine("Configuring settings");
            var settings = new Zivid.NET.Settings
            {
                Acquisitions = { new Zivid.NET.Settings.Acquisition{ Aperture = 5.66,
                                                                 ExposureTime = Duration.FromMicroseconds(8333) } },
                Processing = { Filters = { Outlier = { Removal = { Enabled = true, Threshold = 5.0 } } } }
            };

            Console.WriteLine("Capturing frame");
            var frame = camera.Capture(settings);
            var pointCloud = frame.PointCloud;

            Console.WriteLine("Converting to Halcon point cloud");
            HalconDotNet.HObjectModel3D objectModel3D = ZividToHalconPointCloud(pointCloud);

            string pointCloudFile = "Zivid3D.ply";
            Console.WriteLine("Saving point cloud to: " + pointCloudFile);
            SaveHalconPointCloud(objectModel3D, pointCloudFile);
        }
        catch (Exception ex)
        {
            Console.WriteLine("Error: " + ex.Message);
            Environment.ExitCode = 1;
        }
    }

    private static void SaveHalconPointCloud(HalconDotNet.HObjectModel3D model, string fileName)
    {
        model.WriteObjectModel3d("ply", fileName, "invert_normals", "false");
    }

    private static int FindNumberOfValidPoints(float[,,] pointCloud, ulong height, ulong width)
    {
        var numberOfValidPoints = 0;
        for (ulong i = 0; i < height; i++)
        {
            for (ulong j = 0; j < width; j++)
            {
                double x = pointCloud[i, j, 0];
                if (!Double.IsNaN(x))
                {
                    numberOfValidPoints = numberOfValidPoints + 1;
                }
            }
        }
        return numberOfValidPoints;
    }

    private static HalconDotNet.HObjectModel3D ZividToHalconPointCloud(Zivid.NET.PointCloud pointCloud)
    {
        var height = pointCloud.Height;
        var width = pointCloud.Width;
        var pointsXYZ = pointCloud.CopyPointsXYZ();
        var colorsRGBA = pointCloud.CopyColorsRGBA();

        var numberOfValidPoints = FindNumberOfValidPoints(pointsXYZ, height, width);

        // Initializing HTuples which are later filled with data from the Zivid point cloud.
        // tupleXYZMapping is of shape [width, height, rows[], cols[]], and is used for creating xyz mapping.
        // See more at: https://www.mvtec.com/doc/halcon/13/en/set_object_model_3d_attrib.html

        var tupleX = new HalconDotNet.HTuple();
        var tupleY = new HalconDotNet.HTuple();
        var tupleZ = new HalconDotNet.HTuple();
        var tupleR = new HalconDotNet.HTuple();
        var tupleG = new HalconDotNet.HTuple();
        var tupleB = new HalconDotNet.HTuple();
        var tupleXYZMapping = new HalconDotNet.HTuple();

        tupleX[numberOfValidPoints - 1] = (float)0;
        tupleY[numberOfValidPoints - 1] = (float)0;
        tupleZ[numberOfValidPoints - 1] = (float)0;
        tupleR[numberOfValidPoints - 1] = (byte)0;
        tupleG[numberOfValidPoints - 1] = (byte)0;
        tupleB[numberOfValidPoints - 1] = (byte)0;

        tupleXYZMapping[2 * numberOfValidPoints + 2 - 1] = (uint)0;
        tupleXYZMapping[0] = (uint)width;
        tupleXYZMapping[1] = (uint)height;

        var validPointIndex = 0;
        for (uint i = 0; i < height; i++)
        {
            for (uint j = 0; j < width; j++)
            {
                double x = pointsXYZ[i, j, 0];

                if (!Double.IsNaN(x))
                {
                    tupleX[validPointIndex] = pointsXYZ[i, j, 0];
                    tupleY[validPointIndex] = pointsXYZ[i, j, 1];
                    tupleZ[validPointIndex] = pointsXYZ[i, j, 2];
                    tupleR[validPointIndex] = colorsRGBA[i, j, 0];
                    tupleG[validPointIndex] = colorsRGBA[i, j, 1];
                    tupleB[validPointIndex] = colorsRGBA[i, j, 2];
                    tupleXYZMapping[2 + validPointIndex] = i;
                    tupleXYZMapping[2 + numberOfValidPoints + validPointIndex] = j;
                    validPointIndex++;
                }
            }
        }

        Console.WriteLine("Constructing ObjectModel3D based on XYZ data");
        var objectModel3D = new HalconDotNet.HObjectModel3D(tupleX, tupleY, tupleZ);

        Console.WriteLine("Mapping ObjectModel3D data");
        HalconDotNet.HOperatorSet.SetObjectModel3dAttribMod(objectModel3D, "xyz_mapping", "object", tupleXYZMapping);

        Console.WriteLine("Adding RGB to ObjectModel3D");
        HalconDotNet.HOperatorSet.SetObjectModel3dAttribMod(objectModel3D, "red", "points", tupleR);
        HalconDotNet.HOperatorSet.SetObjectModel3dAttribMod(objectModel3D, "green", "points", tupleG);
        HalconDotNet.HOperatorSet.SetObjectModel3dAttribMod(objectModel3D, "blue", "points", tupleB);

        return objectModel3D;
    }
}