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
                                                                 ExposureTime = Duration.FromMicroseconds(10000) } },
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
                float x = pointCloud[i, j, 0];
                if (!float.IsNaN(x))
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
        var normalsXYZ = pointCloud.CopyNormalsXYZ();
        var colorsRGBA = pointCloud.CopyColorsRGBA();

        var numberOfValidPoints = FindNumberOfValidPoints(pointsXYZ, height, width);
        // Initializing HTuples which are later filled with data from the Zivid point cloud.
        // tupleXYZMapping is of shape [width, height, rows[], cols[]], and is used for creating xyz mapping.
        // See more at: https://www.mvtec.com/doc/halcon/13/en/set_object_model_3d_attrib.html

        var tuplePointsX = new HalconDotNet.HTuple();
        var tuplePointsY = new HalconDotNet.HTuple();
        var tuplePointsZ = new HalconDotNet.HTuple();

        var tupleNormalsX = new HalconDotNet.HTuple();
        var tupleNormalsY = new HalconDotNet.HTuple();
        var tupleNormalsZ = new HalconDotNet.HTuple();

        var tupleColorsR = new HalconDotNet.HTuple();
        var tupleColorsG = new HalconDotNet.HTuple();
        var tupleColorsB = new HalconDotNet.HTuple();

        var tupleXYZMapping = new HalconDotNet.HTuple();

        tuplePointsX[numberOfValidPoints - 1] = (float)0;
        tuplePointsY[numberOfValidPoints - 1] = (float)0;
        tuplePointsZ[numberOfValidPoints - 1] = (float)0;

        tupleNormalsX[numberOfValidPoints - 1] = (float)0;
        tupleNormalsY[numberOfValidPoints - 1] = (float)0;
        tupleNormalsZ[numberOfValidPoints - 1] = (float)0;

        tupleColorsR[numberOfValidPoints - 1] = (byte)0;
        tupleColorsG[numberOfValidPoints - 1] = (byte)0;
        tupleColorsB[numberOfValidPoints - 1] = (byte)0;

        tupleXYZMapping[2 * numberOfValidPoints + 2 - 1] = (uint)0;
        tupleXYZMapping[0] = (uint)width;
        tupleXYZMapping[1] = (uint)height;

        var validPointIndex = 0;
        for (uint i = 0; i < height; i++)
        {
            for (uint j = 0; j < width; j++)
            {
                float x = pointsXYZ[i, j, 0];
                float normal = normalsXYZ[i, j, 0];

                if (!float.IsNaN(x))
                {
                    tuplePointsX[validPointIndex] = pointsXYZ[i, j, 0];
                    tuplePointsY[validPointIndex] = pointsXYZ[i, j, 1];
                    tuplePointsZ[validPointIndex] = pointsXYZ[i, j, 2];
                    tupleColorsR[validPointIndex] = colorsRGBA[i, j, 0];
                    tupleColorsG[validPointIndex] = colorsRGBA[i, j, 1];
                    tupleColorsB[validPointIndex] = colorsRGBA[i, j, 2];
                    tupleXYZMapping[2 + validPointIndex] = i;
                    tupleXYZMapping[2 + numberOfValidPoints + validPointIndex] = j;

                    if (!float.IsNaN(normal))
                    {
                        tupleNormalsX[validPointIndex] = normalsXYZ[i, j, 0];
                        tupleNormalsY[validPointIndex] = normalsXYZ[i, j, 1];
                        tupleNormalsZ[validPointIndex] = normalsXYZ[i, j, 2];
                    }
                    validPointIndex++;
                }
            }
        }

        Console.WriteLine("Constructing ObjectModel3D based on XYZ data");
        var objectModel3D = new HalconDotNet.HObjectModel3D(tuplePointsX, tuplePointsY, tuplePointsZ);

        Console.WriteLine("Mapping ObjectModel3D data");
        HalconDotNet.HOperatorSet.SetObjectModel3dAttribMod(objectModel3D, "xyz_mapping", "object", tupleXYZMapping);

        Console.WriteLine("Adding normals to ObjectModel3D");
        var normalsAttribNames = new HalconDotNet.HTuple("point_normal_x", "point_normal_y", "point_normal_z");
        var normalsAttribValues = new HalconDotNet.HTuple(tupleNormalsX, tupleNormalsY, tupleNormalsZ);
        HalconDotNet.HOperatorSet.SetObjectModel3dAttribMod(objectModel3D, normalsAttribNames, "points", normalsAttribValues);

        Console.WriteLine("Adding RGB to ObjectModel3D");
        HalconDotNet.HOperatorSet.SetObjectModel3dAttribMod(objectModel3D, "red", "points", tupleColorsR);
        HalconDotNet.HOperatorSet.SetObjectModel3dAttribMod(objectModel3D, "green", "points", tupleColorsG);
        HalconDotNet.HOperatorSet.SetObjectModel3dAttribMod(objectModel3D, "blue", "points", tupleColorsB);

        return objectModel3D;
    }
}