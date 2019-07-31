# Zivid & HALCON - Samples

This repository contains code samples for the usage of a **Zivid** 3D camera in **HALCON**.

## Samples list

- [**Capture**](https://github.com/zivid/halcon-samples/blob/master/Capture.hdev) - This example shows how to acquire a 3D color point cloud from the camera and use it to generate a HALCON ObjectModel3D which is then visualized.
- [**CaptureHDR**](https://github.com/zivid/halcon-samples/blob/master/CaptureHDR.hdev) - This example shows how to acquire an HDR image from the camera and use it to generate a HALCON ObjectModel3D which is then visualized.
- [**CaptureHDRCompleteSettings**](https://github.com/zivid/halcon-samples/blob/master/CaptureHDRCompleteSettings.hdev) - This example shows how to acquire an HDR image from the camera (with fully configured settings for each image) and use it to generate a HALCON ObjectModel3D which is then visualized.
- [**CaptureHDRLoop**](https://github.com/zivid/halcon-samples/blob/master/CaptureHDRLoop.hdev) - This example shows how to acquire HDR images from the camera in a loop (while actively changing some HDR settings). Each HDR image is used to generate a HALCON ObjectModel3D which is then visualized.
- [**ConnectToSerialNumberCamera**](https://github.com/zivid/halcon-samples/blob/master/ConnectToSerialNumberCamera.hdev) - This example shows how to connect to a specific Zivid 3D camera based on its serial number.
- [**QuerySettingsAndParameters**](https://github.com/zivid/halcon-samples/blob/master/QuerySettingsAndParameters.hdev) - This example shows how to query information about the image acquisition interface and selected specific parameters of Zivid camera.
- [**ReadPLY**](https://github.com/zivid/halcon-samples/blob/master/ReadPLY.hdev) - This example shows how to import and display a Zivid point cloud from a PLY file.
- [**SurfaceMatchingCreateModelFromFile**](https://github.com/zivid/halcon-samples/blob/master/SurfaceMatchingCreateModel.hdev) - This example shows how to create a model for surface-based matching algorithm integrated into HALCON. This example comes with two models created by this program: a Pringles can (190 g) and a plastic Coca-Cola bottle (0.5 l).
- [**SurfaceMatchingCreateModel**](https://github.com/zivid/halcon-samples/blob/master/SurfaceMatchingCreateModelFromFile.hdev) - This example shows how to create a model for surface-based matching algorithm integrated into HALCON. This example comes with two models created by this program: a Pringles can (190 g) and a plastic Coca-Cola bottle (0.5 l).
- [**SurfaceMatchingFindModelFromFile**](https://github.com/zivid/halcon-samples/blob/master/SurfaceMatchingFindModelFromFile.hdev) - This example shows surface-based 3D matching on data taken with the Zivid camera. The model used for matching is created from a reference view of the object. That model is then searched for in a newly acquired 3D point cloud. This example comes with two existing object models: a Pringles can (190 g) and a plastic Coca-Cola bottle (0.5 l).
- [**SurfaceMatchingFindModel**](https://github.com/zivid/halcon-samples/blob/master/SurfaceMatchingFindModel.hdev) - This example shows surface-based 3D matching on data taken with the Zivid camera. The model used for matching is created from a reference view of the object. That model is then searched for in a newly acquired 3D point cloud. This example comes with two existing object models: a Pringles can (190 g) and a plastic Coca-Cola bottle (0.5 l).
- [**UpdateSettings**](https://github.com/zivid/halcon-samples/blob/master/UpdateSettings.hdev) - This example shows two possible methods of updating camera settings,  immediately and eventually. The first method implies that the execution of every settings configuration command triggers the communication with the camera, updating the specified parameter immediately. The second method is reduced to preconfiguring the settings that are to be updated and then updating them with a specified command. Since the communication with the camera is triggered only once, this method is faster, especially if more than one setting is to be updated.

## Instructions

1. [**Install Zivid Software**](https://www.zivid.com/downloads).
Note: The version tested with Zivid cameras is 1.2.0.

2. [**Install HALCON Software**](https://www.mvtec.com/products/halcon/).
Note: The versions tested with Zivid cameras are 18.11 Progress and 13.0.2 for Windows.

3. [**Set System Variables**](https://help.zivid.com/setting-the-environment-variables-for). This allows accessing a Zivid camera with the HALCON GenICamTL interface.

4. Launch HALCON.

5. Open and run one of the samples.

## Support
If you need assistance with using Zivid cameras, visit our Knowledge Base at [https://help.zivid.com/](https://help.zivid.com/) or contact us at [customersuccess@zivid.com](mailto:customersuccess@zivid.com).

## Licence
Zivid Samples are distributed under the [BSD license](LICENSE).
