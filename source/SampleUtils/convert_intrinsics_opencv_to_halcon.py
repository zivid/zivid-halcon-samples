"""
Read Zivid camera intrinsic parameters (OpenCV model) from YML file or camera,
convert them to internal camera parameters (Halcon model), and save them to .dat file.

Example when reading from file: python convert_intrinsics_opencv_to_halcon.py
         --input-file IntrinsicsOpenCV.yml --model-name "zivid 2"
         --pixels-to-sample "all" --output-file "IntrinsicsHalcon"

Example when reading from camera with settings: python convert_intrinsics_opencv_to_halcon.py
         --input-settings Settings.yml --output-file "IntrinsicsHalcon"

Example when reading from camera without settings: python convert_intrinsics_opencv_to_halcon.py
         --output-file "IntrinsicsHalcon"

The .dat file can be read with read_cam_par HALCON operator which creates a camera parameter
tuple (CameraParam). Alternatively, gen_cam_par_area_scan_polynomial HALCON procedure can be
used to generate a camera parameter tuple (CameraParam).

To generate Zivid camera intrinsic parameters (OpenCV model) for your Zivid camera, run
GetCameraIntrinsics.cpp code sample from https://github.com/zivid/zivid-cpp-samples. An
example YML file for this sample can be found under the main instructions for Zivid samples.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.

"""

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml
from scipy.optimize import minimize


@dataclass
class CameraIntrinsics:
    cx: float
    cy: float
    fx: float
    fy: float
    k1: float
    k2: float
    k3: float
    p1: float
    p2: float

    @classmethod
    def from_file(cls, filepath: Path):
        data = yaml.safe_load(filepath.read_text(encoding="utf-8"))
        cx = data["CameraIntrinsics"]["CameraMatrix"]["CX"]
        cy = data["CameraIntrinsics"]["CameraMatrix"]["CY"]
        fx = data["CameraIntrinsics"]["CameraMatrix"]["FX"]
        fy = data["CameraIntrinsics"]["CameraMatrix"]["FY"]
        k1 = data["CameraIntrinsics"]["Distortion"]["K1"]
        k2 = data["CameraIntrinsics"]["Distortion"]["K2"]
        k3 = data["CameraIntrinsics"]["Distortion"]["K3"]
        p1 = data["CameraIntrinsics"]["Distortion"]["P1"]
        p2 = data["CameraIntrinsics"]["Distortion"]["P2"]
        return cls(cx, cy, fx, fy, k1, k2, k3, p1, p2)

    @classmethod
    def from_camera(cls, camera, settings=None):
        # pylint: disable=import-outside-toplevel
        from zivid.experimental import calibration

        intrinsics = calibration.intrinsics(camera, settings)
        cx = intrinsics.camera_matrix.cx
        cy = intrinsics.camera_matrix.cy
        fx = intrinsics.camera_matrix.fx
        fy = intrinsics.camera_matrix.fy
        k1 = intrinsics.distortion.k1
        k2 = intrinsics.distortion.k2
        k3 = intrinsics.distortion.k3
        p1 = intrinsics.distortion.p1
        p2 = intrinsics.distortion.p2
        return cls(cx, cy, fx, fy, k1, k2, k3, p1, p2)

    def _camera_matrix(self) -> np.ndarray:
        K = np.eye(3)
        K[0, 0] = self.fx
        K[1, 1] = self.fy
        K[0, 2] = self.cx
        K[1, 2] = self.cy
        return K

    def _distortion_coefficients(self) -> np.ndarray:
        return np.array([self.k1, self.k2, self.p1, self.p2, self.k3])

    def undistorted_pixels(self, pixels: np.ndarray) -> np.ndarray:
        undistorted_normalized = cv2.undistortPoints(
            pixels,
            cameraMatrix=self._camera_matrix(),
            distCoeffs=self._distortion_coefficients(),
        )
        return undistorted_normalized * np.array([[[self.fx, self.fy]]]) + np.array([[[self.cx, self.cy]]])


@dataclass
class HalconInternalCameraParameters:
    cx: float
    cy: float
    sx: float
    sy: float
    focus: float
    poly: np.ndarray
    image_size: np.ndarray

    def __init__(
        self,
        intrinsics: CameraIntrinsics,
        pixel_size: float,
        image_size: np.ndarray,
    ):
        self.cx = intrinsics.cx
        self.cy = intrinsics.cy
        self.sx = pixel_size
        self.sy = pixel_size
        self.focus = pixel_size * intrinsics.fx
        self.poly = np.zeros((5))
        self.image_size = image_size

    def undistorted_pixels(self, pixels: np.ndarray) -> np.ndarray:
        x = (pixels[:, :, 0] - self.cx) * self.sx
        y = (pixels[:, :, 1] - self.cy) * self.sy

        r2 = x**2 + y**2
        r4 = r2**2
        r6 = r2 * r4

        x_u = (
            x * (1 + self.poly[0] * r2 + self.poly[1] * r4 + self.poly[2] * r6)
            + self.poly[3] * (r2 + 2 * x**2)
            + 2 * self.poly[4] * x * y
        )
        y_u = (
            y * (1 + self.poly[0] * r2 + self.poly[1] * r4 + self.poly[2] * r6)
            + self.poly[4] * (r2 + 2 * y**2)
            + 2 * self.poly[3] * x * y
        )

        undistorted_pixels = np.zeros(pixels.shape)
        undistorted_pixels[:, :, 0] = x_u / self.sx + self.cx
        undistorted_pixels[:, :, 1] = y_u / self.sy + self.cy
        return undistorted_pixels

    def _dat_text(self) -> str:
        return f"""\
# INTERNAL CAMERA PARAMETERS

ParGroup: Camera: Parameter;
  "Internal camera parameters";

Focus:foc:      {self.focus:.06e};
  DOUBLE:0.0:;
  "Focal length of the lens [meter]";

Poly1:poly1:   {self.poly[0]:.06e};
  DOUBLE::;
  "1st polynomial distortion coefficient";

Poly2:poly2:   {self.poly[1]:.06e};
  DOUBLE::;
  "2nd polynomial distortion coefficient";

Poly3:poly3:   {self.poly[2]:.06e};
  DOUBLE::;
  "3rd polynomial distortion coefficient";

Poly4:poly4:   {self.poly[3]:.06e};
  DOUBLE::;
  "4th polynomial distortion coefficient";

Poly5:poly5:   {self.poly[4]:.06e};
  DOUBLE::;
  "5th polynomial distortion coefficient";

Sx:sx:   {self.sx:.06e};
  DOUBLE:0.0:;
  "Width of a cell on the sensor";

Sy:sy:   {self.sx:.06e};
  DOUBLE:0.0:;
  "Height of a cell on the sensor";

Cx:cx:   {self.cx:.06e};
  DOUBLE::;
  "X-coordinate of the image center";

Cy:cy:   {self.cy:.06e};
  DOUBLE::;
  "Y-coordinate of the image center";

ImageWidth:imgw:  {self.image_size[0]};
  INT:1:32768;
  "Width of the images";

ImageHeight:imgh:  {self.image_size[1]};
  INT:1:32768;
  "Height of the images";"""

    def write_to_dat_file(self, filepath: Path) -> None:
        """Write internal camera parameters (Halcon model) to .dat file.

        Args:
            filepath: Path to .dat file to save HALCON internal camera parameters

        Raises:
            FileExistsError: File path already exists

        """
        if filepath.exists():
            raise FileExistsError(f"{filepath} already exists")
        filepath.write_text(self._dat_text(), encoding="utf-8")


@dataclass
class CameraParameters:
    pixel_size: float
    image_size: np.ndarray
    intrinsics: CameraIntrinsics

    @classmethod
    def from_file(cls, filepath: Path, model_name: str, sampling: str):
        intrinsics = CameraIntrinsics.from_file(filepath)

        if model_name == "zivid 2+":
            pixel_size = 5.48e-6
            image_size = np.array([np.int32(1224), np.int32(1024)])
            if sampling == "all":
                pixel_size /= 2
                image_size *= 2
            elif sampling in ("blueSubsample4x4", "redSubsample4x4", "by4x4"):
                pixel_size *= 2
                image_size = (image_size / 2).astype(np.int32)
        elif model_name == "zivid 2":
            pixel_size = 4.50e-6
            image_size = np.array([np.int32(1944), np.int32(1200)])
            if sampling in ("blueSubsample2x2", "redSubsample2x2"):
                pixel_size *= 2
                image_size = (image_size / 2).astype(np.int32)

        return cls(pixel_size, image_size, intrinsics)

    @classmethod
    def from_camera(cls, settings_path: Optional[Path] = None):
        # pylint: disable=import-outside-toplevel
        import zivid

        app = zivid.Application()
        camera = app.connect_camera()
        settings = zivid.Settings.load(settings_path) if settings_path else None

        intrinsics = CameraIntrinsics.from_camera(camera, settings)
        if camera.info.model in (zivid.CameraInfo.Model.zivid3XL250):
            pixel_size = 5.48e-6
            image_size = np.array([np.int32(1408), np.int32(1408)])  # assume by2x2
            if settings and settings.sampling.pixel == "all":
                pixel_size /= 2
                image_size *= 2
            elif settings and settings.sampling.pixel == "by4x4":
                pixel_size *= 2
                image_size = (image_size / 2).astype(np.int32)
        if camera.info.model in (
            zivid.CameraInfo.Model.zivid2PlusMR130,
            zivid.CameraInfo.Model.zivid2PlusMR60,
            zivid.CameraInfo.Model.zivid2PlusLR110,
        ):
            pixel_size = 5.48e-6
            image_size = np.array([np.int32(1224), np.int32(1024)])  # assume by2x2
            if settings and settings.sampling.pixel == "all":
                pixel_size /= 2
                image_size *= 2
            elif settings and settings.sampling.pixel == "by4x4":
                pixel_size *= 2
                image_size = (image_size / 2).astype(np.int32)
        elif camera.info.model in (
            zivid.CameraInfo.Model.zivid2PlusM130,
            zivid.CameraInfo.Model.zivid2PlusM60,
            zivid.CameraInfo.Model.zivid2PlusL110,
        ):
            pixel_size = 5.48e-6
            image_size = np.array([np.int32(1224), np.int32(1024)])
            if settings and settings.sampling.pixel == "all":
                pixel_size /= 2
                image_size *= 2
            elif settings and settings.sampling.pixel in ("blueSubsample4x4", "redSubsample4x4"):
                pixel_size *= 2
                image_size = (image_size / 2).astype(np.int32)
        elif camera.info.model in (zivid.CameraInfo.Model.zividTwo, zivid.CameraInfo.Model.zividTwoL100):
            pixel_size = 4.50e-6
            image_size = np.array([np.int32(1944), np.int32(1200)])
            if settings and settings.sampling.pixel in ("blueSubsample2x2", "redSubsample2x2"):
                pixel_size *= 2
                image_size = (image_size / 2).astype(np.int32)
        else:
            raise RuntimeError("Unsupported camera model")

        return cls(pixel_size, image_size, intrinsics)


def _cost_function(
    poly: np.ndarray,
    halcon_parameters: HalconInternalCameraParameters,
    intrinsics: CameraIntrinsics,
    pixels: np.ndarray,
) -> float:
    halcon_parameters.poly = poly
    halcon_pixels_undistorted = halcon_parameters.undistorted_pixels(pixels=pixels)
    opencv_pixels_undistorted = intrinsics.undistorted_pixels(pixels=pixels)
    return np.sqrt(np.mean(np.linalg.norm(halcon_pixels_undistorted - opencv_pixels_undistorted, axis=2) ** 2))


def _create_pixels(
    camera_parameters: CameraParameters,
) -> np.ndarray:
    samples_x = 25
    x = np.linspace(0, camera_parameters.image_size[0] - 1, samples_x)
    y = np.linspace(
        0,
        camera_parameters.image_size[1] - 1,
        round(samples_x * camera_parameters.image_size[1] / camera_parameters.image_size[0]),
    )
    xx, yy = np.meshgrid(x, y)
    return np.array([[xx.flatten(), yy.flatten()]]).transpose((2, 0, 1))


def _estimate_halcon_internal_camera_parameters_from_opencv(
    camera_parameters: CameraParameters,
) -> HalconInternalCameraParameters:
    """Estimate internal camera parameters (Halcon model) from intrinsic parameters (OpenCV model).

    Args:
        camera_parameters: Zivid camera parameters (OpenCV intrinsics model)

    Returns:
        Estimated internal camera parameters (Halcon model)

    """
    halcon_parameters = HalconInternalCameraParameters(
        intrinsics=camera_parameters.intrinsics,
        pixel_size=camera_parameters.pixel_size,
        image_size=camera_parameters.image_size,
    )
    pixels = _create_pixels(camera_parameters=camera_parameters)

    optimizer_result = minimize(
        _cost_function,
        np.zeros(5),
        args=(halcon_parameters, camera_parameters.intrinsics, pixels),
        method="nelder-mead",
    )
    halcon_parameters.poly = optimizer_result.x
    return halcon_parameters


def _args() -> argparse.Namespace:
    """Function for taking in arguments from user.

    Returns:
        Argument from user

    """
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--input-intrinsics",
        type=Path,
        help=("Path to Zivid camera intrinsic parameters (OpenCV model) as YML file."),
    )
    group.add_argument(
        "--input-settings",
        type=Path,
        help=("Path to Zivid capture settings to use for getting intrinsics from the camera as YML file."),
    )
    parser.add_argument(
        "--model-name",
        type=str,
        choices=["zivid 2", "zivid 2+"],
        required="--input-intrinsics" in sys.argv,
        help="Zivid camera model",
    )
    parser.add_argument(
        "--pixels-to-sample",
        type=str,
        choices=["all", "blueSubsample2x2", "redSubsample2x2", "blueSubsample4x4", "redSubsample4x4", "by2x2", "by4x4"],
        required="--input-intrinsics" in sys.argv,
        help="Sampling.Pixel setting",
    )
    parser.add_argument(
        "--output-file",
        type=Path,
        required=True,
        help="Path to .dat file to save HALCON internal camera parameters",
    )

    return parser.parse_args()


def _main():
    args = _args()

    if args.input_intrinsics:
        print(f"Reading intrinsics from file '{args.input_intrinsics}'")
        camera_parameters = CameraParameters.from_file(args.input_intrinsics, args.model_name, args.pixels_to_sample)
    else:
        if args.input_settings:
            print("Reading intrinsics from camera with given settings")
            camera_parameters = CameraParameters.from_camera(args.input_settings)
        else:
            print("Reading intrinsics from camera with default settings")
            camera_parameters = CameraParameters.from_camera()

    halcon_internal_camera_parameters = _estimate_halcon_internal_camera_parameters_from_opencv(camera_parameters)
    halcon_internal_camera_parameters.write_to_dat_file(args.output_file)
    print(f"Halcon internal camera parameters saved to {args.output_file}")


if __name__ == "__main__":
    _main()
