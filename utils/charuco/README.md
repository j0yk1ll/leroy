# ChArUco Camera Calibration Toolkit

This toolkit contains three scripts that work together:

- `generate.py` — creates a DIN A4 ChArUco calibration board and `board_config.json`.
- `capture.py` — interactively captures calibration images from your camera.
- `calibrate.py` — detects ChArUco corners and estimates camera intrinsics and distortion.

The workflow is:

```text
generate.py
    ↓
print ChArUco board
    ↓
capture.py
    ↓
calibration images
    ↓
calibrate.py
    ↓
K, distortion coefficients, calibration.yaml
```

## 1. Install dependencies

Optionally create a virtual environment:

```bash
python -m venv .venv
source .venv/bin/activate
```

Install the required packages:

```bash
python -m pip install opencv-contrib-python numpy pillow reportlab pyyaml
```

`opencv-contrib-python` is required because ChArUco support is provided by `cv2.aruco`.

## 2. Generate the ChArUco board

Run:

```bash
mkdir calibration_board

python generate.py \
    --output-dir calibration_board
```

This creates:

```text
calibration_board/
├── charuco_A4.pdf
├── charuco_A4.png
└── board_config.json
```

The default board is:

- 5 × 7 chessboard squares
- 35 mm square size
- 26 mm ArUco marker size
- `DICT_5X5_100`
- DIN A4 page

### Print it correctly

Print `charuco_A4.pdf` using **100% / Actual size**.

Disable:

- Fit to page
- Shrink to printable area
- Automatic scaling

After printing, verify that several chessboard squares measure exactly 35 mm and that the reference line measures exactly 100 mm.

For best results, mount the print flat on a rigid backing. Avoid wrinkles or bending.

## 3. Capture calibration images

Run:

```bash
python capture.py \
    --output-dir calibration_images \
    --camera 0
```

To request a specific resolution:

```bash
python capture.py \
    --output-dir calibration_images \
    --camera 0 \
    --width 1280 \
    --height 720
```

You can also request an FPS:

```bash
python capture.py \
    --output-dir calibration_images \
    --camera 0 \
    --width 1280 \
    --height 720 \
    --fps 30
```

The script prints the actual capture mode after opening the camera. Confirm it matches the mode you intend to use on the robot.

### Controls

- `SPACE` or `C` — save the current frame
- `Q` or `ESC` — quit

Images are saved losslessly as PNG files:

```text
calibration_images/
├── calib_001.png
├── calib_002.png
├── calib_003.png
└── ...
```

### Capture recommendations

Aim for roughly **20–40 good images**.

Variety is more important than sheer count. Capture the board:

- in the image center
- near all four corners
- near the left and right edges
- near the top and bottom edges
- at different distances
- tilted left/right
- tilted forward/backward
- rotated in-plane

Do not capture only fronto-parallel views.

Avoid:

- blurry frames
- heavy motion blur
- overexposure
- a board too small for robust detection
- many nearly identical poses

`capture.py` shows a simple sharpness score. It is only a heuristic, but low values are a useful warning.

### Keep the camera configuration fixed

Calibrate in the same camera mode you intend to use during deployment:

- same resolution
- same aspect ratio
- same lens and focus position
- same digital crop
- same stabilization setting
- same zoom setting

Changing these can change the effective intrinsics.

## 4. Calibrate the camera

Run:

```bash
python calibrate.py \
    --config calibration_board/board_config.json \
    --images "calibration_images/*.png" \
    --output calibration.yaml \
    --save-detections detections
```

The script:

1. reads the exact board definition from `board_config.json`
2. detects ChArUco corners
3. rejects images with too few valid corners
4. estimates the camera matrix
5. estimates lens distortion
6. reports reprojection error
7. writes calibration files

Outputs:

```text
calibration.yaml
calibration.npz
detections/
```

The `detections/` directory contains annotated copies of accepted images so you can inspect the detected corners.

## 5. Understanding the output

The intrinsic matrix is:

```text
K =
[ fx   0  cx ]
[  0  fy  cy ]
[  0   0   1 ]
```

where:

- `fx` — focal length in pixels along X
- `fy` — focal length in pixels along Y
- `cx` — principal-point X coordinate
- `cy` — principal-point Y coordinate

`calibrate.py` prints these values and writes them into `calibration.yaml`.

For DepthART Metric, these are the values to provide when using the corresponding raw camera image.

## 6. Distortion

The default calibration uses OpenCV's normal pinhole radial/tangential distortion model.

Typical coefficients are:

```text
k1, k2, p1, p2, k3
```

If you undistort or rectify images before SLAM, depth estimation, or 3-D projection, use the **rectified camera matrix** associated with those rectified images rather than the raw `K`.

`calibrate.py` writes both the raw calibration and a suggested rectified camera matrix.

## 7. Reprojection error

At the end you may see output such as:

```text
RMS reprojection error: 0.31 px
Mean per-view reprojection error: 0.28 px
```

Lower is better. As a rough practical guide:

- below about 0.5 px: usually very good
- about 0.5–1.0 px: often usable
- above about 1 px: inspect the dataset carefully

Do not judge quality from the number alone. A poorly distributed set of poses can still produce a deceptively low error.

## 8. Recommended directory layout

```text
camera_calibration/
├── generate.py
├── capture.py
├── calibrate.py
├── calibration_board/
│   ├── charuco_A4.pdf
│   ├── charuco_A4.png
│   └── board_config.json
├── calibration_images/
│   ├── calib_001.png
│   ├── calib_002.png
│   └── ...
├── detections/
│   └── ...
├── calibration.yaml
└── calibration.npz
```

## 9. Full example

Generate the board:

```bash
python generate.py --output-dir calibration_board
```

Print `calibration_board/charuco_A4.pdf` at 100% / Actual size.

Capture images:

```bash
python capture.py \
    --camera 0 \
    --width 1280 \
    --height 720 \
    --output-dir calibration_images
```

Calibrate:

```bash
python calibrate.py \
    --config calibration_board/board_config.json \
    --images "calibration_images/*.png" \
    --output calibration.yaml \
    --save-detections detections
```

## 10. Using the calibration in the robotics pipeline

Use the same calibrated intrinsics consistently for:

- visual or visual-inertial SLAM
- DepthART Metric
- back-projecting ZipDepth/DepthART depth into 3-D
- geometric map integration

For a depth value `Z` at pixel `(u, v)`:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth
```

If you resize an image from `(W, H)` to `(W', H')`, scale the intrinsics:

```text
sx = W' / W
sy = H' / H

fx' = sx * fx
fy' = sy * fy
cx' = sx * cx
cy' = sy * cy
```

If you crop the image, the principal point must also be shifted by the crop offset.

## 11. Fisheye cameras

These scripts assume a conventional pinhole camera with radial/tangential distortion.

If the camera uses a fisheye or ultra-wide-angle lens, use a fisheye calibration model instead of the default model in `calibrate.py`.
