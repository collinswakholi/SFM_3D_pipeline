# SFM_3D_pipeline

Structure_From_Motion_3D_Imaging

SFM is used to estimate 3D structure from 2D image sequences taken at various angles of the structure.

This repository provides code for 2D image data collection, calibration and 3D reconstruction.
Specifically, these code were developed to capture data of beef carcasses and reconstruct an accurate and detailed 3D model of the carcass.
Modify the code to suit your case study (especially for camera and motor drive control).

Data collection
- Open folder called "Capture_Data".
- Run "Data_Capture_v1.m" to open up GUI for collecting Data. See attached video for further details
- For our case, we used mvBlueFox2089c camera and Ezi-Motion plusR motor.

Preprocessing
- Collect images of different orientations of a checkerboard in intended 3D space
- Run "Camera calibrator" app in MATLAB, load checkerboard images, calibrate, save calibration session mat file
- Open folder called "calib_mats".
- Run "convertCalib.m" to convert calibration session to distortion parameters and camera instrinsic matrix
- Copy the output mat file to "Reconstruction\calib" folder
 
Reconstruction
- Open folder called "Reconstruction".
- For reconstruction, run "main.m", preferably section by section. Be sure to adjust variables to suit your dataset
- Create suitiable background detection function if you need to
- Feature extraction process uses features from five diffenrt feature extraxtion methods or a combination of them
- Saved .ply file in the out folder can be visualised and/or further analysed using MeshLab, 3DFZephr, or any 3D preview/analysis program
- Sample of 3D reconstruction see video attached.
- Most of the reconstruction functions were inherited from "https://github.com/Alex-Badea/Pocket-3DR", visit link for further details.

Requirements
- MATLAB 2017 or Higher
- Image processing toolbox
- vlfeat toolbox (https://www.vlfeat.org/)
