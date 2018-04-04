# INS-GPS

The file INS.pdf presents the main equations implemented for static IMU data.

Folders:

**** Folders for calibration and testing for static data ****
- "calculations" contain symbolic jacobian calculations used in "EKF_GDA".
- "EKF" contains code based on the navLab IIT lab work. 
- "EKF_GDA" is a very simple implementation that uses the jacobians explained in "calculations and the pdf document.
- "linerizedKF" is similar to "EKF", but with a linearized KF insted of an EKF.
- "calibration" contains the code used to obtain the biases and scale factor as explained in the calibration section of the INS.pdf file.

**** Folders containing code to be used with real moving data ****
- "DEV" contains the latest developments in the code for a real cart moving.


The static data of the IMU at different frequencies is contained in "DATA_STATIC".
The moving data of the IMU at different frequencies ia contained in "DATA_MOVE".

