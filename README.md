# INS-GPS

The file INS.pdf presents the main equations implemented for static IMU data.

The files in "calculations" contain symbolic jacobian calculations used in "EKF_GDA".
"EKF" contains code based on the navLab IIT lab work. "EKF_GDA" is a very simple implementation that uses the jacobians explained in "calculations and the pdf document.
"linerizedKF" is similar to "EKF", but with a linearized KF insted of an EKF.

The statis data of the IMU at different frequencies is contained in "DATA_STATIC".

