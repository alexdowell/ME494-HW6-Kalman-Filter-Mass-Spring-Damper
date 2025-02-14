# ME494 HW6: Kalman Filter Mass Spring Damper  

## Description  
This repository contains the sixth homework assignment for **ME494**, focusing on system identification using a **Kalman Filter** to estimate the states of a **Mass-Spring-Damper (MSD) system**. The assignment involves deriving discrete-time state-space equations, implementing a **Kalman Filter (KF)** for state estimation, and comparing the results with numerical differentiation techniques. The repository includes MATLAB scripts, datasets, and a PDF containing problem descriptions.  

## Files Included  

### **Part 1: State-Space Discretization and Observer Design**  
- **File:** ME_494_HW6_Dowell.pdf  
- **Topics Covered:**  
  - Derivation of discrete-time state equations  
  - Observer matrix definition and system measurements  
  - State-space model formulation for **MSD system**  
  
### **Part 2: Kalman Filter Implementation for MSD System**  
- **File:** SID_HW6.m  
- **File:** msd_data_hw6.mat  
- **Topics Covered:**  
  - Kalman Filter design for estimating **position, velocity, and acceleration**  
  - State estimation using noisy measurements  
  - Tuning process using **Q and R matrices**  
  - Comparison of Kalman Filter and numerical derivative results  
  
### **Part 3: Numerical Differentiation Support Function**  
- **File:** deriv.m  
- **Topics Covered:**  
  - Supports **SID_HW6.m** by computing smoothed numerical differentiation  
  - Used for estimating velocity and acceleration from position data  
  - Provides alternative comparison for Kalman Filter results  
  
## Installation  
Ensure MATLAB is installed before running the scripts. No additional toolboxes are required.  

## Usage  

### **Running the Kalman Filter for MSD System**  
1. Open MATLAB.  
2. Load `msd_data_hw6.mat` into the workspace.  
3. Run the script:  
   ```SID_HW6```  
4. View estimated **position, velocity, and acceleration** plots.  
5. Compare the **numerical differentiation** results against **KF outputs**.  

## Example Output  

- **State-Space Model Representation**  
  - Derived discrete-time **state equations** for MSD system  
  - Observer matrix **H** formulation  

- **Kalman Filter Estimation Results**  
  - Position tracking: **Filtered vs. Measured** comparison  
  - Velocity tracking: **KF output vs. Numerical Differentiation**  
  - Acceleration tracking: **KF estimated vs. true acceleration**  

- **Residual and Error Analysis**  
  - Residual distribution plots for estimated states  
  - Evaluation of tracking performance over time  

## Contributions  
This repository is intended for academic research and educational use. Contributions and modifications are welcome.  

## License  
This project is open for research and educational purposes.  

---  
**Author:** Alexander Dowell  

