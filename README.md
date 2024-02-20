# Aimvatar-IMU-Bridge
A C++ reimplementation of 6 DOF Position and Orientation Tracking from a single IMU using Mahony AHRS(Attitude and Heading Reference System)  algorithm

### Project Structure

```sql
Aimvatar-IMU-Bridge/
|
|-- src/                           # Source files for C++ calculations
|   |-- main.cpp                   # Main C++ file
|   |-- IMUProcessing.cpp          # IMU data processing
|   |-- DataReader.cpp             # Data reading utilities
|   `-- AdditionalModules/         # Other C++ modules
|       |-- Module1.cpp
|       `-- Module2.cpp
|
|-- include/                       # Header files for C++ source files
|   |-- IMUProcessing.h
|   |-- DataReader.h
|   `-- AdditionalModules/
|       |-- Module1.h
|       `-- Module2.h
|
|-- lib/                           # External libraries and dependencies
|   |-- ximu_matlab_library/       # x-IMU MATLAB library converted to C++
|   `-- other_libs/
|
|-- data/                          # Data files (IMU data, results, etc.)
|   |-- raw/
|   `-- processed/
|
|-- scripts/                       # Python scripts for data visualization
|   |-- visualize.py
|   `-- utils.py
|
|-- matlab/                        # Original MATLAB scripts (for reference)
|   |-- SixDOFanimation.m
|   |-- Script.m
|   |-- quaternion_library/
|   |   |-- quaternProd.m
|   |   `-- ... (other files)
|   `-- ximu_matlab_library/
|       |-- ImportDirectory.m
|       `-- ... (other files)
|
|-- docs/                          # Documentation files
|   |-- README.md
|   `-- project_notes.md
|
|-- tests/                         # Unit tests for C++ code
|   |-- test_main.cpp
|   `-- test_modules/
|       |-- test_module1.cpp
|       `-- test_module2.cpp
|
|-- build/                         # Build configuration files
|   |-- CMakeLists.txt
|   `-- makefile
|
`-- .gitignore                     # Gitignore file

```
