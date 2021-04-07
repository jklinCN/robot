# robot
Tool: 

- CoppeliaSim Edu 4.1.0
- Visual Studio 2019

Model: Universal Robots UR3

Tips:

1. You should open remoteApiSharedLib-64.vcxproj and generate remoteApi.dll and remoteApiSharedLib-64.lib. Then includes them in path:

   ```
   ...\CoppeliaRobotics\CoppeliaSimEdu\programming\remoteApiBindings\lib\x64\Release
   ```

2. Preprocessor definition

   ```
   DO_NOT_USE_SHARED_MEMORY
   NON_MATLAB_PARSING
   MAX_EXT_API_CONNECTIONS=255
   _CRT_SECURE_NO_WARNINGS
   ```
   
3. Additional Library

   - [Eigen](eigen.tuxfamily.org): a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.