setlocal
REM "A quick batch file with variable options to kick off a cmake build on Windows"

SET MAYA_VERSION=2019
SET BUILD=mayabuild_%MAYA_VERSION%
SET COMPILER=Visual Studio 15 2017 Win64

SET PFX=%~dp0
cd %PFX%
rmdir %BUILD% /s /q
mkdir %BUILD%
cd %BUILD%

cmake ^
    -DMAYA_VERSION=%MAYA_VERSION% ^
    -G "%COMPILER%" ..\

cmake --build . --config Release --target INSTALL

pause
