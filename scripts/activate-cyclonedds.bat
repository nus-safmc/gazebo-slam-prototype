@echo off
REM Pixi activation script to configure CycloneDDS for ROS 2
REM This prevents FastDDS hanging issues in RoboStack environments
REM See: https://github.com/RoboStack/ros-jazzy/issues/57

REM Save any existing RMW_IMPLEMENTATION value for restoration
if defined RMW_IMPLEMENTATION (
    set "_PIXI_BACKUP_RMW_IMPLEMENTATION=%RMW_IMPLEMENTATION%"
) else (
    set "_PIXI_BACKUP_RMW_IMPLEMENTATION="
)

REM Force CycloneDDS to avoid FastDDS hanging issues
set "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"

REM Find project root and set CycloneDDS config
for %%i in ("%~dp0..") do set "PROJECT_ROOT=%%~fi"
set "CYCLONEDDS_URI=file://%PROJECT_ROOT%\scripts\px4\cyclonedds.xml"

echo ROS 2 DDS configured: RMW_IMPLEMENTATION=%RMW_IMPLEMENTATION%