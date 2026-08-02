# Grab one frame from each arm camera and copy the JPEGs to .\capture
# Run from Windows PowerShell (docker must be reachable, e.g. docker-desktop).
$ErrorActionPreference = "Stop"
$CaptureDir = Join-Path $PSScriptRoot "..\capture"
New-Item -ItemType Directory -Force -Path $CaptureDir | Out-Null

docker exec arm_ros bash -c `
  "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && python3 /project/scripts/snap_cameras.py /tmp/snap"

docker cp "arm_ros:/tmp/snap/camera_table.jpg"  (Join-Path $CaptureDir "camera_table.jpg")
docker cp "arm_ros:/tmp/snap/camera_stick.jpg"  (Join-Path $CaptureDir "camera_stick.jpg")
docker cp "arm_ros:/tmp/snap/camera_ee.jpg"     (Join-Path $CaptureDir "camera_ee.jpg")

Get-ChildItem $CaptureDir | Select-Object Name, Length
Write-Host "frames saved to $CaptureDir"
