# Setup: Windows host (run in PowerShell as normal user, parts need admin)
# For the BabyROS arm_6dof ROS 2 / WSL 2 / Gazebo stack.
#
# Run pieces individually:  powershell -ExecutionPolicy Bypass -File setup_windows.ps1
# The WSL2 + usbipd parts require an elevated shell.

$ErrorActionPreference = "Continue"

Write-Host "== 1/4 Checking WSL 2 =="
$wsl = wsl --status
Write-Host $wsl

Write-Host "`n== 2/4 Updating WSL (WSLg for Gazebo GUI) =="
wsl --update

Write-Host "`n== 3/4 Checking usbipd-win (gamepad passthrough) =="
$usbipd = Get-Command usbipd -ErrorAction SilentlyContinue
if (-not $usbipd) {
  Write-Host "usbipd not found. Install it with:"
  Write-Host "  winget install --interactive --exact dorssel.usbipd-win"
} else {
  Write-Host "usbipd found. Example passthrough for a gamepad:"
  Write-Host "  usbipd list"
  Write-Host "  usbipd bind --busid <BUSID>   (elevated)"
  Write-Host "  usbipd attach --wsl --busid <BUSID>"
}

Write-Host "`n== 4/4 WSL config (systemd + auto mount) =="
wsl -e sh -c 'grep -q "systemd" /etc/wsl.conf 2>/dev/null || echo "[boot]
systemd=true" | sudo tee -a /etc/wsl.conf || echo "edit /etc/wsl.conf manually: [boot] systemd=true"'

Write-Host "`nDone. Restart WSL afterwards:  wsl --shutdown"
