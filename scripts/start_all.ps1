# ============================================================================
# start_all.ps1 - Windows one-click entry point for the BabyROS arm stack.
# Runs start_all.sh inside the Kali WSL distro.
#
# Usage:  powershell -ExecutionPolicy Bypass -File scripts\start_all.ps1
# ============================================================================
$ErrorActionPreference = "Stop"
$script = "/mnt/c/Users/tosha/Downloads/Roboticarm/scripts/start_all.sh"
wsl -d kali-linux -- bash $script
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
