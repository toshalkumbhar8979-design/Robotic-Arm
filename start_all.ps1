# =============================================================================
# start_all.ps1 - ONE file to start everything:
#   1. Docker container arm_ros (ROS 2 Jazzy + Gazebo Harmonic)
#   2. Gazebo stream (Xvfb + x11vnc + websockify) -> noVNC in the browser
#   3. Full ROS stack (arm, moveit, teleop, pick, cameras, rosbridge, dashboard)
#   4. Vision FastAPI dashboard on Windows (port 8050)
#   5. Opens browser: Digital Twin panel (embedded Gazebo) + full Gazebo window
# Run:  powershell -ExecutionPolicy Bypass -File start_all.ps1
# =============================================================================
$ErrorActionPreference = 'Stop'

$ROOT          = Split-Path -Parent $MyInvocation.MyCommand.Path
$COMPOSE_FILE  = Join-Path $ROOT 'scripts\compose-arm.yml'
$FRONTEND      = Join-Path $ROOT 'vision-autonomous-robotic-arm-main\vision-autonomous-robotic-arm-main\dashboard\frontend'
$BACKEND_DIR   = Join-Path $ROOT 'vision-autonomous-robotic-arm-main\vision-autonomous-robotic-arm-main\dashboard\backend'
$DASHBOARD_URL = 'http://localhost:8050/#panel-digital-twin'
$NOVNC_URL     = 'http://localhost:6080/vnc.html?autoconnect=1&resize=scale&reconnect=1'

function Say([string]$m) { Write-Host "==> $m" -ForegroundColor Cyan }

function Wait-Port([int]$port, [int]$seconds, [string]$name) {
    foreach ($i in 1..$seconds) {
        $c = New-Object System.Net.Sockets.TcpClient
        try {
            $iar = $c.BeginConnect('localhost', $port, $null, $null)
            if ($iar.AsyncWaitHandle.WaitOne(2000) -and $c.Connected) {
                $c.Close(); return $true
            }
        } catch { }
        $c.Close()
        Start-Sleep -Seconds 1
    }
    return $false
}

# ---------------------------------------------------------------- 1. Docker --
Say "Checking Docker engine..."
docker info *> $null
if ($LASTEXITCODE -ne 0) { Write-Error "Docker engine not running. Start Docker Desktop first." }
Say "Starting container arm_ros..."
docker compose -f $COMPOSE_FILE up -d
if ($LASTEXITCODE -ne 0) { Write-Error "Container failed to start." }

# ------------------------------------------------ 2. Deploy + start stack ----
Say "Deploying launch files and starting Gazebo stream + full stack (takes ~60s)..."
docker cp (Join-Path $ROOT 'src\arm_description\launch\arm_sim.launch.py') arm_ros:/root/ros2_ws/src/arm_description/launch/arm_sim.launch.py
docker cp (Join-Path $ROOT 'scripts\stream_gazebo_gui.sh') arm_ros:/root/stream_gazebo_gui.sh
docker exec arm_ros bash /root/restart.sh *> $null

# ------------------------------------------------------ 3. Wait for Gazebo ----
Say "Waiting for the Gazebo stream (noVNC)..."
if (-not (Wait-Port 6080 120 'noVNC')) { Write-Error "Gazebo stream did not come up on :6080" }
Say "Gazebo stream is UP: $NOVNC_URL"

# ---------------------------------------------- 4. Vision dashboard on 8050 ---
if (Wait-Port 8050 2 'dashboard') {
    Say "Dashboard already running on :8050 (reusing it)."
} else {
    Say "Starting FastAPI dashboard backend on :8050..."
    $log = Join-Path $env:TEMP 'opencode\dashboard_backend.log'
    $p = Start-Process python -ArgumentList 'main.py' -WorkingDirectory $BACKEND_DIR `
         -WindowStyle Hidden -RedirectStandardOutput $log -RedirectStandardError "$log.err" -PassThru
    Start-Sleep -Seconds 6
    if (-not (Wait-Port 8050 60 'dashboard')) {
        Write-Warning "Dashboard did not answer on :8050. Log: $log"
    } else {
        Say "Dashboard is UP: http://localhost:8050"
    }
}

# --------------------------------------------------------------- 5. Browser ---
Say "Opening the Digital Twin panel (embedded Gazebo GUI) and the full Gazebo window..."
Start-Process $DASHBOARD_URL
Start-Sleep -Milliseconds 1200
Start-Process $NOVNC_URL

Say ""
Say "ALL DONE. What's running:"
Say "  - Gazebo world (table, ram blocks, buckets, arm, 3 cameras) in the container"
Say "  - Full Gazebo window : $NOVNC_URL"
Say "  - Digital Twin panel : $DASHBOARD_URL (embedded Gazebo + joint sliders + gripper + pick + cameras + gamepad)"
Say "  - ROS dashboard      : http://localhost:8000"
Say "  - rosbridge WebSocket: ws://localhost:9090"
Say "  - Camera feeds       : http://localhost:8080/stream?topic=/camera_table/image_raw"
