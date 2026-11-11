# Gento demo control from this PC (PowerShell).  Usage:  .\gento.ps1 <command>
#
#   status     robot state, nav tasks, running job, last log lines        (no motion)
#   selfscan   measure arms + tray with the lidar, store in the model      (no motion; nothing within ~1 m in front)
#   check T|D  what docking at T / D would do from the current pose       (no motion)
#   demo-dry [GAP]  all checks of the demo run, no motion
#   to-start [GAP]  DRIVE to the table T: T1 -> align -> straight in to GAP m (default 0.45)   (MOVES, asks y/n)
#   demo [GAP]      DRIVE: out of the table -> D1 -> straight towards the conveyor to GAP m (default 0.25),
#                   stops earlier if robot/arms/tray would touch something            (MOVES, asks y/n)
#   goto NAME [GAP]       DRIVE: dock at station NAME (T, D, ...)                            (MOVES, asks y/n)
#   tune NAME KEY VALUE   fine-tune: gap (m) | lat_offset (m, + = further left) | yaw_offset (deg, + = left)
#   teach NAME            record the current lateral position as station NAME (turns a little, asks y/n)
#   job        job status JSON (what the skye_task adapter reads)
#   stop       software stop: kill job, cancel moves + tasks, soft manual
#   log        follow the job log (Ctrl+C to leave; the job keeps running)
#   web        open the chassis web page (starts the tunnel if needed)
#
# Before any motion: someone at the red e-stop, hard switch on AUTO, charging cable unplugged.
param([Parameter(Position = 0)][string]$Cmd = 'help', [Parameter(Position = 1)][string]$Arg = '',
      [Parameter(ValueFromRemainingArguments = $true)][string[]]$Rest = @())

$remote = if ($env:GENTO_DOCK_RUN) { $env:GENTO_DOCK_RUN } else { 'bash ~/gento_dock/run.sh' }   # chassis install dir

function Confirm-Motion([string]$what) {
    Write-Host ""
    Write-Host "  About to MOVE the robot: $what" -ForegroundColor Yellow
    Write-Host "  - someone at the e-stop?  - hard switch AUTO?  - charging cable unplugged?  - tray held at >= 1.2 m?" -ForegroundColor Yellow
    $a = Read-Host "  Type y to start"
    if ($a -ne 'y') { Write-Host "  cancelled"; exit 1 }
}

switch ($Cmd) {
    'status'   { ssh chassis "$remote status" }
    'selfscan' { ssh chassis "$remote selfscan" }
    'check'    { if (-not $Arg) { $Arg = 'D' }; ssh chassis "$remote check $Arg" }
    'demo-dry' { ssh chassis "$remote demo-dry $Arg" }
    'to-start' { Confirm-Motion "drive to the table (start point T), gap $(if ($Arg) {$Arg} else {'0.45'}) m"; ssh chassis "$remote to-start $Arg"; Write-Host "  follow with: .\gento.ps1 log    stop with: .\gento.ps1 stop" }
    'demo'     { Confirm-Motion "table -> D1 -> straight towards the conveyor, gap $(if ($Arg) {$Arg} else {'0.25'}) m (stops earlier if blocked)"; ssh chassis "$remote demo $Arg"; Write-Host "  follow with: .\gento.ps1 log    stop with: .\gento.ps1 stop" }
    'goto'     { Confirm-Motion "dock at station $Arg"; ssh chassis "$remote goto $Arg $($Rest -join ' ')" }
    'tune'     { ssh chassis "$remote tune $Arg $($Rest -join ' ')" }
    'teach'    { Confirm-Motion "teach station $Arg at the current lateral position (turns a little)"; ssh chassis "$remote teach $Arg" }
    'job'      { ssh chassis "$remote job-status" }
    'stop'     { ssh chassis "$remote stop" }
    'log'      { ssh -t chassis "$remote log" }
    'web'      {
        $ok = $false
        try { $ok = (Invoke-WebRequest -UseBasicParsing -TimeoutSec 3 http://localhost:8080/).StatusCode -eq 200 } catch {}
        if (-not $ok) { Start-Process ssh -ArgumentList '-o ExitOnForwardFailure=yes -o ServerAliveInterval=10 chassis-web' -WindowStyle Minimized; Start-Sleep 4 }
        Start-Process 'chrome.exe' 'http://localhost:8080'
    }
    default    { Get-Content $PSCommandPath -TotalCount 18 | Select-Object -Skip 1 }
}
