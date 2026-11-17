# Gento demo control from this PC (PowerShell).  Usage:  .\gento.ps1 <command>
#
#   status     robot state, nav tasks, running job, last log lines        (no motion)
#   selfscan   measure arms + tray with the lidar, store in the model      (no motion; nothing within ~1 m in front)
#   check T|D  what docking at T / D would do from the current pose       (no motion)
#   bench T|D  planning time of each docking step on one lidar snapshot    (no motion; limit 1.2 s)
#   latency [N]  timed report of the last N jobs (input -> plan -> command -> motion; start-up marked)
#   demo-dry [GAP]  all checks of the demo run, no motion
#   to-start [GAP]  DRIVE to the table T: T1 -> align -> straight in to GAP m (default 0.45)   (MOVES, asks y/n)
#   demo [GAP]      DRIVE: out of the table -> D1 -> straight towards the conveyor to GAP m (default 0.25),
#                   stops earlier if robot/arms/tray would touch something            (MOVES, asks y/n)
#   goto NAME [GAP] [--via none|random|W] [--jitter R]   DRIVE: dock at station NAME (T, D, ...);    (MOVES, asks y/n)
#                         --via: default random (waypoints / direct equally likely), none = direct, --jitter: random spot within R m of the safe point
#   tune NAME KEY VALUE   fine-tune: gap (m) | lat_offset (m, + = further left) | yaw_offset (deg, + = left)
#   teach NAME            record the current lateral position as station NAME (turns a little, asks y/n)
#   teach-pose NAME [BACK] [left|right]   NO MOTION: robot stands EXACTLY at the final pose of NAME (driven
#                         there by hand, arms in carry pose): records gap + heading + lateral position to the
#                         face, and a staging pose BACK m (default 0.5) straight behind it
#   teach-staging NAME [map]  NO MOTION: robot at a safe point near NAME (room to turn): record it as NAME's
#                         staging point; map = dock NAME by map position (turn there, translate straight in)
#   teach-ref NAME        NO MOTION: robot exactly at map station NAME: record a reference scan (~1 cm docking)
#   teach-waypoint NAME [clear]  NO MOTION: robot in an open area (room to turn): record route waypoint NAME
#   teach-waypoint NAME norandom|random   NO MOTION: keep NAME but leave it out of random routes / allow it again
#   waypoints             list waypoints and which ones a random route may use       (no motion)
#   accept NAME [clear]   NO MOTION: robot lined up in front of NAME: structure the model touches there was
#                         verified clear on site -> docking ignores exactly those points
#   restage NAME          NO MOTION: recompute the staging spot of a taught station from here (robot near it)
#   lineup NAME           DRIVE: like goto, but stop lined up in front of NAME before driving in  (asks y/n)
#   record-bg NAME        DRIVE: goto NAME that also records its background (then only NEW objects block) (asks y/n)
#   bg-clear NAME         NO MOTION: forget NAME's recorded background
#   undock                DRIVE: reverse straight out along the last docking approach              (asks y/n)
#   job        job status JSON (what the skye_task adapter reads)
#   task [args]    START THE FULL TASK on the Orin (arms + navigation, in tmux)       (MOVES, asks y/n)
#                  handover where it stands: Enter closes the grippers, 1 = held, go / 0 = not right (grippers open, put it right) -> D -> place -> T
#                  --handover: drive to T first and open there, then the same handover
#                  --finish-place 135: a place stopped at D with the arms down: release, retract, back to T
#                  args e.g. --cycles 10 (10 runs in a row, stops at a failure) | --down-mm 120 | --no-return
#                  --holding: tray still in the grippers after a stop: no opening, straight to D, place, back
#                  --random-route: also a random spot near the safe points (waypoints are random by default)
#   grip open|close [left|right]   only the grippers, arms do not move (not while a task runs: o / c there)
#   task-attach    re-attach to the running task (after a Wi-Fi drop); Ctrl-b d detaches, task keeps running
#   task-stop      Ctrl-C to the task (stops arms + chassis) and software-stop the chassis
#   task-check     no motion: task config + SDK version on the Orin
#   relocate   fix the localisation: match the lidar scan to the whole map, apply the pose  (no motion)
#   (goto / undock / to-start / demo show the job log until it ends; Ctrl+C only stops watching)
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
    'bench'    { if (-not $Arg) { $Arg = 'D' }; ssh chassis "$remote bench $Arg" }
    'latency'  { if (-not $Arg) { $Arg = '5' }; ssh chassis "$remote latency $Arg" }
    'demo-dry' { ssh chassis "$remote demo-dry $Arg" }
    'to-start' { Confirm-Motion "drive to the table (start point T), gap $(if ($Arg) {$Arg} else {'0.45'}) m"; ssh -t chassis "$remote to-start $Arg && $remote follow" }
    'demo'     { Confirm-Motion "table -> D1 -> straight towards the conveyor, gap $(if ($Arg) {$Arg} else {'0.25'}) m (stops earlier if blocked)"; ssh -t chassis "$remote demo $Arg && $remote follow" }
    'goto'     { Confirm-Motion "dock at station $Arg"; ssh -t chassis "$remote goto $Arg $($Rest -join ' ') && $remote follow" }
    'tune'     { ssh chassis "$remote tune $Arg $($Rest -join ' ')" }
    'teach-pose' { ssh chassis "$remote teach-pose $Arg $($Rest -join ' ')" }
    'teach-staging' { ssh chassis "$remote teach-staging $Arg $($Rest -join ' ')" }
    'teach-ref' { ssh chassis "$remote teach-ref $Arg" }
    'accept'   { ssh chassis "$remote accept $Arg $($Rest -join ' ')" }
    'teach-waypoint' { ssh chassis "$remote teach-waypoint $Arg $($Rest -join ' ')" }
    'waypoints' { ssh chassis "$remote waypoints" }
    'restage'  { ssh chassis "$remote restage $Arg $($Rest -join ' ')" }
    'bg-clear' { ssh chassis "$remote bg-clear $Arg" }
    'record-bg' { Confirm-Motion "dock at $Arg and record its background (belt in front EMPTY, nobody in front)"; ssh -t chassis "$remote record-bg $Arg && $remote follow" }
    'lineup'   { Confirm-Motion "line up in front of station $Arg (not driving in)"; ssh -t chassis "$remote lineup $Arg && $remote follow" }
    'undock'   { Confirm-Motion "reverse straight out of the last docking"; ssh -t chassis "$remote undock && $remote follow" }
    'teach'    { Confirm-Motion "teach station $Arg at the current lateral position (turns a little)"; ssh chassis "$remote teach $Arg" }
    'job'      { ssh chassis "$remote job-status" }
    'task'        { Confirm-Motion "FULL TASK on the Orin: arms to carry pose, handover here (Enter closes, 1 = go, 0 = open again), drive to D, place, back to T";
                    ssh -t orin "bash ~/gento_skye/chassis/client/orin_task.sh start $Arg $($Rest -join ' ')" }
    'grip'        { if ($Arg -notin 'open', 'close') { Write-Host "usage: .\gento.ps1 grip open|close [left|right]"; exit 1 }
                    ssh -t orin "bash ~/gento_skye/chassis/client/orin_task.sh grip $Arg $($Rest -join ' ')" }
    'task-attach' { ssh -t orin "bash ~/gento_skye/chassis/client/orin_task.sh attach" }
    'task-stop'   { ssh orin "bash ~/gento_skye/chassis/client/orin_task.sh stop" }
    'task-check'  { ssh orin "bash ~/gento_skye/chassis/client/orin_task.sh check" }
    'relocate' { ssh chassis "$remote relocate" }
    'stop'     { ssh chassis "$remote stop" }
    'log'      { ssh -t chassis "$remote log" }
    'web'      {
        # the page comes through an SSH tunnel via the Orin's Wi-Fi; it dies when the Wi-Fi drops -> rebuild it
        function Test-Web { try { return (Invoke-WebRequest -UseBasicParsing -TimeoutSec 3 http://127.0.0.1:8080/).StatusCode -eq 200 } catch { return $false } }
        if (-not (Test-Web)) {
            # a dead tunnel can still hold the local port: end it first
            Get-NetTCPConnection -LocalPort 8080 -State Listen -ErrorAction SilentlyContinue | ForEach-Object {
                $p = Get-Process -Id $_.OwningProcess -ErrorAction SilentlyContinue
                if ($p -and $p.ProcessName -eq 'ssh') { Stop-Process -Id $p.Id -Force }
            }
            Write-Host "  starting the tunnel to the chassis web page ..."
            Start-Process ssh -ArgumentList '-N -o ExitOnForwardFailure=yes -o ServerAliveInterval=10 -o ServerAliveCountMax=3 -o ConnectTimeout=10 chassis-web' -WindowStyle Hidden
            $ok = $false
            foreach ($i in 1..15) { Start-Sleep 1; if (Test-Web) { $ok = $true; break } }
            if (-not $ok) {
                Write-Host "  web page not reachable: is the PC on the robot Wi-Fi?  test with:  ssh orin echo ok" -ForegroundColor Red
                exit 1
            }
        }
        Start-Process 'chrome.exe' 'http://localhost:8080'
    }
    default    { Get-Content $PSCommandPath -TotalCount 48 | Select-Object -Skip 1 }
}
