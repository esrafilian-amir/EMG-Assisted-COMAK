# number of COMAK analyses running: MATLAB batch runs plus Python example runs
$n = @(Get-CimInstance Win32_Process | Where-Object {
    ($_.Name -eq 'MATLAB.exe' -and $_.CommandLine -match '-batch') -or
    ($_.Name -eq 'python.exe' -and $_.CommandLine -match 'run_py\.py|example\w+\.py')
}).Count
Write-Output $n
