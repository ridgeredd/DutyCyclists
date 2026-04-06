$exe = ".\direwolf_mod\build\src\direwolf.exe"
$env:GPS_FOLDER_PATH = "..\appv2\GPSData"

while ($true) {
    Write-Host "Starting direwolf..."
    & $exe
    Write-Host "App exited. Restarting in 2 seconds..."
    Start-Sleep -Seconds 2
}
