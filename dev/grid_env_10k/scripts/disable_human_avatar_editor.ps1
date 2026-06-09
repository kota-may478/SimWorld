# Disable Human_Avatar (Method A). Run after UE Editor is fully closed.
$src = "C:\UEProjects\SimWorld\Content\Human_Avatar"
$dst = "C:\UEProjects\SimWorld\Content\_disabled_Human_Avatar"
if (-not (Test-Path $src)) {
    Write-Host "OK: Human_Avatar already absent"
    exit 0
}
if (Test-Path $dst) {
    Write-Error "Destination exists: $dst — remove or rename it first"
    exit 1
}
Rename-Item -Path $src -NewName "_disabled_Human_Avatar"
Write-Host "OK: renamed Human_Avatar -> _disabled_Human_Avatar"
