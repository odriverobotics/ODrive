param(
    [ValidateSet("build", "run", "image", "clean", "gc", "dev-up", "dev-shell", "dev-down", "help")]
    [string]$Action = "help",
    [string]$Image = "odrive-build-img",
    [string]$DevContainer = "odrive-dev-container"
)

$ErrorActionPreference = "Stop"
$RepoPath = (Resolve-Path $PSScriptRoot).Path
$MountArg = "type=bind,source=$RepoPath,target=/ODrive"

function Show-Usage {
    Write-Host "Usage: .\dockerbuild.ps1 <action> [-Image <image>] [-DevContainer <name>]"
    Write-Host ""
    Write-Host "Actions:"
    Write-Host "  build      clean + build image + run compile container"
    Write-Host "  run        run compile container (requires image)"
    Write-Host "  image      build image only"
    Write-Host "  clean      remove build artifacts and dev container"
    Write-Host "  gc         clean + remove image + prune dangling images"
    Write-Host "  dev-up     create/start a long-running dev container"
    Write-Host "  dev-shell  enter the long-running dev container"
    Write-Host "  dev-down   stop the long-running dev container"
    Write-Host "  help       show this help"
}

function Clean-Artifacts {
    Write-Host "Cleaning previous build artifacts..."
    $paths = @(
        (Join-Path $RepoPath "build"),
        (Join-Path $RepoPath "Firmware\autogen"),
        (Join-Path $RepoPath "Firmware\build"),
        (Join-Path $RepoPath "Firmware\.tup")
    )

    foreach ($path in $paths) {
        if (Test-Path $path) {
            Remove-Item -Recurse -Force $path -ErrorAction SilentlyContinue
        }
    }

    & docker rm -f $DevContainer 2>$null | Out-Null
}

function Build-Image {
    Write-Host "Building image: $Image"
    & docker build -t $Image $RepoPath
}

function Run-Compile {
    Write-Host "Running compile container..."
    & docker run --rm -it --mount $MountArg $Image
}

function Ensure-DevContainer {
    $exists = (& docker ps -a --format "{{.Names}}" | Where-Object { $_ -eq $DevContainer }).Count -gt 0
    if ($exists) {
        Write-Host "Starting existing dev container: $DevContainer"
        & docker start $DevContainer | Out-Null
    } else {
        Write-Host "Creating dev container: $DevContainer"
        & docker run -dit `
            --name $DevContainer `
            --mount $MountArg `
            -w /ODrive/Firmware `
            $Image bash -lc "sleep infinity" | Out-Null
    }
}

switch ($Action) {
    "build" {
        Clean-Artifacts
        Build-Image
        Run-Compile
    }
    "run" {
        Run-Compile
    }
    "image" {
        Build-Image
    }
    "clean" {
        Clean-Artifacts
    }
    "gc" {
        Clean-Artifacts
        Write-Host "Removing image: $Image"
        & docker rmi $Image 2>$null | Out-Null
        & docker image prune -f
    }
    "dev-up" {
        Ensure-DevContainer
    }
    "dev-shell" {
        & docker exec -it $DevContainer bash
    }
    "dev-down" {
        & docker stop $DevContainer 2>$null | Out-Null
    }
    default {
        Show-Usage
    }
}
