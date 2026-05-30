Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$imagesRoot = Join-Path $repoRoot "images"
$dataDir = Join-Path $repoRoot "data"
$outputPath = Join-Path $dataDir "local-product-images.json"

$allowedExtensions = @(".jpg", ".jpeg", ".png", ".webp")
$excludedRootFiles = @("no-image.png", "robot-mark.png")
$excludedFolders = @("projects")

New-Item -ItemType Directory -Path $dataDir -Force | Out-Null

$catalog = Get-ChildItem -Path $imagesRoot -Recurse -File |
  Where-Object {
    $allowedExtensions -contains $_.Extension.ToLowerInvariant()
  } |
  Where-Object {
    $relativePath = $_.FullName.Substring($repoRoot.Length + 1).Replace("\", "/")
    $segments = $relativePath -split "/"

    if($segments.Length -lt 3){
      return $excludedRootFiles -notcontains $_.Name
    }

    return $excludedFolders -notcontains $segments[1]
  } |
  Sort-Object FullName |
  ForEach-Object {
    $relativePath = $_.FullName.Substring($repoRoot.Length + 1).Replace("\", "/")
    $segments = $relativePath -split "/"

    [pscustomobject]@{
      path = $relativePath
      folder = if($segments.Length -gt 2){ $segments[1] } else { "" }
      name = [System.IO.Path]::GetFileNameWithoutExtension($_.Name)
      extension = $_.Extension.TrimStart(".").ToLowerInvariant()
    }
  }

$catalog |
  ConvertTo-Json -Depth 4 |
  Set-Content -Path $outputPath -Encoding UTF8

Write-Output "Generated $($catalog.Count) image entries at $outputPath"
