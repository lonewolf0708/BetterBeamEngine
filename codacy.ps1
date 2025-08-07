# Codacy CLI wrapper script using Docker
# Usage: .\codacy.ps1 analyze --help

# Get current directory and convert to Windows path with forward slashes for Docker
$currentDir = (Get-Item -Path ".").FullName -replace "\\\\", "/"

# Build the command as a single string
$command = "docker run --rm "
$command += "-v ${currentDir}:/src "
$command += "-v ${currentDir}/.codacyrc:/.codacyrc "
$command += "-v ${currentDir}/.codacy.json:/.codacy.json "

# Add Docker socket if it exists
if (Test-Path "//var/run/docker.sock") {
    $command += "-v //var/run/docker.sock:/var/run/docker.sock "
}

# Add the image and pass through all arguments
$command += "codacy/codacy-analysis-cli:latest $args"

# Execute the command
Invoke-Expression $command