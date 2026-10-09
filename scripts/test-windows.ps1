# Compile and run the ROS-independent controller tests using the Lab 2 compiler.
$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$Compiler = 'C:\msys64\ucrt64\bin\g++.exe'

if (-not (Test-Path -LiteralPath $Compiler))
{
    throw "MSYS2 UCRT64 compiler not found: $Compiler"
}

$TestOutput = Join-Path $ProjectRoot 'test-output'
New-Item -ItemType Directory -Path $TestOutput -Force | Out-Null
$Executable = Join-Path $TestOutput 'wall_follower_test.exe'
& $Compiler -std=c++17 -Wall -Wextra -Wpedantic -Werror -O2 `
    -I (Join-Path $ProjectRoot 'turtlebot3_gazebo\include') `
    (Join-Path $ProjectRoot 'turtlebot3_gazebo\src\wall_follower.cpp') `
    (Join-Path $ProjectRoot 'turtlebot3_gazebo\src\scan_reader.cpp') `
    (Join-Path $ProjectRoot 'tests\wall_follower_test.cpp') -o $Executable

if ($LASTEXITCODE -ne 0)
{
    throw 'Controller test compilation failed'
}

& $Executable

if ($LASTEXITCODE -ne 0)
{
    throw 'Controller tests failed'
}
