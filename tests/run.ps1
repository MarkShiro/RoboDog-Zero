$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    $zig = Join-Path $root '.tools/host/ziglang/zig.exe'
    if (-not (Test-Path -LiteralPath $zig)) {
        $command = Get-Command zig -ErrorAction SilentlyContinue
        if (-not $command) { throw 'Install Zig 0.14+ or place zig.exe in .tools/host/ziglang/' }
        $zig = $command.Source
    }
    New-Item -ItemType Directory -Path (Join-Path $root 'tmp') -Force | Out-Null
    foreach ($test in @('test_protocol','test_motor_map','test_remote_ui','test_mega_final','test_mega_motion')) {
        & $zig cc -x c++ -std=c++17 -nostdlib++ -Itests/stubs "tests/$test.cpp" -o "tmp/$test.exe"
        if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $test" }
        & "./tmp/$test.exe"
        if ($LASTEXITCODE -ne 0) { throw "Test failed: $test" }
    }
} finally { Pop-Location }
