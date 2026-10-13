param([switch]$Test)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$taskSdk = Join-Path $taskRoot 'sdk\ucrt64'
$env:PATH = "$taskSdk\bin;$env:PATH"
python "$PSScriptRoot\patch_vendor.py"
if ($LASTEXITCODE) { throw 'Vendor compatibility patch failed' }
& "$taskSdk\bin\cmake.exe" -S $taskRoot -B "$taskRoot\build" -G Ninja "-DCMAKE_PREFIX_PATH=$taskSdk" "-DCMAKE_C_COMPILER=$taskSdk/bin/gcc.exe" "-DCMAKE_CXX_COMPILER=$taskSdk/bin/g++.exe" -DCMAKE_BUILD_TYPE=Release *> "$taskRoot\configure.log"
if ($LASTEXITCODE) { Get-Content "$taskRoot\configure.log" -Tail 35; throw 'CMake configuration failed; see configure.log' }
Write-Output 'Configured. Compiling application and pinned libraries; details in build.log.'
& "$taskSdk\bin\cmake.exe" --build "$taskRoot\build" --parallel 4 *> "$taskRoot\build.log"
if ($LASTEXITCODE) { Get-Content "$taskRoot\build.log" -Tail 60; throw 'Build failed; see build.log' }
if ($Test) {
 & "$taskSdk\bin\ctest.exe" --test-dir "$taskRoot\build" --output-on-failure
 if ($LASTEXITCODE) { throw 'Tests failed' }
}
