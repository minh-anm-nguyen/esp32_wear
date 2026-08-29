# Activate ESP-IDF for this project.
#   . .\export-idf.ps1      (note the leading dot and space)
#
# Why this wrapper instead of calling IDF's export.ps1 directly:
# the ESP-IDF Installer Manager (EIM) uses a layout that does not match what
# idf_tools.py expects by default, so the three variables below must be set.
#
#   idf_tools.py looks for tools       in  $IDF_TOOLS_PATH\tools\<tool-name>\
#   idf_tools.py looks for the venv    in  $IDF_TOOLS_PATH\python_env\...   (EIM puts it elsewhere)
#   idf_tools.py looks for constraints in  $IDF_TOOLS_PATH\espidf.constraints.v6.1.txt

$env:IDF_PATH           = "D:\esp\v6.1-beta1\esp-idf"
$env:IDF_TOOLS_PATH     = "D:\Espressif"
$env:IDF_PYTHON_ENV_PATH = "D:\Espressif\tools\python\v6.1-beta1\venv"

# EIM ships the constraints file one directory too deep; copy it where
# idf_tools.py will actually look.
$constraints = Join-Path $env:IDF_TOOLS_PATH "espidf.constraints.v6.1.txt"
if (-not (Test-Path $constraints)) {
    $src = Join-Path $env:IDF_TOOLS_PATH "tools\espidf.constraints.v6.1.txt"
    if (Test-Path $src) {
        Copy-Item $src $constraints
        Write-Host "Copied constraints file to $constraints"
    } else {
        Write-Warning "espidf.constraints.v6.1.txt not found at $src"
    }
}

. (Join-Path $env:IDF_PATH "export.ps1")
