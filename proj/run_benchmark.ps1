param(
    [string]$Target = "dynamic_obstacle",
    [string]$LogFile = ".\ofm_benchmark_log.txt",
    [string]$StageSummaryFile = ".\ofm_stage_summary.txt",
    [switch]$Append,
    [switch]$SkipStageSummary
)

$ErrorActionPreference = "Stop"

# Ensure we run from proj/ so xmake finds the target and exe gets correct cwd for config/
Push-Location $PSScriptRoot
try {
    if ($Append) {
        $teeParams = @{ FilePath = $LogFile; Append = $true }
    } else {
        $teeParams = @{ FilePath = $LogFile }
    }

    $timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    "[$timestamp] benchmark_target=$Target" | Tee-Object @teeParams

    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $runLines = New-Object System.Collections.Generic.List[string]
    $runExitCode = 0
    $prevErrorActionPreference = $ErrorActionPreference
    try {
        # Native app stderr should be captured as log lines, not terminate the script.
        $ErrorActionPreference = "Continue"
        # Run xmake first, capture output; $LASTEXITCODE is from xmake (not from pipeline).
        $output = xmake run $Target 2>&1
        $runExitCode = $LASTEXITCODE
        foreach ($line in $output) {
            $lineStr = if ($line -is [System.Management.Automation.ErrorRecord]) { $line.ToString() } else { "$line" }
            $runLines.Add($lineStr)
        }
        # Append xmake output to log (always append, since header was written above)
        $output | Tee-Object -FilePath $LogFile -Append
    }
    finally {
        $ErrorActionPreference = $prevErrorActionPreference
        $sw.Stop()
    }

    $endTimestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    "[$endTimestamp] elapsed_seconds=$($sw.Elapsed.TotalSeconds)" | Tee-Object -FilePath $LogFile -Append
    "[$endTimestamp] run_exit_code=$runExitCode" | Tee-Object -FilePath $LogFile -Append

    if (-not $SkipStageSummary) {
        $knownStages = @(
            "UpdateBoundaryCondition",
            "Rebuild Projection Matrix",
            "Advection",
            "Projection 1",
            "Marching Backward flowmap",
            "Marching Forward flowmap",
            "Impulse reconstruction",
            "BFECC",
            "Projection 2"
        )

        # Match "StageName: X.XX ms" or "  StageName: X.XX ms"
        $stageRegex = '^\s*([A-Za-z][A-Za-z0-9 _-]+):\s*([0-9]+(?:\.[0-9]+)?)\s*ms\s*$'
        $entries = @()
        foreach ($line in $runLines) {
            $m = [regex]::Match($line, $stageRegex)
            if (-not $m.Success) {
                continue
            }
            $stageName = $m.Groups[1].Value.Trim()
            if ($knownStages -notcontains $stageName) {
                continue
            }
            $entries += [pscustomobject]@{
                Name = $stageName
                Ms   = [double]$m.Groups[2].Value
            }
        }

        if ($entries.Count -gt 0) {
            $summaryLines = New-Object System.Collections.Generic.List[string]
            $summaryLines.Add("benchmark_target=$Target")
            $summaryLines.Add("timestamp=$endTimestamp")
            $summaryLines.Add("elapsed_seconds=$($sw.Elapsed.TotalSeconds)")
            $summaryLines.Add("")
            $summaryLines.Add("stage,count,avg_ms,min_ms,max_ms")

            $grouped = $entries | Group-Object -Property Name
            foreach ($g in $grouped) {
                $vals = $g.Group.Ms
                $avg = ($vals | Measure-Object -Average).Average
                $min = ($vals | Measure-Object -Minimum).Minimum
                $max = ($vals | Measure-Object -Maximum).Maximum
                $summaryLines.Add(("{0},{1},{2:N6},{3:N6},{4:N6}" -f $g.Name, $g.Count, $avg, $min, $max))
            }

            Set-Content -Path $StageSummaryFile -Value $summaryLines -Encoding utf8
            "wrote stage summary: $StageSummaryFile" | Tee-Object -FilePath $LogFile -Append
        }
        else {
            "no known stage timing lines found for target '$Target'" | Tee-Object -FilePath $LogFile -Append
        }
    }

    if ($runExitCode -ne 0) {
        exit $runExitCode
    }
}
finally {
    Pop-Location
}
