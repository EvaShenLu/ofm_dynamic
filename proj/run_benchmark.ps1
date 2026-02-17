param(
    [string]$Target = "dynamic_obstacle",      # xmake 目标名
    [string]$LogFile = ".\ofm_benchmark_log.txt",       # 完整日志路径
    [string]$StageSummaryFile = ".\ofm_stage_summary.txt",  # 阶段汇总 CSV 路径
    [switch]$Append,                          # 是否追加到已有日志
    [switch]$SkipStageSummary,                # 是否跳过阶段汇总生成
    [int]$Duration = 30                        # 运行时长（秒），0 表示不限制
)

# 遇到错误立即停止，便于排查
$ErrorActionPreference = "Stop"

# 切换到脚本所在目录（proj/），确保 xmake 能找到目标，且可执行文件以正确 cwd 加载 config/
Push-Location $PSScriptRoot
$originalConfigContent = $null
try {
    # 根据 -Append 参数决定 Tee-Object 是覆盖还是追加
    if ($Append) {
        $teeParams = @{ FilePath = $LogFile; Append = $true }
    } else {
        $teeParams = @{ FilePath = $LogFile }
    }
    $timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    "[$timestamp] benchmark_target=$Target" | Tee-Object @teeParams

    # 若指定 Duration > 0，临时修改配置使程序在指定秒数后正常退出
    $configPath = Join-Path $Target "config\$Target.json"
    if ($Duration -gt 0 -and (Test-Path $configPath)) {
        $originalConfigContent = Get-Content $configPath -Raw -Encoding utf8
        if ($originalConfigContent -match '"frame_rate"\s*:\s*(\d+)') {
            $frameRate = [int]$Matches[1]
        } else {
            $frameRate = 30
        }
        $totalFrame = [int]($Duration * $frameRate)
        $newContent = $originalConfigContent -replace '"total_frame"\s*:\s*-?\d+', "`"total_frame`": $totalFrame"
        Set-Content -Path $configPath -Value $newContent -Encoding utf8 -NoNewline
        "[$timestamp] duration=${Duration}s, total_frame=$totalFrame (frame_rate=$frameRate)" | Tee-Object -FilePath $LogFile -Append
    }

    # 高精度计时器，用于统计总耗时
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    # 保存每行输出，供后续解析阶段耗时
    $runLines = New-Object System.Collections.Generic.List[string]
    $runExitCode = 0
    $timedOut = $false
    $prevErrorActionPreference = $ErrorActionPreference
    try {
        # 原生程序 stderr 应作为日志行捕获，而非导致脚本终止
        $ErrorActionPreference = "Continue"
        # 使用独立进程执行并在 Duration 秒后强制停止（硬超时）。
        $stdoutPath = [System.IO.Path]::GetTempFileName()
        $stderrPath = [System.IO.Path]::GetTempFileName()
        try {
            $proc = Start-Process -FilePath "xmake" -ArgumentList @("run", $Target) -WorkingDirectory $PSScriptRoot `
                -NoNewWindow -PassThru -RedirectStandardOutput $stdoutPath -RedirectStandardError $stderrPath

            if ($Duration -gt 0) {
                $exited = $proc.WaitForExit($Duration * 1000)
                if (-not $exited) {
                    $timedOut = $true
                    "[$(Get-Date -Format "yyyy-MM-dd HH:mm:ss")] timeout reached (${Duration}s), terminating process..." | Tee-Object -FilePath $LogFile -Append
                    Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
                    $proc.WaitForExit()
                }
            } else {
                $proc.WaitForExit()
            }

            if ($timedOut) {
                # GNU timeout-compatible convention: 124 means timed out.
                $runExitCode = 124
            } else {
                $runExitCode = $proc.ExitCode
            }

            $output = @()
            if (Test-Path $stdoutPath) { $output += Get-Content -Path $stdoutPath -Encoding utf8 }
            if (Test-Path $stderrPath) { $output += Get-Content -Path $stderrPath -Encoding utf8 }

            foreach ($line in $output) {
                $lineStr = "$line"
                $runLines.Add($lineStr)
            }
            # 将运行输出追加到日志（头部已写入，此处始终 Append）
            $output | Tee-Object -FilePath $LogFile -Append
        }
        finally {
            if (Test-Path $stdoutPath) { Remove-Item -Path $stdoutPath -Force -ErrorAction SilentlyContinue }
            if (Test-Path $stderrPath) { Remove-Item -Path $stderrPath -Force -ErrorAction SilentlyContinue }
        }
    }
    finally {
        $ErrorActionPreference = $prevErrorActionPreference
        $sw.Stop()
    }

    # 记录结束时间、总耗时和退出码
    $endTimestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    "[$endTimestamp] elapsed_seconds=$($sw.Elapsed.TotalSeconds)" | Tee-Object -FilePath $LogFile -Append
    "[$endTimestamp] run_exit_code=$runExitCode" | Tee-Object -FilePath $LogFile -Append
    "[$endTimestamp] timed_out=$timedOut" | Tee-Object -FilePath $LogFile -Append

    # 阶段汇总：从输出中解析各计算阶段耗时并生成 CSV
    if (-not $SkipStageSummary) {
        # 仿真程序已知的阶段名称列表，仅解析这些阶段（忽略其他杂项输出）
        $knownStages = @(
            "UpdateBoundaryCondition",    # 更新边界条件
            "Rebuild Projection Matrix",  # 重建投影矩阵
            "Advection",                  # 平流
            "Projection 1",               # 第一次投影
            "Marching Backward flowmap",  # 反向流图行进
            "Marching Forward flowmap",   # 正向流图行进
            "Impulse reconstruction",    # 脉冲重建
            "BFECC",                      # 双向误差补偿对流
            "Projection 2"                # 第二次投影
        )

        # 正则：匹配 "StageName: X.XX ms" 或带前导空格的 "  StageName: X.XX ms"
        $stageRegex = '^\s*([A-Za-z][A-Za-z0-9 _-]+):\s*([0-9]+(?:\.[0-9]+)?)\s*ms\s*$'
        $entries = @()
        foreach ($line in $runLines) {
            $m = [regex]::Match($line, $stageRegex)
            if (-not $m.Success) { continue }
            $stageName = $m.Groups[1].Value.Trim()
            # 只保留已知阶段，过滤无关行
            if ($knownStages -notcontains $stageName) { continue }
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
            # CSV 表头：阶段名、出现次数、平均/最小/最大耗时(ms)
            $summaryLines.Add("stage,count,avg_ms,min_ms,max_ms")

            # 按阶段名分组，计算统计量
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

    # 若程序非零退出，将退出码传递给调用方
    if ($runExitCode -ne 0) {
        exit $runExitCode
    }
}
finally {
    # 恢复此前为 Duration 而临时修改的配置
    if ($null -ne $originalConfigContent -and (Test-Path $configPath)) {
        Set-Content -Path $configPath -Value $originalConfigContent -Encoding utf8 -NoNewline
    }
    # 恢复进入脚本前的当前目录
    Pop-Location
}
