# ============================================================================
# tools/cubemx_post_export.ps1
#
# 用途:CubeMX(重新)导出代码之后跑一遍,把"会被生成器覆盖"的几处改回来,
#       并对整个工程做一次自检。可反复运行(幂等)。
#
# 为什么需要它:
#   · Core/Inc/FreeRTOSConfig.h 每次导出都会被重新生成 ⇒ 堆大小、错误钩子会被打回默认;
#   · Middlewares/.../portable/ 会被换成 CubeMX 自己那套移植层(对 H7 是 GCC/ARM_CM4F)
#     ⇒ 如果 CMakeLists 里写的是别的路径(比如 CM7/r0p1),清理重建就会
#       'portmacro.h: No such file or directory' ——
#       更阴的是:**缺头文件不会触发重编**(GCC -MP 会给缺失头生成伪目标),
#       于是靠旧 .obj 还能编过,直到某天你 clean 一次才炸。
#
# 用法(在工程根目录):
#     powershell -ExecutionPolicy Bypass -File tools/cubemx_post_export.ps1
#     powershell -ExecutionPolicy Bypass -File tools/cubemx_post_export.ps1 -NoBuild
#     powershell -ExecutionPolicy Bypass -File tools/cubemx_post_export.ps1 -Power5VOff
#     powershell -ExecutionPolicy Bypass -File tools/cubemx_post_export.ps1 -OspiPrescaler 2
#
# 5V 策略:
#   默认 PC15(POWER_5V)上电为高 = 5V 打开(台架/外接 IMU 需要)。
#   需要恢复"上电关闭"时加 -Power5VOff。
#
# ★ 本脚本不修改 RCC / PLL / SystemClock_Config / PeriphCommonClock_Config,
#   重新生成后的 640MHz 时钟树保持 CubeMX 生成原样。
# ============================================================================

param(
    [switch]$NoBuild,
    [switch]$Power5VOff,
    [int]$OspiPrescaler = -1   # -1 = 保留 CubeMX 生成值;>=0 才强制覆盖
)

$power5vHigh = -not $Power5VOff

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$heapWanted = 32768
$script:issues = 0

function Info($m) { Write-Host "  $m" }
function Ok($m)   { Write-Host "  [OK]   $m" }
function Fix($m)  { Write-Host "  [修复] $m" -ForegroundColor Cyan }
function Warn($m) { Write-Host "  [注意] $m" -ForegroundColor Yellow; $script:issues++ }
function Bad($m)  { Write-Host "  [错误] $m" -ForegroundColor Red;    $script:issues++ }

Write-Host "`n=== 1. FreeRTOSConfig.h(堆 + 错误钩子) ==="
$cfg = 'Core/Inc/FreeRTOSConfig.h'
if (-not (Test-Path -LiteralPath $cfg)) { Bad "找不到 $cfg" }
else {
    $t = [System.IO.File]::ReadAllText($cfg)
    $orig = $t

    # ① 堆大小(整体替换那一行,不管原值多少)
    $t = [regex]::Replace($t, '#define\s+configTOTAL_HEAP_SIZE\s+\(\(size_t\)\s*\d+\s*\)',
                              "#define configTOTAL_HEAP_SIZE                    ((size_t)$heapWanted)")

    # ② 内存分配失败钩子(没有就插在 configUSE_TICK_HOOK 之后)
    if ($t -notmatch 'configUSE_MALLOC_FAILED_HOOK') {
        $t = [regex]::Replace($t, "(#define\s+configUSE_TICK_HOOK[^\r\n]*\r?\n)",
                                  "`$1#define configUSE_MALLOC_FAILED_HOOK             1`r`n")
    }
    # ③ 栈溢出检测(没有就插在 configCPU_CLOCK_HZ 之前)
    if ($t -notmatch 'configCHECK_FOR_STACK_OVERFLOW') {
        $t = [regex]::Replace($t, "(#define\s+configCPU_CLOCK_HZ[^\r\n]*\r?\n)",
                                  "#define configCHECK_FOR_STACK_OVERFLOW           2`r`n`$1")
    }

    if ($t -ne $orig) { [System.IO.File]::WriteAllText($cfg, $t); Fix "$cfg 已补回(堆=$heapWanted,两个错误钩子)" }
    else { Ok "$cfg 已经是对的" }

    $now = [System.IO.File]::ReadAllText($cfg)
    if ($now -match "#define\s+configTOTAL_HEAP_SIZE\s+\(\(size_t\)\s*(\d+)") {
        if ([int]$Matches[1] -eq $heapWanted) { Ok "configTOTAL_HEAP_SIZE = $heapWanted" }
        else { Bad "configTOTAL_HEAP_SIZE = $($Matches[1])(应为 $heapWanted)" }
    } else { Bad 'configTOTAL_HEAP_SIZE 没找到' }
    if ($now -match 'configUSE_MALLOC_FAILED_HOOK\s+1')   { Ok 'configUSE_MALLOC_FAILED_HOOK = 1' }   else { Bad 'configUSE_MALLOC_FAILED_HOOK 没打开' }
    if ($now -match 'configCHECK_FOR_STACK_OVERFLOW\s+2') { Ok 'configCHECK_FOR_STACK_OVERFLOW = 2' } else { Bad 'configCHECK_FOR_STACK_OVERFLOW 没打开' }
}

    # configASSERT:CubeMX 默认就是"关中断死循环",正合我们要求;这里只做确认
    # (两个坑都踩过:① 不能用整行正则,for( ;; ) 的空格写法会变;② 判定要读文件内容,
    #   $cfg 是路径字符串、内容在 $now 里,且这段在块外,所以这里重新读一次最稳)
    $cfgText = [System.IO.File]::ReadAllText($cfg)
    if ($cfgText -match 'configASSERT\([^\)]*\)[^\r\n]*taskDISABLE_INTERRUPTS') { Ok 'configASSERT = 关中断死循环(assert 失败会停住,便于定位)' }
    else { Warn 'configASSERT 不是"关中断死循环",assert 失败会被静默忽略,请确认' }

    # ★ CLion RTOS 视图(FreeRTOS Objects 的 runtime 列)—— 这 4 行必须待在 USER CODE 里
    #   (CubeMX 生成不出 portCONFIGURE_TIMER_FOR_RUN_TIME_STATS / portGET_RUN_TIME_COUNTER_VALUE ⇒ 只能手写)
    if ($now -match 'configGENERATE_RUN_TIME_STATS\s+1') {
        Ok 'configGENERATE_RUN_TIME_STATS = 1(USER CODE Defines 里,用 TIM5;CLion runtime 列可用)'
    } else {
        Warn 'FreeRTOSConfig.h 的 USER CODE BEGIN/END Defines 之间缺 configGENERATE_RUN_TIME_STATS 那 4 行(CLion runtime 列会没数据)'
    }
Write-Host "`n=== 2. FreeRTOS 移植层:确保是 M7 专用 ARM_CM7/r0p1,且树里只有一个 port.c ==="
$canonicalRel = 'tools/port/ARM_CM7/r0p1'                                     # 正本(上游 V10.6.2,已按 blob SHA 校验字节)
$targetRel    = 'Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1'
$targetAbs    = Join-Path $root $targetRel
$stashDir     = Join-Path $root 'tools/port/_stash_ARM_CM4F'
$gccDir       = Join-Path $root 'Middlewares/Third_Party/FreeRTOS/Source/portable/GCC'

# ① 正本必须在
if (-not (Test-Path -LiteralPath (Join-Path $root "$canonicalRel/portmacro.h"))) {
    Bad "$canonicalRel 里没有移植层正本(port.c / portmacro.h)"
}
else {
    # ② CubeMX 导出会把整个 portable/ 换成 CM4F 并删掉 CM7 ⇒ 从正本重装
    if (-not (Test-Path -LiteralPath (Join-Path $targetAbs 'portmacro.h'))) {
        New-Item -ItemType Directory -Force $targetAbs | Out-Null
        Copy-Item (Join-Path $root "$canonicalRel/*") $targetAbs -Force
        Fix "$targetRel 已从正本重装(CubeMX 导出会把它删掉)"
    }
    else { Ok "$targetRel 在位" }

    foreach ($n in @('port.c', 'portmacro.h')) {
        $a = Join-Path $root "$canonicalRel/$n"
        $b = Join-Path $targetAbs $n
        if ((Test-Path -LiteralPath $a) -and (Test-Path -LiteralPath $b) -and
            ((Get-FileHash -LiteralPath $a).Hash -eq (Get-FileHash -LiteralPath $b).Hash)) { Ok "$n 与正本字节一致" }
        else { Bad "$n 与正本不一致(删掉 $targetAbs 再跑一次本脚本)" }
    }
}

# ③ 把 CM7 以外的移植层移出编译树 —— file(GLOB) 会把它们都收进去,两份 port.c 必然符号重复
if (Test-Path -LiteralPath $gccDir) {
    Get-ChildItem -LiteralPath $gccDir -Directory -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -ne 'ARM_CM7' } | ForEach-Object {
            $dest = Join-Path $stashDir $_.Name
            New-Item -ItemType Directory -Force (Split-Path -Parent $dest) | Out-Null
            if (Test-Path -LiteralPath $dest) { Remove-Item -LiteralPath $dest -Recurse -Force }
            Move-Item -LiteralPath $_.FullName -Destination $dest -Force
            Fix "已把 $($_.Name) 移出编译树(备份在 tools/port/_stash_ARM_CM4F\)—— 两份 port.c 会符号重复"
        }
}

# ④ 两个 CMake 的两处路径都指向 CM7
foreach ($f in @('CMakeLists.txt', 'CMakeLists_template.txt')) {
    if (-not (Test-Path -LiteralPath $f)) { Warn "$f 不存在"; continue }
    $c = [System.IO.File]::ReadAllText($f)
    $c2 = [regex]::Replace($c, 'Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/(ARM_CM4F|ARM_CM7/r0p1)', $targetRel)
    if ($c2 -ne $c) { [System.IO.File]::WriteAllText($f, $c2); Fix "$f 的移植层路径已改为 $targetRel" }
    else {
        if ($c -match [regex]::Escape($targetRel)) { Ok "$f 已指向 $targetRel" }
        else { Bad "$f 里没找到 $targetRel" }
    }
}

# ⑤ 清掉构建目录里的陈旧 port 对象 —— 源码已被 CubeMX 换掉、对象却还留着。
#    这正是本工程踩过的坑:旧 ARM_CM7 对象被链进了新内核(缺头文件不触发重编),
#    排查"运行跳错误"花了大半天。宁可多删一次,让编译器重新编。
foreach ($bd in @('cmake-build-debug-mingw', 'cmake-build-release-mingw', 'cmake-build-verify')) {
    if (-not (Test-Path -LiteralPath $bd)) { continue }
    Get-ChildItem -LiteralPath $bd -Recurse -Directory -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -like 'ARM_CM*' -and $_.Name -ne 'ARM_CM7' } | ForEach-Object {
            Remove-Item -LiteralPath $_.FullName -Recurse -Force
            Fix "已清理陈旧移植层对象($($_.Name)):源码已不存在,留着只会误导"
        }
}

Write-Host "`n=== 2.5 OCTOSPI2(默认保留 CubeMX 生成的 ClockPrescaler)==="
# ★ ClockPrescaler 和系统时钟绑定:640MHz 时 CubeMX 可能给 3,480MHz 时给 2。
#   默认不要盲改;只有显式传 -OspiPrescaler N 才覆盖。
if (Test-Path -LiteralPath 'Core/Src/octospi.c') {
    $o = [System.IO.File]::ReadAllText('Core/Src/octospi.c')
    $o2 = $o
    if ($OspiPrescaler -ge 0) {
        $o2 = [regex]::Replace($o2, 'hospi2\.Init\.ClockPrescaler\s*=\s*\d+;', "hospi2.Init.ClockPrescaler = $OspiPrescaler;")
    }
    $o2 = [regex]::Replace($o2, 'hospi2\.Init\.SampleShifting\s*=\s*[A-Z_]+;', 'hospi2.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_HALFCYCLE;')
    if ($o2 -ne $o) { [System.IO.File]::WriteAllText('Core/Src/octospi.c', $o2); Fix 'octospi.c:SampleShifting/FifoThreshold 已补回(ClockPrescaler 按参数保留/覆盖)' }
    else {
        if ($o -match 'hospi2\.Init\.ClockPrescaler\s*=\s*(\d+);') {
            Ok "ClockPrescaler = $($Matches[1])(保留 CubeMX 生成值)"
        } else { Warn 'octospi.c 里没找到 ClockPrescaler' }
        if ($o -match 'hospi2\.Init\.SampleShifting\s*=\s*HAL_OSPI_SAMPLE_SHIFTING_HALFCYCLE;') { Ok 'SampleShifting = HALFCYCLE' } else { Bad 'SampleShifting 不是 HALFCYCLE' }
        if ($o -match 'hospi2\.Init\.FifoThreshold\s*=\s*(\d+);') {
            Ok "FifoThreshold = $($Matches[1])(保留 CubeMX 生成值;Flash 读写异常再试 8)"
        } else { Warn 'octospi.c 里没找到 FifoThreshold' }
    }
    # ★ 这句在 Core/Src/stm32h7xx_it.c 里(不是 octospi.c!)—— 自动轮询/中断模式必需
    $itfile = 'Core/Src/stm32h7xx_it.c'
    if (Test-Path -LiteralPath $itfile) {
        if ((Get-Content -LiteralPath $itfile -Raw) -match 'HAL_OSPI_IRQHandler\(&hospi2\)') { Ok 'it.c 里有 HAL_OSPI_IRQHandler(&hospi2)(自动轮询/中断必需)' }
        else { Bad 'it.c 里缺少 HAL_OSPI_IRQHandler(&hospi2) —— 自动轮询会收不到中断' }
    }
}
Write-Host "`n=== 3. 关键 USER CODE 段(导出应保留,这里只做确认) ==="
$checks = @(
    @{f='Core/Src/main.c';         p='Robot_Init\(\)';                        d='main.c 调 Robot_Init()'},
    @{f='Core/Src/main.c';         p='volatile uint32_t g_fault_code';        d='main.c 定义 g_fault_code'},
    @{f='Core/Inc/main.h';         p='extern volatile uint32_t g_fault_code'; d='main.h 声明 g_fault_code'},
    @{f='Core/Src/freertos.c';     p='OSTaskInit\(\)';                        d='freertos.c 调 OSTaskInit()'},
    @{f='Core/Src/freertos.c';     p='vApplicationMallocFailedHook';          d='freertos.c 内存失败钩子'},
    @{f='Core/Src/freertos.c';     p='vApplicationStackOverflowHook';         d='freertos.c 栈溢出钩子'},
    @{f='Core/Src/stm32h7xx_it.c'; p='g_fault_cfsr';                          d='it.c HardFault 现场记录'}
)
foreach ($c in $checks) {
    if ((Test-Path -LiteralPath $c.f) -and (Select-String -LiteralPath $c.f -Pattern $c.p -Quiet)) { Ok $c.d }
    else { Bad "$($c.d) —— 丢了,需手工补回(见 git 历史/说明文档)" }
}

Write-Host "`n=== 3.5 FreeRTOS 钩子:必须是"有且只有一份定义",且带诊断 ==="
if (Test-Path -LiteralPath 'Core/Src/freertos.c') {
    $frt = [System.IO.File]::ReadAllText('Core/Src/freertos.c')
    foreach ($h in @('vApplicationMallocFailedHook', 'vApplicationStackOverflowHook')) {
        $defs = ([regex]::Matches($frt, "void\s+$h\s*\([^;]*?\)\s*\{")).Count
        if ($defs -eq 1) { Ok "$h 只有一份定义" }
        elseif ($defs -eq 0) { Warn "$h 没有实现(FreeRTOSConfig 里开了钩子就必须有)" }
        else { Bad "$h 有 $defs 份定义 —— 会重复定义编译不过!(CubeMX 的空壳在 USER CODE 4/5,诊断写那里,别另写)" }
    }
    if ($frt -match 'g_fault_code\s*=\s*1u') { Ok 'malloc 失败钩子记录了 g_fault_code=1' } else { Warn 'malloc 失败钩子没记录 g_fault_code(=1)' }
    if ($frt -match 'g_fault_code\s*=\s*2u') { Ok '栈溢出钩子记录了 g_fault_code=2' }    else { Warn '栈溢出钩子没记录 g_fault_code(=2)' }
}
$wantState = if ($power5vHigh) { 'GPIO_PIN_SET' } else { 'GPIO_PIN_RESET' }
$wantText  = if ($power5vHigh) { 'SET(5V 打开)' } else { 'RESET(5V 关闭)' }
Write-Host "`n=== 4. PC15(板上 5V 输出使能,目标:$wantText)==="
# 说明:这个脚在 CubeMX 里叫 PC15-OSC32_OUT,标签为 POWER_5V。
#   · 有标签时:生成 POWER_5V_Pin;
#   · 标签被导出丢掉时(出现过):生成裸引脚 GPIO_PIN_15。
#   两种写法都要认,否则会误报。
if (Test-Path -LiteralPath 'Core/Src/gpio.c') {
    $g = [System.IO.File]::ReadAllText('Core/Src/gpio.c')
    $reset = ($g -match 'HAL_GPIO_WritePin\([^;]*POWER_5V_Pin[^;]*GPIO_PIN_RESET') -or
             ($g -match 'HAL_GPIO_WritePin\(\s*GPIOC\s*,\s*GPIO_PIN_15\s*,\s*GPIO_PIN_RESET')
    $set   = ($g -match 'HAL_GPIO_WritePin\([^;]*POWER_5V_Pin[^;]*GPIO_PIN_SET') -or
             ($g -match 'HAL_GPIO_WritePin\(\s*GPIOC\s*,\s*GPIO_PIN_15\s*,\s*GPIO_PIN_SET')

    $already = if ($power5vHigh) { $set } else { $reset }
    if ($already) {
        Ok "PC15 已经是 $wantText"
    } else {
        $oldState = if ($power5vHigh) { 'GPIO_PIN_RESET' } else { 'GPIO_PIN_SET' }
        $newText = $g -replace "HAL_GPIO_WritePin\(([^;]*POWER_5V_Pin|[^;]*GPIO_PIN_15[^;]*),\s*$oldState\)", "HAL_GPIO_WritePin(`$1, $wantState)"
        if ($newText -ne $g) {
            [System.IO.File]::WriteAllText('Core/Src/gpio.c', $newText)
            Fix "gpio.c: PC15 初始值改为 $wantText"
        } else {
            Warn "PC15 的初始化语句没找到,手工确认它必须是 GPIO_Output 且初值为 $wantText"
        }
    }
}
# 同时把"意图"写回 .ioc:标签/PinState/GPIOParameters 都补成目标值,下次导出也一致。
if (Test-Path -LiteralPath 'TSRobotSystem.ioc') {
    $ioc = [System.IO.File]::ReadAllText('TSRobotSystem.ioc')
    $ioc2 = $ioc
    if ($ioc2 -match '(?m)^PC15-OSC32_OUT\.PinState=') {
        $ioc2 = $ioc2 -replace '(?m)^PC15-OSC32_OUT\.PinState=.*$', "PC15-OSC32_OUT.PinState=$wantState"
    } else {
        $ioc2 = $ioc2 -replace '(?m)^(PC15-OSC32_OUT\.Signal=GPIO_Output)', "`$1`r`nPC15-OSC32_OUT.PinState=$wantState"
    }
    if ($ioc2 -notmatch '(?m)^PC15-OSC32_OUT\.GPIO_Label=') {
        $ioc2 = $ioc2 -replace '(?m)^(PC15-OSC32_OUT\.Signal=GPIO_Output)', "`$1`r`nPC15-OSC32_OUT.GPIO_Label=POWER_5V"
    }
    if ($ioc2 -notmatch '(?m)^PC15-OSC32_OUT\.GPIOParameters=') {
        $ioc2 = $ioc2 -replace '(?m)^(PC15-OSC32_OUT\.GPIO_Label=POWER_5V)', "PC15-OSC32_OUT.GPIOParameters=PinState,GPIO_Label`r`n`$1"
    }
    if ($ioc2 -ne $ioc) {
        [System.IO.File]::WriteAllText('TSRobotSystem.ioc', $ioc2)
        Fix ".ioc 里 PC15 已写成 PinState=$wantState / 标签 POWER_5V"
    }
}
# ============================================================================
# 2.6 链接脚本:补回 .RAM_D3 段(BDMA 专用缓冲)
#   SPI6 的 DMA 请求在 H7 上只能走 BDMA,而 BDMA 只能访问 D3 SRAM(0x38000000)。
#   CubeMX 每次导出都会重写 .ld ⇒ 这里把它补回去(幂等),否则 ws2812_buffer 会被放回
#   RAM_D1、BDMA 读不到 ⇒ 灯不亮(表现为"不亮但不死机")。
# ============================================================================
Write-Host "`n=== 2.6 链接脚本 .RAM_D3 段(BDMA 缓冲)===" -ForegroundColor Cyan
$ldFixed = 0
foreach ($ldPath in @('STM32H723VGTX_FLASH.ld', 'STM32H723VGTX_RAM.ld')) {
    if (-not (Test-Path -LiteralPath $ldPath)) { continue }
    $text = [System.IO.File]::ReadAllText($ldPath)
    $ldNl = if ($text -match "`r`n") { "`r`n" } else { "`n" }      # ★ 保留原换行(.ld 是 LF,不能改成 CRLF)
    $ln = New-Object System.Collections.Generic.List[string]
    $ln.AddRange([string[]]($text -split "`r?`n"))
    $chg = $false

    # ① 内存区域:缺 RAM_D3 (xrw) 就补在 RAM_D2 之后
    $hasRegion = $false
    foreach ($x in $ln) { if ($x -match 'RAM_D3\s*\(xrw\)') { $hasRegion = $true; break } }
    if (-not $hasRegion) {
        for ($i = 0; $i -lt $ln.Count; $i++) {
            if ($ln[$i] -match 'RAM_D2\s*\(xrw\)') {
                $ln.Insert($i + 1, '    RAM_D3  (xrw)    : ORIGIN = 0x38000000,   LENGTH = 16K')
                $chg = $true; break
            }
        }
    }

    # ② 输出段:只保留一份 .RAM_D3 (NOLOAD) 块(多了删,少了补)
    $starts = @()
    for ($i = 0; $i -lt $ln.Count; $i++) { if ($ln[$i] -match '^\s*\.RAM_D3 \(NOLOAD\)') { $starts += $i } }
    if ($starts.Count -gt 1) {
        for ($k = $starts.Count - 1; $k -ge 1; $k--) {
            $s = $starts[$k]; $e = $s
            while (($e -lt $ln.Count) -and ($ln[$e] -notmatch '\}\s*>RAM_D3')) { $e++ }
            if ($e -lt $ln.Count) { $ln.RemoveRange($s, $e - $s + 1) }
        }
        $chg = $true
    } elseif ($starts.Count -eq 0) {
        $block = @('.RAM_D3 (NOLOAD) :', '  {', '    . = ALIGN(32);', '    *(.RAM_D3)', '    *(.RAM_D3*)', '    . = ALIGN(32);', '  } >RAM_D3', '')
        $at = -1
        for ($i = 0; $i -lt $ln.Count; $i++) { if ($ln[$i] -match '^\s*\.AXI_SRAM \(NOLOAD\)') { $at = $i; break } }
        if ($at -lt 0) { for ($i = 0; $i -lt $ln.Count; $i++) { if ($ln[$i] -match '\}\s*>RAM_D2') { $at = $i + 1; break } } }
        if ($at -ge 0) { $ln.InsertRange($at, [string[]]$block); $chg = $true }
    }

    if ($chg) { [System.IO.File]::WriteAllText($ldPath, ($ln -join $ldNl)); Fix "$ldPath 的 .RAM_D3 段已修正(补回/去重)"; $ldFixed++ }
    else { Ok "$ldPath 里有且只有一份 .RAM_D3 段(BDMA 缓冲:SPI6/WS2812 需要)" }
}
if ($ldFixed -gt 0) { Info "链接脚本已自动修正:下次不用再手工改了 ✓" }

Write-Host "`n=== 5. User 段(自己加的目录/源码) ==="
# ★ 期望的 include 列表:直接按**文件系统**算(含头文件的目录),不写死清单 —— 新加模块不用改脚本 ✓
$userDirs = @('User/Bsp', 'User/Device', 'User/App')
$userRoot = (Resolve-Path 'User').Path
foreach ($d in (Get-ChildItem 'User' -Recurse -Directory)) {
    if ((Get-ChildItem $d.FullName -File -ErrorAction SilentlyContinue | Where-Object { $_.Extension -in '.h','.hpp' }).Count -gt 0) {
        $rel = $d.FullName.Replace($userRoot + '\', '').Replace('\', '/')
        $userDirs += ('User/' + $rel)
    }
}
$userDirs = $userDirs | Select-Object -Unique

# ★ 顺手清掉"目录已经不存在"的 User/... 行(改过目录名之后不留垃圾)
foreach ($cmFile in @('CMakeLists.txt', 'CMakeLists_template.txt')) {
    if (-not (Test-Path -LiteralPath $cmFile)) { continue }
    $text2 = [System.IO.File]::ReadAllText($cmFile)
    $nl2 = if ($text2 -match "`r`n") { "`r`n" } else { "`n" }
    $cl2 = New-Object System.Collections.Generic.List[string]
    $cl2.AddRange([string[]]($text2 -split "`r?`n"))
    $n2 = 0
    for ($i = $cl2.Count - 1; $i -ge 0; $i--) {
        $tt = $cl2[$i].Trim()
        if (($tt -like 'User/*') -and ($tt -notlike '*/*.*') -and (-not (Test-Path -LiteralPath $tt))) { $cl2.RemoveAt($i); $n2++ }
    }
    if ($n2 -gt 0) { [System.IO.File]::WriteAllText($cmFile, ($cl2 -join $nl2)); Fix "$cmFile 清掉 $n2 行已不存在的 User 目录" }
}

foreach ($cmFile in @('CMakeLists.txt', 'CMakeLists_template.txt')) {
    if (-not (Test-Path -LiteralPath $cmFile)) { continue }
    $text = [System.IO.File]::ReadAllText($cmFile)
    $cmNl = if ($text -match "`r`n") { "`r`n" } else { "`n" }
    $cl = New-Object System.Collections.Generic.List[string]
    $cl.AddRange([string[]]($text -split "`r?`n"))
    $chg = $false
    # ① 去重(同一个目录只留第一次出现)
    $seen = @{}
    for ($i = $cl.Count - 1; $i -ge 0; $i--) {
        $t = $cl[$i].Trim()
        if ($userDirs -contains $t) {
            if ($seen.ContainsKey($t)) { $cl.RemoveAt($i); $chg = $true } else { $seen[$t] = $true }
        }
    }
    # ② 补缺(插在已知成员 User/App/motor_task 之后,缩进照抄那一行)
    $anchor = -1
    for ($i = 0; $i -lt $cl.Count; $i++) { if ($cl[$i].Trim() -eq 'User/App/motor_task') { $anchor = $i; break } }
    if ($anchor -ge 0) {
        $indent = ($cl[$anchor] -replace '\S.*$', '')
        $add = @()
        foreach ($d in $userDirs) { if ((-not $seen.ContainsKey($d)) -and (Test-Path -LiteralPath $d)) { $add += ($indent + $d) } }
        if ($add.Count -gt 0) { $cl.InsertRange($anchor + 1, [string[]]$add); $chg = $true }
    }
    if ($chg) { [System.IO.File]::WriteAllText($cmFile, ($cl -join $cmNl)); Fix "$cmFile 的 User include 列表已修正(补缺/去重,按文件系统算)" }
}
foreach ($d in $userDirs) {
    $inCm = ((Select-String -LiteralPath 'CMakeLists.txt' -Pattern ([regex]::Escape($d)) -Quiet) -and (Select-String -LiteralPath 'CMakeLists_template.txt' -Pattern ([regex]::Escape($d)) -Quiet))
    if ((Test-Path -LiteralPath $d) -and $inCm) { Ok "$d 存在且在 include 列表里" }
    else { Warn "$d 或其 include 行缺失" }
}
$srcCount = (Get-ChildItem -Path User -Recurse -File -Include *.c,*.cpp,*.h | Measure-Object).Count
Info "User/ 下源文件 $srcCount 个(由 CMakeLists 里的 file(GLOB_RECURSE ... User/*.*) 收集)"

Write-Host ""
if ($script:issues -eq 0) { Write-Host "自检通过:没有需要人工处理的问题 ✓" -ForegroundColor Green }
else { Write-Host "自检发现 $script:issues 处需要注意(见上面 [注意]/[错误])" -ForegroundColor Yellow }

if (-not $NoBuild)
{
    Write-Host "`n=== 6. 编译验证 ==="
    $cmake = 'E:\clion\CLion 2026.2.2\bin\cmake\win\x64\bin\cmake.exe'
    if (Test-Path -LiteralPath $cmake) {
        $buildDir = 'cmake-build-debug-mingw'
        & $cmake -S . -B $buildDir 2>&1 | Select-String -Pattern 'Configuring done|CMake Error' | ForEach-Object { Info $_.Line.Trim() }

        # ★ 编译重试:本机 MinGW 偶发 "can't create xxx.obj.d: No such file or directory"
        #   (重新配置后第一次编译时最容易出现),重跑一次必过 —— 试两次,第二次仍失败才算真失败。
        $attempt = 0
        $buildOk = $false
        while ($attempt -lt 2 -and -not $buildOk) {
            $attempt++
            $out = & $cmake --build $buildDir 2>&1
            $code = $LASTEXITCODE
            if ($code -eq 0) {
                $out | Select-String -Pattern 'FLASH:|RAM_D1:|Linking' -CaseSensitive | ForEach-Object { Info $_.Line.Trim() }
                $buildOk = $true
            }
            else {
                if ($attempt -lt 2) { Warn "第 1 次编译撞上 MinGW 偶发错误(常见:can't create *.obj.d),自动重试……" }
                else {
                    $out | Select-String -Pattern 'error:|Error' -CaseSensitive | Select-Object -First 8 | ForEach-Object { Info $_.Line.Trim() }
                }
            }
        }
        if ($buildOk) { Ok "build 成功$(if ($attempt -gt 1) { "(第 $attempt 次)" })" } else { Bad 'build 失败(重试后仍失败,见上面报错)' }
    } else { Warn "找不到 cmake:$cmake(跳过编译,自己在 CLion 里 Build 即可)" }
}
