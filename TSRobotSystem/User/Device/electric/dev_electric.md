# dev_electric — 继电器 / 电源输出设备接口

文件：`dev_electric.h` / `dev_electric.cpp`

## 1. 它管什么

板上的**继电器**（或等效的 MOSFET 电源开关）通断 —— 比如 24V / 5V 支路。
每一路就是一个 GPIO：给有效电平就合上，给无效电平就断开。

## 2. 现状（本工程 CubeMX 配置）

| 通道 | 引脚 | 状态 |
|---|---|---|
| `ELECTRIC_CH_5V` | **PC15**（标签 `POWER_5V`）| **已配** ✓ —— 实际由 `User/Bsp/power` 负责，本模块只转发 |
| `ELECTRIC_CH_RELAY_1/2/3` | — | **未配** ⇒ 表里留空，操作返回 `false`，不会去动别的脚 ✓ |

## 3. ★ 加通道不用改代码

在 CubeMX 里把某个脚设成 `GPIO_Output`、**标签写成 `RELAY_1`（或 `RELAY_2` / `RELAY_3`）**，
重新生成即可 —— 本模块用 `#ifdef RELAY_1_Pin` 自动把它接进通道表 ✓

（CubeMX 会为每个带标签的脚生成 `RELAY_1_Pin` / `RELAY_1_GPIO_Port` 宏，和现有的
`POWER_5V_Pin` 是同一套机制。要更多路就把 `Enum_Electric_Channel` 里加一项、
表里再加一段 `#ifdef`。）

**每路的有效电平**在表里配：

```cpp
{RELAY_1_GPIO_Port, RELAY_1_Pin, false, "RL1"},   // active_high = false ⇒ 低电平合上
```

继电器模块大多是**低有效**，所以新加的通道默认写成 `false`；如果是高有效的板子改成 `true`。

## 4. API

| 接口 | 说明 |
|---|---|
| `Electric_Init()` | 初始化：**所有已配置通道置"断开"**（安全默认）|
| `Electric_Set(ch, on)` | 合上/断开某一路 |
| `Electric_Get(ch)` | 读该路当前状态（读的是**引脚电平**，不是命令值）|
| `Electric_Toggle(ch)` | 翻转 |
| `Electric_Set_All(on)` | 全部合上/断开（只作用于已配置通道）|
| `Electric_Is_Configured(ch)` | 该通道是否已配置 |
| `Electric_Get_Channel_Count()` | 已配置通道数 |
| `Electric_Get_Channel(index)` | 取第 index 个已配置通道（遍历用）|
| `Electric_Get_Name(ch)` | 通道名（调试/日志用）|

## 5. 用法（已接进 `Robot_Init()`）

```cpp
electric_init_ok = Electric_Init();            // 上电先全断(和 POWER_5V 出厂关闭一个思路)

Electric_Set(ELECTRIC_CH_RELAY_1, true);       // 合上 1 号
bool on = Electric_Get(ELECTRIC_CH_RELAY_1);

Electric_Set(ELECTRIC_CH_5V, true);            // 5V 也走这里(内部转发 Power_5V_Enable)
Electric_Set_All(false);                       // 全断
```

## 6. 与 `bsp_power` 的分工（别重复实现）

* **`User/Bsp/power`**：板载电源本体 —— **PC15 的 5V 使能** + **电源电压检测**（ADC1/PC4）✓
* **`User/Device/electric`**：**多路输出的统一接口** —— 把 5V 和以后的继电器放在一张表里，
  按通道号操作 ✓（5V 那一路**转发**给 `bsp_power`，PC15 只由一处负责 ✓）

## 7. ⚠️ 注意

1. **上电默认全部断开** ✓ —— 别指望它保持上次状态（要"记住状态"得自己存到 Flash，
   可以用 `User/Bsp/flash` + `User/Device/log` 那套 ✓）；
2. `Electric_Set()` 在 `Electric_Init()` 之前调用会返回 `false`（防止 GPIO 还没配好就乱写）✓；
3. **没有过流/过热反馈** ✗ —— 本接口只管"合/断"，负载保护得靠硬件；
4. 若以后继电器是**总线设备**（CAN / 串口继电器板），那属于"协议设备"，
   照 `User/Device/motor` 的模式再写一个（带帧解析 + 状态回读），本模块仍是 GPIO 版 ✓

## 8. 上板自检

```
① electric_init_ok == true（至少一路可用；现在就是 5V 那一路）
② Electric_Set(ELECTRIC_CH_5V, true) ⇒ 万用表量 5V 输出应出现；Electric_Get() 读回 true
③ 加一个 RELAY_1 标签重新生成 ⇒ Electric_Get_Channel_Count() 应从 1 变 2，代码没改一行 ✓
④ Electric_Set_All(false) ⇒ 所有已配置通道都断开
```
