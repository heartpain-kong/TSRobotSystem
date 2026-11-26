//
// robot.cpp — 机器人层:唯一的初始化入口 + 高层逻辑(仿 basic_framework 的 application/robot.c)
//
// Robot_Init():各 BSP → 各子系统 Init(只初始化) → OSTaskInit() 建任务
// Robot_Task():跑一拍的高层决策(读遥控/视觉 → 给子系统发参考值)
//

#include "robot.h"
#include "octospi.h"   /* hospi2(采样 OSPI 状态用) */
#include "FreeRTOS.h"        /* RTOS 诊断:xPortGetFreeHeapSize / uxTaskGetStackHighWaterMark */
#include "task.h"
#include "robot_def.h"
#include "robot_task.h"         //OSTaskInit()

#include "bsp_fdcan.h"
#include "dev_w25q64.h"      //外部 Flash(W25Q64,走 OCTOSPI2)
#include "dev_bmi088.h"
#include "dev_bmi088.h"   /* second IMU (DM-IMU-L1, CAN) */      //IMU(BMI088,走 SPI2)
#include "dev_buzzer.h"      //蜂鸣器(PB15 / TIM12_CH2)
#include "dev_ws2812.h"     //WS2812 灯带(SPI6 / PA7,TX-only)
#include "bsp_power.h"      //板上 5V 输出使能(PC15)
#include "bsp_timestamp.h"  //长时间戳(TIM5 = 1MHz 自由计数,无中断)
#include "bsp_adc.h"        //ADC(ADC1 = PC4 循环采样;ADC3 = MCU 温度)
#include "bsp_flash.h"      //外部 8MB QSPI Flash 存储层(基于 dev_w25q64)
#include "dev_log.h"        //运行日志(环形→Flash LOG 区 + USB 实时控制台)
#include "dev_electric.h"   //继电器/电源输出通道(表驱动)
#include "dev_remote.h"         //遥控器/键鼠(DBUS,UART5)
#include "dev_referee.h"    //裁判系统(USART1 115200)
#include "dev_minipc.h"        //miniPC(USB CDC 虚拟串口,转发上位机/视觉)
#include "cmd_task.h"       //Cmd_Init():注册上/下 MCU 链路(USART2 RS485)
#include "msg_center.h"     //任务间通信:环形数组
#include "imu_task.h"        //姿态解算(BMI088 + EKF)
#include "chassis_task.h"    //底盘(麦轮)
#include "gimbal_task.h"     //云台(yaw/pitch)
#include "highRise_task.h"  //末端机构(发射/升降)
#include "debug_task.h"     //调试遥测(串口 → 上位机)
#include "usbd_cdc_if.h"    //USB CDC 发送(日志控制台用)

/* ---- 各子系统 ---- */
#if ROBOT_HAVE_MOTOR
#include "motor_task.h"
#endif

/* ---- 调试用的自检结果(调试器直接看这几个变量)---- */
volatile bool imu_ext_init_ok = false;  /* second IMU (DM-IMU-L1) init ok */
volatile bool bmi088_init_ok = false;   /* IMU:BMI088 CHIP_ID 自检是否通过 */
volatile bool buzzer_init_ok = false;   /* 蜂鸣器:TIM12_CH2 初始化是否成功 */
volatile bool ws2812_init_ok = false;   /* WS2812:SPI6 灯带初始化是否成功 */
volatile bool ws2812_clock_ok = false;  /* WS2812:SPI6 时钟是否为 2.5Mbit/s(false ⇒ CubeMX 里分频没改成 32) */
volatile bool adc1_init_ok = false;     /* ADC1(PC4)校准 + DMA 循环采样是否启动成功 */
volatile bool minipc_init_ok = false;      /* miniPC(USB CDC)是否初始化成功 */
volatile bool minipc_online = false;       /* 上位机是否在线 */
volatile uint32_t minipc_rx = 0u;          /* 上位机累计收帧数 */
volatile bool referee_init_ok = false;  /* 裁判系统(USART1)是否初始化成功 */
volatile bool referee_online = false;   /* 裁判系统是否在线 */
volatile uint32_t referee_rx = 0u;      /* 裁判系统累计收帧数 */
volatile uint16_t referee_hp = 0u;      /* 我方血量 */
volatile uint16_t referee_stage_s = 0u; /* 当前阶段剩余时间(s) */
volatile uint8_t referee_progress = 0u; /* 0 未开始 … 4 比赛中 */
static Robot_Data_t s_robot;                  /* 决策层状态(发给各子系统) */
volatile bool motor_485_init_ok = false;   /* RS485 电机(宇树 YS)初始化是否成功 */
volatile bool rc_init_ok = false;       /* 遥控器(DBUS/UART5)是否初始化成功 */
volatile bool rc_online = false;        /* 遥控器是否在线 */
volatile uint32_t rc_rx_count = 0u;     /* 遥控器累计收帧数 */
volatile bool log_init_ok = false;      /* 日志(Flash LOG 区)是否初始化成功 */
volatile bool electric_init_ok = false; /* 继电器/输出通道是否初始化成功 */
volatile bool power_voltage_ok = false; /* 电源电压检测是否就绪(ADC1/PC4) */
volatile float power_voltage = 0.0f;    /* 电源电压(V)—— 调试看这个 */
volatile bool log_full = false;         /* 日志记录区是否已满 */
volatile uint32_t log_dropped = 0u;     /* 日志丢弃字节数 */
volatile bool flash_init_ok = false;    /* 外部 8MB QSPI Flash(存储层)是否初始化成功 */
volatile uint32_t flash_jedec_id = 0u;  /* 读回来的 JEDEC ID,应为 0xEF4017 */
volatile float adc1_mv = 0.0f;          /* ADC1 平均电压(mV)—— 调试看这个 */
volatile float mcu_temp_c = 0.0f;       /* MCU 温度(℃)—— ADC3 内部传感器 */

/* USB CDC 实时日志控制台:把一段文本发到上位机
   ⚠️ USB 还没枚举好时 CDC_Transmit_HS 会返回 BUSY/FAIL —— 丢掉即可,绝不能卡住控制 */
static void Robot_Usb_Console(const char *__text, uint16_t __len)
{
    (void)CDC_Transmit_HS((uint8_t *)__text, __len);
}


/**
 * @brief 机器人初始化 —— 唯一需要放进 main.c 的函数(在 osKernelStart 之前调)
 *
 * 仿 basic_framework 的 RobotInit():各 BSP → 各子系统 Init。
 * 与参考工程有两处差异,都有原因:
 *   ① **建任务不在这里** —— CMSIS-RTOS2 要求先 osKernelInitialize() 才能建线程,
 *      而 main.c 的 USER CODE 2 在它之前。所以建任务拆成 OSTaskInit()
 *      (声明在 robot.h),由 Core/Src/freertos.c 的 RTOS_THREADS 段调用。
 *   ② **不关中断** —— 参考工程初始化期间 `__disable_irq()`;本工程不能关:
 *      灵足等模块的发送接口内部用 HAL_GetTick() 做 TX FIFO 超时等待,
 *      关中断后 SysTick 不涨 ⇒ 超时永远不触发 ⇒ 卡死。
 *      我们的初始化路径本来就不依赖中断,开着更安全。
 */

/* ============================================================================
 * RTOS 诊断量(调试器直接看)★ 卡死/任务消失时先看这几个
 *   rtos_free_heap       : 现在还剩多少 RTOS 堆(B)
 *   rtos_min_free_heap   : 历史最低剩余(B) —— 接近 0 就是"堆被吃干"
 *   hwm_xxx              : 各任务栈高水位(单位 = word = 4 B) —— 越小越接近溢出
 *   thread_create_failed : >0 说明有任务根本没建出来(堆不够)
 * ========================================================================== */

extern osThreadId_t motor_task_handle;
extern osThreadId_t robot_task_handle;
extern osThreadId_t error_task_handle;
extern osThreadId_t cmd_task_handle;
extern osThreadId_t imu_task_handle;
extern osThreadId_t chassis_task_handle;
extern osThreadId_t gimbal_task_handle;
extern osThreadId_t highRise_task_handle;
extern volatile uint32_t g_thread_create_failed;

volatile uint32_t rtos_free_heap = 0u;
volatile uint32_t rtos_min_free_heap = 0u;
volatile uint32_t thread_create_failed_dbg = 0u;
volatile uint32_t hwm_motor = 0u, hwm_robot = 0u, hwm_error = 0u, hwm_cmd = 0u;
volatile uint32_t hwm_imu = 0u, hwm_chassis = 0u, hwm_gimbal = 0u, hwm_highRise = 0u;

static void RTOS_Update_Diagnostics(void)
{
    rtos_free_heap = (uint32_t)xPortGetFreeHeapSize();
    rtos_min_free_heap = (uint32_t)xPortGetMinimumEverFreeHeapSize();
    thread_create_failed_dbg = g_thread_create_failed;

    hwm_motor     = (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)motor_task_handle);
    hwm_robot     = (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)robot_task_handle);
    hwm_error     = (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)error_task_handle);
    hwm_cmd       = (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)cmd_task_handle);
    hwm_imu       = (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)imu_task_handle);
    hwm_chassis   = (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)chassis_task_handle);
    hwm_gimbal    = (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)gimbal_task_handle);
    hwm_highRise  = (uint32_t)uxTaskGetStackHighWaterMark((TaskHandle_t)highRise_task_handle);
}
extern "C" void Robot_Init(void)
{
    /* ★ 采样 2:Robot_Init 入口的 OSPI 状态(1=READY 正常 ; 2/4 = 已经忙) */
    /* Robot_Init entry */
    flash_diag_err = (uint32_t)hospi2.State;

    /* 1) CAN 过滤器(上电一次;必须在各电机 bsp_can_init() 之前) */
    /* 0) 长时间戳(TIM5,1MHz 自由计数 ⇒ 1 计数 = 1µs,3600s 才回绕)★ 必须最先做:
          后面所有模块(BMI088/Flash/蜂鸣器/WS2812)的超时、以及电机频率排程都要用它。
          内部只 HAL_TIM_Base_Start(**不开中断**),不占 NVIC、不影响 RTOS。 */
    /* 任务间通信(环形数组)先清空,后面的模块才会往里发 */
    Msg_Init();

    Timestamp_Init(&htim5);
    can_filter_init();

    /* 2) 各子系统初始化(只初始化 + 注册接收回调,不建任务) */
#if ROBOT_HAVE_MOTOR
    motor_task_init();      /* CAN / 485 电机统一在这里初始化 ✓ */

#endif

    /* 2.1) IMU(BMI088 @ SPI2):注册 SPI 回调 + 自检两颗 die
           (加速度 CHIP_ID 应为 0x1E、陀螺 0x0F;结果看 bmi088_init_ok)
           ⚠️ 依赖:MX_SPI2_Init() 已在 main() 里跑过 */
    BSP_SPI_Init(&hspi2, nullptr);
    bmi088_init_ok = BMI088_Init();

    /* 外接 IMU(DM-IMU-L1)暂时不用:CAN3 专供上下 MCU 链路。
       以后要接回来,在这里恢复 imu_ext_init_ok = Imu_Ext_Init();(必须在 Cmd_Init 之前) */
    Cmd_Init();     /* cmd 层:注册上/下 MCU 链路(USART2 RS485,500Hz) */
    Imu_Init();     /* 姿态解算:初始化 EKF(设备 BMI088 上面已初始化) */
    Debug_Init();    /* 调试串口提前初始化:找零期间只发不收 */
#if !ROBOT_UPPER_ONLY
    Chassis_Init(); /* 底盘解算 + 找零(阻塞);期间 Debug_Send_Homing 直接发 TX。
                       ★ 上板没有舵向电机/光电,绝不能走这里(会一直等找零) */
#endif
    Gimbal_Init();  /* 云台:关节角控制(电机分配见 robot_def.h) */
    HighRise_Init(); /* 末端机构:摩擦轮/拨弹(机构类型见 robot_def.h 的 HIGHRISE_MECH) */

    /* 2.2) 外部 Flash(W25Q64 @ OCTOSPI2):注册 OSPI 回调 + 自检 JEDEC ID(EF 40 17)
           详见 User/Bsp/flash/bsp_flash.md
           ⚠️ 依赖:MX_OCTOSPI2_Init() 已在 main() 里跑过(HAL_OSPI_Init + OSPIM 配置) */
    /* 注:W25Q64 的注册与初始化由存储层 Flash_Init() 统一做(见下面 2.9),这里不再重复 */

    /* 2.3) 蜂鸣器(PB15 = TIM12_CH2,已在 CubeMX 里配成 PWM;PSC=159 ⇒ 计数器 1MHz)
            ⚠️ 依赖:MX_TIM12_Init() 已在 main() 里跑过 */
    buzzer_init_ok = Buzzer_Init(&htim12, TIM_CHANNEL_2, 160000000u);

    /* 2.4) WS2812 灯带(SPI6 / PA7,TX-only):
            ⚠️ 依赖 CubeMX 里两点:① SPI6 的 BaudRatePrescaler = 32(2.5Mbit/s)
                                  ② NVIC 里 SPI6 global interrupt = Enable
            ws2812_clock_ok = false 就说明第 ① 点没改(灯会乱码,但不会死机) */
    ws2812_init_ok = WS2812_Init(WS2812_LED_NUM);
    ws2812_clock_ok = WS2812_Is_Clock_Ok();
    WS2812_Clear();
    WS2812_Show();

    /* 2.6) 板上 5V 输出(PC15):默认关闭;台架模式由 POWER_5V_ON_AT_BOOT 决定 */
#if !POWER_5V_ON_AT_BOOT
    Power_5V_Enable(false);
#endif

    /* 2.8) ADC:ADC1(PC4,16 位,2 个 rank)校准 + 启动 DMA 循环采样,CPU 之后只读缓冲;
            ADC3 只绑定句柄(内部温度传感器走按需阻塞转换,不用 DMA —— BDMA 写不了 RAM_D1) */
    adc1_init_ok = BSP_ADC_Init(&hadc1, 2);
    BSP_ADC_Bind_ADC3(&hadc3);

    /* 2.9) 外部 8MB QSPI Flash 存储层(手册 P12:存调试数据 / LCD 图片)
            内部会注册 OSPI 回调 + W25Q64_Init(读 JEDEC ID 自检),并读回 ID 供调试 */
#if FLASH_SELFTEST
    /* ★ 四步自测(日志仍关闭):结果在遥测"Flash 诊断"里 ✓ */
    (void)Flash_Self_Test();
#endif

#if FLASH_ENABLE
    flash_init_ok  = Flash_Init();
    flash_jedec_id = Flash_Get_Jedec_Id();

    /* 2.10) 运行日志(用同一颗 Flash 的 LOG 区)★ 必须在 Flash_Init() 之后 */
    log_init_ok = Log_Init();
#else
    /* ★ 外部 Flash 硬件不通 ⇒ 关掉存储/日志,不污染错误码和 LED ✓ */
    flash_init_ok = true;
    log_init_ok   = true;
    flash_jedec_id = 0u;
#endif

    /* 上电链:本次编号 / 本次首地址 / 上次首尾地址(方便查上一次上电写了哪一段) */
#if LOG_DEBUG_BOOT_CHAIN
    Log_Printf(LOG_LEVEL_INFO,
               "log: boot=%u start=0x%X prev_start=0x%X prev_stop=0x%X wrap=%u",
               (unsigned)Log_Get_Boot_Id(),
               (unsigned)(FLASH_LOG_ADDR + 4096u + Log_Get_Boot_Start()),
               (unsigned)(FLASH_LOG_ADDR + 4096u + Log_Get_Prev_Start()),
               (unsigned)(FLASH_LOG_ADDR + 4096u + Log_Get_Prev_Offset()),
               (unsigned)Log_Get_Wrap_Count());
#endif
    Log_Set_Console(Robot_Usb_Console);      /* 先注册;USB 起来后再 Enable(见 freertos.c) */
    Log_Printf(LOG_LEVEL_INFO, "boot: flash=%s jedec=%06X",
               flash_init_ok ? "ok" : "fail", (unsigned int)flash_jedec_id);

    /* 2.11) 继电器/电源输出通道(上电全断) */
    electric_init_ok = Electric_Init();
    Log_Printf(LOG_LEVEL_INFO, "electric: %d channel(s) configured",
               (int)Electric_Get_Channel_Count());

    /* 2.12) 电源电压检测(ADC1/PC4) */
    power_voltage_ok = Power_Voltage_Init();

    /* 2.12) 遥控器/键鼠(DBUS 100000 8E1,UART5 / PD2) */
    rc_init_ok = Remote_Init(&huart5);
    Log_Printf(LOG_LEVEL_INFO, "rc: init %s (uart5 100000 8E1)", rc_init_ok ? "ok" : "fail");

    /* 2.13) 裁判系统(USART1 115200,PA9/PA10)—— 数据同时给 cmd 和 robot */
    minipc_init_ok = MiniPC_Init();      /* miniPC:USB CDC(USB 本身由 freertos 的 StartDefaultTask 初始化) */
    referee_init_ok = Referee_Init(&huart1);
    Log_Printf(LOG_LEVEL_INFO, "referee: init %s (usart1 115200)", referee_init_ok ? "ok" : "fail");

    /* 2.15) 调试台:按需上电就打开 5V(见 robot_def.h 的 POWER_5V_ON_AT_BOOT) */
#if POWER_5V_ON_AT_BOOT
    Power_5V_Enable(true);    /* ★ PC15 高电平有效:5V 输出打开(供外接 IMU 等) ✓ */
    Log_Printf(LOG_LEVEL_INFO, "power: 5V forced ON at boot (bench mode)");
#endif

    /* 3) 建 RTOS 任务不在这里 —— 见 robot.h 里 OSTaskInit() 的说明 */
}

/**
 * @brief 机器人高层逻辑,跑一拍(由 ROBOT 任务按 ROBOT_TASK_PERIOD_MS 调用)
 *
 * 这里放"决策":遥控/视觉/上位机 → 算出底盘速度、云台角度、发射机构目标…
 * 电机自己的 1kHz 控制与发帧在 MOTOR 任务里,不在这里做。
 */
extern "C" void Robot_Task(void)
{
    /* 子系统"跑一拍":IMU 按 1ms 周期读;Flash 推进状态机(完成/超时都在里面) */
    /* BMI088_Task() 已交给 IMU 任务(1kHz);这里不再调 */
    W25Q64_Task();
    Buzzer_Task();      /* 处理 Buzzer_Beep() 的自动停止 */
    WS2812_Task();      /* WS2812 发送超时保护 */

    Timestamp_Update(); /* 长时间戳:软件计回绕(1 小时才可能用到一次) */

    /* 日志落盘(内部限速 1 条/ms) + 电压检测(内部限速 100Hz) */
    /* 设备推进 Remote_Task()/Referee_Task() 已交给 CMD 任务(100Hz);这里只读状态做调试量 */
    minipc_online      = MiniPC_Is_Online();
    minipc_rx          = MiniPC_Get()->Rx_Frames;
    referee_online   = Referee_Is_Online();
    referee_rx       = Referee_Get()->Rx_Frames;
    referee_hp       = Referee_Get()->Hp;
    referee_stage_s  = Referee_Get_Stage_Remain();
    referee_progress = Referee_Get()->Game_Progress;
    /* ---- 决策层:按流程图,裁判系统 → robot → chassis/gimbal/highRise ----
       裁判系统离线(练习/调试)⇒ 一律允许;在线 ⇒ 只有比赛进行中(Game_Progress == 4)才允许 */
    s_robot.Seq++;
    s_robot.Referee_Online = referee_online;
    s_robot.Referee_Progress = referee_progress;
    s_robot.Motion_Allowed = (!referee_online) || (referee_progress == 4u);
    s_robot.Shoot_Allowed = s_robot.Motion_Allowed;
    s_robot.Mode_Limit = s_robot.Motion_Allowed ? 0u : 2u;
    (void)Msg_Publish(MSG_TOPIC_ROBOT, &s_robot, (uint16_t)sizeof(s_robot));

    rc_online    = Remote_Is_Online();
    rc_rx_count  = Remote_Get_Rx_Count();
    Log_Task();
    Power_Task();
    power_voltage = Power_Get_Voltage();
    log_full      = Log_Is_Full();
    log_dropped   = Log_Get_Dropped_Bytes();

    /* ADC 调试量:每拍刷新,调试器直接看 */
    /* RTOS 诊断:每 500ms 刷一次,调试器直接看(卡死/任务消失时最先看这几个) */
    {
        static uint32_t s_diag_ms = 0u;
        uint32_t now_ms = (uint32_t)Timestamp_Get_Ms();
        if ((uint32_t)(now_ms - s_diag_ms) >= 500u)
        {
            s_diag_ms = now_ms;
            RTOS_Update_Diagnostics();
        }
    }

    adc1_mv    = BSP_ADC_Get_Mv_Average(&hadc1, 2);
    mcu_temp_c = BSP_ADC_Get_Temperature();

    /* ---- 现在可以点灯了,例如:心跳/状态指示 ---- */
    /* WS2812_Set_All(0, 0, 8);  WS2812_Show(); */

#if ROBOT_HAVE_MOTOR
    /* 目前留空:等遥控/视觉接进来后,在这里算出参考值再下发,例如
     *     chassis.Set_Wheel_Speed(vx, vy, wz);     // 底盘解算后给四轮
     *     gimbal_yaw.Set_Pos(yaw_ref);             // 云台
     * 现在不动电机,上电不会乱冲。 */
#endif
}

/* ---- 决策层接口 ---- */

const Robot_Data_t *Robot_Get(void)
{
    return &s_robot;
}

bool Robot_Is_Motion_Allowed(void)
{
    return s_robot.Motion_Allowed;
}
