/**********************************************************************************/
/******************************调试记录********************************************/
/**********************************************************************************/
//NODE  DR SF影响开窗时间 SF越大 开窗时间越久  SF7 34MS  SF10 200MS
//2020/01/05  debug打印选择 UART2&UART3
//      CONFIG_UART_DEBUG=0 //默认 
//      CONFIG_UART_CMD=1  //默认
//2020/01/07  RFSWITCH开关选择
//      注意:放到自制件中需要修改 RFSWITCH 具体参考网关
//      正式交付模式 TEST需要屏蔽 同时需要修改rfswitch
//2020/01/10数据发送 接收buff  
//      AppData[LORAWAN_APP_DATA_MAX_SIZE];
//      tx_cmd_str[LORAWAN_APP_DATA_MAX_SIZE]
//2020/01/19 LoRaMacClassB.c的RxBeaconSetup( TimerTime_t rxTime, bool activateDefaultChannel )中
//      frequency=502700000;//suqi 強制賦值 test
//2020/01/21   在ATcmd.c中 static int node_workmode(int argc, char *argv[])中加入ClassB的参数初始化
//      LWanDevConfig_t default_dev_config = (LWanDevConfig_t)LWAN_DEV_CONFIG_DEFAULT;
//      default_dev_config.classb_param.beacon_freq=502700000;
//      default_dev_config.classb_param.beacon_dr  =DR_TEST;
//      default_dev_config.classb_param.pslot_freq =502700000;
//      default_dev_config.classb_param.pslot_dr   =DR_TEST;
//      g_lwan_dev_config_p = lwan_dev_config_init(&default_dev_config);
//2020/01/21  void RegionCommonRxBeaconSetup( RegionCommonRxBeaconSetupParams_t* rxBeaconSetupParams ) 中加入
//     printf("Rxbeacon, freq: %lu, sf: %d tim:%ld\r\n", rxBeaconSetupParams->Frequency , (12-datarate),rxBeaconSetupParams->RxTime);
//     参数调试
//2020/01/21  GPIOC, GPIO_Pin_1 上电时候为高 设置完RTC为低 rcconfig时候为高 超时后为低  需要用抓IO口电平
//2020/02/10  目前用的到中断有三个
//            RTC_Alarm_IRQHandler  //rtc中断
//            INTERRUPT_HANDLER(EXTI7_IRQHandler) //IO口外部中断
//            INTERRUPT_HANDLER(TIM2_CAP_IRQHandler) 串口中断接收中断
//2020/0216   printf("write time1: %lu\r\n",McuWakeUpTime);    打印时候使用%d 会导致打印数值不准 使用 %lu
//2020/02/20 读取时间tick 由u32更改为u32+u16 測試底层驱动OK 上层软件未测试 未实现