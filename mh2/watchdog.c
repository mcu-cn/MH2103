// Watchdog handler on MH2
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_MH2H7
#include "internal.h" // IWDG
#include "sched.h" // DECL_TASK

#if CONFIG_MACH_MH2H7 // mh2h7 libraries only define IWDG1 and IWDG2
#define IWDG IWDG1
#endif

void
watchdog_reset(void)
{
    IWDG->KR = 0xAAAA;
}
DECL_TASK(watchdog_reset);

void
watchdog_init(void)
{
    #if 0
    IWDG->KR = 0x5555;
    IWDG->PR = 0;
    IWDG->RLR = 0x0FFF; // 410-512ms timeout (depending on mh2 chip)
    IWDG->KR = 0xCCCC;
    #else // 时间计算(40KHz的LSI情况下):T = ((4*2^pr)*rlr) / 40 (ms).
    IWDG->KR = 0X5555;//使能对IWDG->PR和IWDG->RLR的写		 										  
    IWDG->PR = 2;  //设置分频系数   
    IWDG->RLR = 0xFFF;  //从加载寄存器
    //T = ((4 * 2^2) * 0xFFF) / 40 = 1638ms
    IWDG->KR = 0XAAAA;//装载预装载值,MH2必须增加这一条
    IWDG->KR = 0XCCCC;//使能看门狗
    #endif
}
DECL_INIT(watchdog_init);
