#include "Bsp_key.h"


void Bsp_key_Init(void)
{
     NVIC_EnableIRQ(GPIO_INT_IRQN);//开启按键引脚的GPIOA端口中断
}



//void GROUP1_IRQHandler(void)//Group1的中断服务函数
//{
//    //读取Group1的中断寄存器并清除中断标志位
//    switch( DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1) )
//    {
//        //检查是否是KEY的GPIOA端口中断，注意是INT_IIDX，不是PIN_18_IIDX
//        case User_Key_INT_IIDX:
//            //如果按键按下变为高电平
//            if( DL_GPIO_readPins(User_Key_PORT, User_Key_PIN_18_PIN) > 0 )
//            {
//                //设置LED引脚状态翻转
//                DL_GPIO_togglePins(LED1_PORT, LED1_PIN_28_PIN);
//							

//							
//            }
//        break;
//    }
//}

