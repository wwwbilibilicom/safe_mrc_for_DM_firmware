# USART1 Debug Log

## Step 1：确认 MRC_DebugCLI_UART_IdleHandler 从未执行
检查结果：进入了中断，但是buffer没有数据

## Step 2：检查 DMAMUX1 Channel 1（最关键的寄存器）
地址 0x40020804
数据：29 00 00 2A

## Step 3：检查 DMA1 Stream1 寄存器
 DMA1_S1M0AR = 0x2000001C

## Step 4：确认 buffer 实际地址
地址为：0x20000131