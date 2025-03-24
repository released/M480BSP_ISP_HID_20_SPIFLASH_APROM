# M480BSP_ISP_HID_20_SPIFLASH_APROM
 M480BSP_ISP_HID_20_SPIFLASH_APROM

update @ 2025/03/24

1. combine BSP sample code : ISP_HID_20 and ISP_UART_SPIFLASH_M487KM , to upgrade SPI flash under ISP tool

2. under isp_user.c , need to modify as below 

__keypoint in isp_user.c__

```c
    else if (lcmd == CMD_GET_DEVICEID)
    {
        outpw(response + 8, SYS->PDID);
        // outpw(response + 8, 0x00D4874E);
        goto out;
    }
```

```c
    else if (lcmd == CMD_CONNECT)
    {
		outpw(response + 8, 0x001540EF);
        g_packno = 1;
        goto out;
    }
```

```c
    else if (lcmd == CMD_ERASE_SPIFLASH)
    {
        uint32_t u32Addr = inpw(pSrc);
        SPIM_EraseBlock(u32Addr, 0, OPCODE_BE_64K, 1, 1);
        CLK_SysTickLongDelay(5000000);

    }
    else if (lcmd == CMD_UPDATE_SPIFLASH)
    {
        uint32_t u32Addr, u32NTx;
        u32Addr = inpw(pSrc);
        u32NTx = inpw(pSrc + 4);
        pSrc += 8;
        //SPIM_SetQuadEnable(0, 1);
        SPIM_DMA_Write(u32Addr, 0, u32NTx, pSrc, CMD_NORMAL_PAGE_PROGRAM);
    }
```

3. add SPIM init in main.c , use UART0 to print debug message , check define : ENABLE_UART0 , UART0_PRINTF

__keypoint in main.c__

```c
    g_apromSize = 0x80000 - 0x3000; // reserve data flash 4K , APROM_Bootloader.bin : 8K
    g_dataFlashAddr = 0x7F000;
    g_dataFlashSize = 0x1000;
```

4. add SPIM driver will more than 4K size  , hence need to add last 3 pages in APROM for boot loader ( include data flash : 4K)

use hid_20_iap.sct to spilt boot loader in LDROM and APROM

__keypoint in hid_20_iap.sct__

```c
	LOAD_ROM_1  0x100000 0xFFF
	{
		LDROM_Bootloader.bin  0x100000 0xFFF
		{
			startup_M480.o (RESET, +FIRST)
			.ANY (+RO)		
			;fmc_user.o (+RO)
			;hid_transfer.o (+RO)
			;hsusbd_user.o (+RO)
			isp_user.o (+RO)
			spim.o (+RO)
			;system_m480.o (+RO)
			;targetdev.o (+RO)
			main.o (+RO)
		}
		
		SRAM  0x20000000 0x20000
		{
			* (+RW, +ZI)
		}
	}

	LOAD_ROM_2  0x7D000 0x2000
	{
		APROM_Bootloader.bin  0x7D000 0x2000
		{
			.ANY (+RO)
		}
	}


```

5. below is ICP setting and config setting capture , when programming boot loader (LDROM_Bootloader.bin,APROM_Bootloader.bin)

ICP tool , main page
![image](https://github.com/released/M480BSP_ISP_HID_20_SPIFLASH_APROM/blob/main/ICP_connnect.jpg)

config bits setting (boot from LDROM with IAP , enable data flash 4K , set SPIM pin config)
![image](https://github.com/released/M480BSP_ISP_HID_20_SPIFLASH_APROM/blob/main/ICP_config.jpg)

use ICP tool to verity APROM_Bootloader.bin existence
![image](https://github.com/released/M480BSP_ISP_HID_20_SPIFLASH_APROM/blob/main/ICP_connnect2.jpg)

6. below is ISP operation ,

test ISP tool connection (use M487 PFM evb to test)
![image](https://github.com/released/M480BSP_ISP_HID_20_SPIFLASH_APROM/blob/main/ISP_connnect.jpg)

press EVB button : SW2 , to entry boot loader mode
![image](https://github.com/released/M480BSP_ISP_HID_20_SPIFLASH_APROM/blob/main/log_entry_boot_code.jpg)

must erase SPI flash individually
![image](https://github.com/released/M480BSP_ISP_HID_20_SPIFLASH_APROM/blob/main/ISP_connnect2.jpg)

program APROM and SPIM bin ( use BSP/SPIM_DMM_RUN_CODE for example)
![image](https://github.com/released/M480BSP_ISP_HID_20_SPIFLASH_APROM/blob/main/ISP_connnect3.jpg)


7. flash address mapping as below 

LDROM : 0x100000 ( 1 PAGE ) , programming with LDROM_Bootloader.bin

APROM : 0x7D000 ( 2 PAGES ) , programming with APROM_Bootloader.bin

DATA FLASH : 0x7F000 ( 1 PAGE )

![image](https://github.com/released/M480BSP_ISP_HID_20_SPIFLASH_APROM/blob/main/memory_map_512K.jpg)

8. below is log message when entry boot loader and app code

![image](https://github.com/released/M480BSP_ISP_HID_20_SPIFLASH_APROM/blob/main/log.jpg)


