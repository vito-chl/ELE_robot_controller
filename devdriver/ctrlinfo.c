#include"ctrlinfo.h"

#define CTRLINFOR_UART "uart2"
#define CMD_MAX_LEN 64

static rt_device_t crtlinfo_uart;

static struct rt_semaphore crtlinfo_uart_irq_rx_sem;
static struct rt_semaphore crtlinfo_uart_finish_rec_sem;

static uint8_t data_recing = 0;

struct serial_configure crtlinfo_uart_config = RT_SERIAL_CONFIG_DEFAULT;

static uint8_t rec_databuf[64];
static uint8_t rec_databytes;

data_handle_t ctrl_funclist[FUNCLIST_SIZE];

extern uint8_t return_data_buf[];
extern uint8_t return_data_bufsize;

/* 接收中断回调函数 */
static rt_err_t uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    if (size > 0)
    {
        rt_sem_release(&crtlinfo_uart_irq_rx_sem);
    }
    return RT_EOK;
}


/* 数据接受线程处理函数 */
static void rxuart_thread_entry(void *parameter)
{
	char ch;
	int8_t rec_num;
	int8_t all_data_num;
	
    while(1)
    {
		rec_num = 0;
		all_data_num = 0;
		
		while(all_data_num==0 || rec_num!=all_data_num){ 	//串口接受到数据
			
			while(rt_device_read(crtlinfo_uart, -1, &ch, 1) == 0)
			{
				rt_sem_take(&crtlinfo_uart_irq_rx_sem, RT_WAITING_FOREVER);
			}

			if(data_recing==0 && ch==0x3A) //接收到帧头
			{
				data_recing = 1;	
				rec_num = 0;
				all_data_num = 0;
			}
			
			if(data_recing == 1)
			{
				rec_num++; 
				if(rec_num==4) // 寻找总字节数位置
				{
					if(ch > 4)
						goto __rec_reset;

					all_data_num = 12; //不需要计算 协议固定了
					rec_databytes = all_data_num;
				}
				rec_databuf[rec_num-1]=ch;
			}
		}
		
		if(rec_databuf[rec_databytes-1]==0x0A && rec_databuf[rec_databytes-2]==0x0D)
		{
			data_recing = 0;
			rt_sem_release(&crtlinfo_uart_finish_rec_sem); // 接受完成请求处理
			goto __rec_reset;
		}
			
		
		__rec_reset:
			rec_num = 0;
			all_data_num = 0;
	}
}


/* 命令执行函数 */
static void directive_run_thread_entry(void* arg)
{
	data_handle_t* ihandle;
	uint8_t devaddr_rec;
	uint8_t funcnum_rec;
	
	int32_t data_rec;
	
	uint16_t crc;
	
	while(1)
	{
		rt_sem_take(&crtlinfo_uart_finish_rec_sem, RT_WAITING_FOREVER);
		
		crc = get_crc16(&rec_databuf[1], rec_databytes-5);
		
		if((uint8_t)(crc>>8)!=rec_databuf[rec_databytes-4] ||	\
			(uint8_t)crc!=rec_databuf[rec_databytes-3])
			continue;

		data_rec = 	((uint32_t)rec_databuf[4])<<24 | \
					((uint32_t)rec_databuf[5])<<16 | \
					((uint32_t)rec_databuf[6])<<8  | \
					((uint32_t)rec_databuf[7]);
		
		for(int i=0; i<FUNCLIST_SIZE; i++)
		{
			ihandle = &ctrl_funclist[i];
			devaddr_rec = rec_databuf[1];
			funcnum_rec = rec_databuf[2];
			
			if(devaddr_rec==ihandle->dev_addr && funcnum_rec==ihandle->func_num && ihandle->func!=0) // 匹配
			{
				int ret = ihandle->func(devaddr_rec ,data_rec);
				if(ret == 0) // 执行成功且没有读出数据
				{
					rt_device_write(crtlinfo_uart, 0, rec_databuf, rec_databytes);//写回消息
				}
				else if(ret == 1) // 执行成功需要返回数据
				{
					rt_device_write(crtlinfo_uart, 0, return_data_buf, return_data_bufsize);//写回消息
				}
				break;
			}
			else rt_kprintf("Error: ctrl info\n");
		}
	}
}

/* 初始化 */
int init_ctrlinfo(void)
{
    crtlinfo_uart = rt_device_find(CTRLINFOR_UART);

    crtlinfo_uart_config.baud_rate = BAUD_RATE_19200;
    crtlinfo_uart_config.data_bits = 8;
    crtlinfo_uart_config.stop_bits = 1;
    crtlinfo_uart_config.bufsz = 256;
    crtlinfo_uart_config.parity = PARITY_NONE;
    crtlinfo_uart_config.bit_order = BIT_ORDER_LSB;

    rt_device_control(crtlinfo_uart, RT_DEVICE_CTRL_CONFIG, &crtlinfo_uart_config);
    rt_device_open(crtlinfo_uart, RT_DEVICE_FLAG_INT_RX);
	
	rt_sem_init(&crtlinfo_uart_irq_rx_sem, "ciirq", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&crtlinfo_uart_finish_rec_sem, "cirecf", 0, RT_IPC_FLAG_FIFO);
	
	rt_device_set_rx_indicate(crtlinfo_uart, uart_rx_ind);
	
    rt_thread_t thread_uart = rt_thread_create("cirec", rxuart_thread_entry, RT_NULL, 1024, 25, 10);
	rt_thread_t thread_data = rt_thread_create("ciexe", directive_run_thread_entry, RT_NULL, 4096, 25, 10);
	
	if (thread_uart != RT_NULL)
    {
        rt_thread_startup(thread_uart);
		rt_kprintf("ctrlinfo_rec_uart: ctrlinfo_rec_uart init finish\n");
    }
	if (thread_data != RT_NULL)
    {
        rt_thread_startup(thread_data);
		rt_kprintf("ctrlinfo_exec_cmd: ctrlinfo_exec_cmd init finish\n");
    }

    rt_kprintf("ctrlinfo: all init finish\n");
	
	return 0;
}
INIT_APP_EXPORT(init_ctrlinfo);
