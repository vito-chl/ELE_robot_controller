#include"motor.h"

#define MOTOR_UART "uart3"
#define CMD_MAX_LEN 64

static rt_device_t motor_uart;

static struct rt_semaphore motor_uart_rx_sem;
static struct rt_semaphore motor_uart_start_rec_sem;
static struct rt_semaphore motor_uart_finish_rec_sem;

struct serial_configure motor_uart_config = RT_SERIAL_CONFIG_DEFAULT;

static uint8_t rec_databuf[64];
static uint8_t rec_databytes;


/* 接收数据回调函数 */
static rt_err_t uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    if (size > 0)
    {
        rt_sem_release(&motor_uart_rx_sem);
    }
    return RT_EOK;
}

static void motor_rxuart_thread_entry(void *parameter)
{
	char ch;
	int8_t rec_num;
	int8_t all_data_num;
	
    while(1)
    {
		rt_sem_take(&motor_uart_start_rec_sem, RT_WAITING_FOREVER); //等待接受请求信号量 
		rec_num = 0;
		all_data_num = 0;
		
		while(all_data_num==0 || rec_num!=all_data_num){
			//等待串口接受到数据
			while(rt_device_read(motor_uart, -1, &ch, 1) == 0)
			{
				rt_sem_take(&motor_uart_rx_sem, RT_WAITING_FOREVER);
			}
			
			rec_num++;
			
			if(rec_num==3) 
			{
				all_data_num = ch+5;
				rec_databytes = all_data_num;
			}
				
			rec_databuf[rec_num-1]=ch;
		}
		rt_sem_release(&motor_uart_finish_rec_sem);
	}
}



/* 初始化与电机控制器通信的串口 */
int init_motor_uart(void)
{
    motor_uart = rt_device_find(MOTOR_UART);
	
	if(motor_uart!=RT_NULL)
		rt_kprintf("motor_uart: find uart2 and set it\n");
	else 
	{
		rt_kprintf("motor_uart: don't find uart2, fail init motor\n");
		return 0;
	}

    motor_uart_config.baud_rate = 115200;
    motor_uart_config.data_bits = 8;
    motor_uart_config.stop_bits = 1;
    motor_uart_config.bufsz = 256;
    motor_uart_config.parity = PARITY_NONE;
    motor_uart_config.bit_order = BIT_ORDER_LSB;

    if(rt_device_control(motor_uart, RT_DEVICE_CTRL_CONFIG, &motor_uart_config)!=RT_EOK)
	{
		rt_kprintf("motor_uart: can't config uart2\n");
	}
	else 
	{
		rt_kprintf("motor_uart: config uart2 finish\n");
	}
	
	if(rt_device_open(motor_uart, RT_DEVICE_FLAG_INT_RX)!=RT_EOK)
	{
		rt_kprintf("motor_uart: can't open uart2\n");
		rt_device_close(motor_uart);
	}
	else 
	{
		rt_kprintf("motor_uart: open uart2 finish\n");
	}
	
	rt_sem_init(&motor_uart_rx_sem, "mtirq", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&motor_uart_start_rec_sem, "mtsr", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&motor_uart_finish_rec_sem, "mtfr", 0, RT_IPC_FLAG_FIFO);
	
	rt_device_set_rx_indicate(motor_uart, uart_rx_ind);
	rt_kprintf("motor_uart: bind uart2 irq to func\n");
	
    rt_thread_t thread = rt_thread_create("mtrec", motor_rxuart_thread_entry, RT_NULL, 1024, 25, 10);
	
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
		rt_kprintf("motor_uart: motor_rec_uart thread init finish\n");
    }
	
    rt_kprintf("motor_uart %s: all init finish\n", MOTOR_UART);
	
	return 0;
}
INIT_APP_EXPORT(init_motor_uart);

/* 发送指令 */
static void send_motor_cmd(uint8_t* ptr, uint8_t len)
{
	uint16_t crc = get_crc16(ptr, len-2);
	ptr[len-1] = (uint8_t)(crc>>8);
	ptr[len-2] = (uint8_t)(crc);
	
	rt_device_write(motor_uart, 0, ptr, len);
}

/* 电机设置, 不接受返回值 */
int ctrl_motor_noret(uint8_t motor_id, uint8_t* cmd, uint8_t len)
{
	cmd[0] = motor_id;
	send_motor_cmd(cmd, len);
	return 0;
}

/* 电机设置, 接受返回值 u16 */
// crc 正确返回1 不正确返回0
int ctrl_motor_ret(uint8_t motor_id, uint8_t* cmd, uint8_t len, uint8_t* retptr, uint8_t* retlen)
{
	cmd[0] = motor_id;
	send_motor_cmd(cmd, len);
	
	/* 开始使能接受 */
	rt_sem_release(&motor_uart_start_rec_sem);
	
	/* 等待接受完毕 */
	rt_sem_take(&motor_uart_finish_rec_sem, RT_WAITING_FOREVER);
	
	/* crc校验 */
	uint16_t crc = get_crc16(rec_databuf, rec_databytes-2);
	
	/* 提取CRC高位低位 */
	uint8_t hcrc = (uint8_t)crc>>8;
	uint8_t lcrc = (uint8_t)crc;
	
	/* CRC验证 成功进行拷贝 */
	if(lcrc==rec_databuf[rec_databytes-2] && hcrc==rec_databuf[rec_databytes-1])
	{
		for(int i=0; i<rec_databytes; i++)
		retptr[i] = rec_databuf[i];
		*retlen = rec_databytes;
		return 1;
	}

	return 0;
}

void set_speed(uint8_t motorid, int16_t speed)
{
	uint8_t cmd[] = SET_MOVE_SPEED_CMD;
	cmd[4] = (uint8_t)(speed>>8);
	cmd[5] = (uint8_t)(speed);
	
	ctrl_motor_noret(motorid, cmd, sizeof(cmd));
}

void set_distance(uint8_t motorid, int32_t distance)
{
	uint8_t cmd[] = SET_MOVE_DISTANCE_CMD;
	cmd[7]  = (uint8_t)(distance>>24);
	cmd[8]  = (uint8_t)(distance>>16);
	cmd[9]  = (uint8_t)(distance>>8);
	cmd[10] = (uint8_t)(distance);
	
	ctrl_motor_noret(motorid, cmd, sizeof(cmd));
}

int16_t read_speed(uint8_t motorid)
{
	uint8_t cmd[] = READ_CUR_SPEED_VALUE_CMD;
	uint8_t buf[16]; 
	uint8_t len;
	uint16_t uret = 0; 
	if(ctrl_motor_ret(motorid, cmd, sizeof(cmd), buf, &len) == 1)
	{
		uret = (uint16_t)buf[3]<<8 | (uint16_t)buf[3];
	}
	return (int16_t)uret;
}

int32_t read_set_pos(uint8_t motorid)
{
	uint8_t cmd[] = READ_SET_DISTANCE_VALUE_CMD;
	uint8_t buf[16]; 
	uint8_t len;
	uint32_t uret = 0; 
	if(ctrl_motor_ret(motorid, cmd, sizeof(cmd), buf, &len) == 1)
	{
		uret=(uint32_t)buf[3]<<24 | (uint32_t)buf[4]<<16 |(uint32_t)buf[5]<<8 | (uint32_t)buf[6];
	}
	return (int32_t)uret;
}

int32_t read_cur_pos(uint8_t motorid)
{
	uint8_t cmd[] = READ_CUR_DISTANCE_VALUE_CMD;
	uint8_t buf[16]; 
	uint8_t len;
	uint32_t uret = 0; 
	if(ctrl_motor_ret(motorid, cmd, sizeof(cmd), buf, &len) == 1)
	{
		uret=(uint32_t)buf[3]<<24 | (uint32_t)buf[4]<<16 |(uint32_t)buf[5]<<8 | (uint32_t)buf[6];
	}
	return (int32_t)uret;
}

void motor_init(uint8_t motorid)
{
	uint8_t cmd_set_optmode[] =  SET_OPTMODE_CMD;
	ctrl_motor_noret(motorid, cmd_set_optmode, sizeof(cmd_set_optmode));
}

void motor_set_mode(uint8_t motorid, int mode)
{
	if(mode == SPEED_MODE)
	{
		uint8_t cmd_set_ctrlmode[] =  SET_CTRMODE_SPEED_CMD;
		ctrl_motor_noret(motorid, cmd_set_ctrlmode, sizeof(cmd_set_ctrlmode));
	}
	
	else if(mode == DISTANCE_MODE)
	{
		uint8_t cmd_set_ctrlmode[] = SET_CTRMODE_POS_CMD;
		uint8_t cmd_set_relative[] = SET_MOVMODE_RELATIVE_CMD;
		uint8_t cmd_enable_motor[] = ENABLD_MOTOR_CMD;
		
		ctrl_motor_noret(motorid, cmd_set_ctrlmode, sizeof(cmd_set_ctrlmode));
		ctrl_motor_noret(motorid, cmd_set_relative, sizeof(cmd_set_relative));
		ctrl_motor_noret(motorid, cmd_enable_motor, sizeof(cmd_enable_motor));	
	}
}