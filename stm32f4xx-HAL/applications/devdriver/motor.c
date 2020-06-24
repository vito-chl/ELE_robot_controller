#include"motor.h"

#define MOTOR_UART "uart3"
#define CMD_MAX_LEN 64

static rt_device_t motor_uart;

static struct rt_semaphore motor_uart_rx_sem;
static struct rt_semaphore motor_uart_start_rec_sem;
static struct rt_semaphore motor_uart_finish_rec_sem;

struct serial_configure motor_uart_config = RT_SERIAL_CONFIG_DEFAULT;

uint8_t rec_databuf[64];
uint8_t rec_databytes;


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

/* CRC校验 */
uint16_t get_crc16(uint8_t* ptr, uint8_t len)
{
	uint8_t i;
	uint16_t crc = 0xFFFF;
	if(len == 0) len = 1;
	while(len--)
	{
		crc ^= *ptr;
		for(i=0; i<8; i++)
		{
			if(crc&1)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else 
			{
				crc >>= 1;
			}
		}
		ptr++;
	}
	return crc;
}

/* 初始化与电机控制器通信的串口 */
int init_motor_uart(void)
{
    motor_uart = rt_device_find(MOTOR_UART);

    motor_uart_config.baud_rate = 115200;
    motor_uart_config.data_bits = 8;
    motor_uart_config.stop_bits = 1;
    motor_uart_config.bufsz = 256;
    motor_uart_config.parity = PARITY_NONE;
    motor_uart_config.bit_order = BIT_ORDER_LSB;

    rt_device_control(motor_uart, RT_DEVICE_CTRL_CONFIG, &motor_uart_config);
    rt_device_open(motor_uart, RT_DEVICE_FLAG_INT_RX);
	
	rt_sem_init(&motor_uart_rx_sem, "motor_uart_rx_sem", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&motor_uart_start_rec_sem, "motor_uart_start_rec_sem", 0, RT_IPC_FLAG_FIFO);
	rt_sem_init(&motor_uart_finish_rec_sem, "motor_uart_finish_rec_sem", 0, RT_IPC_FLAG_FIFO);
	
	rt_device_set_rx_indicate(motor_uart, uart_rx_ind);
	
    rt_thread_t thread = rt_thread_create("motor_rec_uart", motor_rxuart_thread_entry, RT_NULL, 1024, 25, 10);

    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
		rt_kprintf("motor_uart: motor_rec_uart init finish\n");
    }
	
    rt_kprintf("motor_uart: all init finish\n");
	
	return 0;
}
INIT_APP_EXPORT(init_motor_uart);

/* 发送指令 */
void send_motor_cmd(uint8_t* ptr, uint8_t len)
{
	uint16_t crc = get_crc16(ptr, len-2);
	ptr[len-2] = (uint8_t)crc>>8;
	ptr[len-1] = (uint8_t)crc;
	
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
	if(hcrc==rec_databuf[rec_databytes-2] && lcrc==rec_databuf[rec_databytes-1])
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
	uint8_t cmd[] = SET_MOVE_SPEED;
	cmd[4] = (uint8_t)speed>>8;
	cmd[5] = (uint8_t)speed;
	
	ctrl_motor_noret(motorid, cmd, sizeof(cmd));
}

void set_distance(uint8_t motorid, int32_t distance)
{
	uint8_t cmd[] = SET_MOVE_DISTANCE;
	cmd[7]  = (uint8_t)distance>>24;
	cmd[8]  = (uint8_t)distance>>16;
	cmd[9]  = (uint8_t)distance>>8;
	cmd[10] = (uint8_t)distance;
	
	ctrl_motor_noret(motorid, cmd, sizeof(cmd));
}

int16_t read_speed(uint8_t motorid)
{
	uint8_t cmd[] = READ_CUR_SPEED_VALUE;
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
	uint8_t cmd[] = READ_SET_DISTANCE_VALUE;
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
	uint8_t cmd[] = READ_CUR_DISTANCE_VALUE;
	uint8_t buf[16]; 
	uint8_t len;
	uint32_t uret = 0; 
	if(ctrl_motor_ret(motorid, cmd, sizeof(cmd), buf, &len) == 1)
	{
		uret=(uint32_t)buf[3]<<24 | (uint32_t)buf[4]<<16 |(uint32_t)buf[5]<<8 | (uint32_t)buf[6];
	}
	return (int32_t)uret;
}
