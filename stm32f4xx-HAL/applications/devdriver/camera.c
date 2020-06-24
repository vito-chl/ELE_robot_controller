#include"camera.h"

/* 双光相机的通信通道 */
#define CARMEA_UART "uart2"
#define CMD_LEN     16

static int8_t cmd[CMD_LEN];

static rt_device_t camera_uart;

struct serial_configure camera_uart_config = RT_SERIAL_CONFIG_DEFAULT;

/* 初始化与双光相机通信的串口 */
int init_camera_uart(void)
{
    camera_uart = rt_device_find(CARMEA_UART);

    camera_uart_config.baud_rate = 115200;
    camera_uart_config.data_bits = 8;
    camera_uart_config.stop_bits = 1;
    camera_uart_config.bufsz = 256;
    camera_uart_config.parity = PARITY_NONE;
    camera_uart_config.bit_order = BIT_ORDER_LSB;

    rt_device_control(camera_uart, RT_DEVICE_CTRL_CONFIG, &camera_uart_config);

    rt_device_open(camera_uart, RT_DEVICE_FLAG_INT_RX);

    rt_kprintf("camera_uart: init finish\n");

    cmd[0] = 0xEB;
    cmd[1] = 0x90;

    for(int i=2; i<16; i++)
    {
        cmd[i] = 0;
    }

    return 0;
}

INIT_APP_EXPORT(init_camera_uart);

void set_angle(int16_t x, int16_t y)
{
    cmd[2] = 0x26;
    cmd[3] = (int8_t)(x>>8);
    cmd[4] = (int8_t)(x);
    cmd[5] = (int8_t)(y>>8);
    cmd[6] = (int8_t)(y);
}

void open_motor()
{
    cmd[2] = 0x27;
}

void close_motor()
{
    cmd[2] = 0x28;
}

void close_follow()
{
    cmd[2] = 0x29;
}

void follow()
{
    cmd[2] = 0x2a;
}

void to_mid()
{
    cmd[2] = 0x2b;
}

void prevent_gyro_drift()
{
    cmd[2] = 0x2c;
}

void set_zoom_rate(int8_t r)
{
    cmd[8] = r;
}

void update_cmd()
{
    /* 填写校验信息 */
    cmd[15] = 0;

    /* 串口发送指令 */
    rt_device_write(camera_uart, 0, cmd, 16);
}
