#include "uart_ttys.hpp"

#define UART_DEV    "/dev/ttyS8"

static int fd_ttys = -1;

int uart_init(void)
{
    fd_ttys = open(UART_DEV, O_RDWR|O_NOCTTY);
    if(-1 == fd_ttys)
    {
        perror("can't open serial port");
        return -1;
    }

    //==========配置串口============//
    struct  termios opt;          //配置串口的属性定义在结构体struct termios中
    tcgetattr(fd_ttys, & opt);         //获取终端控制属性
    
    cfsetispeed(& opt, B460800);  //指定输入波特率(若不设置系统默认9600bps)
    cfsetospeed(& opt, B460800);  //指定输出波特率(若不设置系统默认9600bps)
 
    /* c_lflag 本地模式 */
    opt.c_cflag &= ~ INPCK;           //不启用输入奇偶检测
    opt.c_cflag |= (CLOCAL |  CREAD); //CLOCAL忽略 modem 控制线,CREAD打开接受者
 
    /* c_lflag 本地模式 */
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE |  ISIG); //ICANON启用标准模式;ECHO回显输入字符;ECHOE如果同时设置了 ICANON，字符 ERASE 擦除前一个输入字符，WERASE 擦除前一个词;ISIG当接受到字符 INTR, QUIT, SUSP, 或 DSUSP 时，产生相应的信号
 
    /* c_oflag 输出模式 */
    opt.c_oflag &= ~ OPOST;             //OPOST启用具体实现自行定义的输出处理
    opt.c_oflag &= ~(ONLCR | OCRNL);    //ONLCR将输出中的新行符映射为回车-换行,OCRNL将输出中的回车映射为新行符
 
    /* c_iflag 输入模式 */
    opt.c_iflag &= ~(ICRNL |  INLCR);          //ICRNL将输入中的回车翻译为新行 (除非设置了 IGNCR),INLCR将输入中的 NL 翻译为 CR
    opt.c_iflag &= ~(IXON | IXOFF | IXANY);    //IXON启用输出的 XON/XOFF流控制,IXOFF启用输入的 XON/XOFF流控制,IXANY(不属于 POSIX.1；XSI) 允许任何字符来重新开始输出
 
    /* c_cflag 控制模式 */
    opt.c_cflag &= ~ CSIZE;     //字符长度掩码,取值为 CS5, CS6, CS7, 或 CS8,加~就是无
    opt.c_cflag |=  CS8;        //数据宽度是8bit
    opt.c_cflag &= ~ CSTOPB;    //CSTOPB设置两个停止位，而不是一个,加~就是设置一个停止位
    opt.c_cflag &= ~ PARENB;    //PARENB允许输出产生奇偶信息以及输入的奇偶校验,加~就是无校验
 
    /* c_cc[NCCS] 控制字符 */
    opt.c_cc[VTIME] = 1 ;   //等待数据时间(10秒的倍数),每个单位是0.1秒  若20就是2秒
    opt.c_cc[VMIN] = 1 ;    //最少可读数据,非规范模式读取时的最小字符数，设为0则为非阻塞，如果设为其它值则阻塞，直到读到到对应的数据,就像一个阀值一样，比如设为8，如果只接收到3个数据，那么它是不会返回的，只有凑齐8个数据后一齐才READ返回，阻塞在那儿

    tcflush(fd_ttys, TCIOFLUSH);         //刷串口清缓存
    tcsetattr(fd_ttys, TCSANOW, &opt);   //设置终端控制属性,TCSANOW：不等数据传输完毕就立即改变属性
	
	return 0;
}

//todo:存在未写入完全的问题，对此情况进行处理
int uart_write(uint8_t *w_buff, uint16_t len)
{
    int write_size = write(fd_ttys, w_buff, len);
    if(write_size < 0)
    {
        perror("serial port write error");
        return -1;
    }
    return write_size;
}

int uart_read(uint8_t *r_buff,uint16_t len)
{
    int read_size = 0;
    read_size = read(fd_ttys, r_buff, len);
    if(read_size < 0)
    {
        perror("serial port read error");
        return -1;
    }
    return read_size;
}

void uart_close(void)
{
    close(fd_ttys);
}