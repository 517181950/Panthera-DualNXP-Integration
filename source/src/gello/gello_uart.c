#include "gello_codec.h"
#include "gello_uart.h"
#include "core/controller.h"
#ifdef CONFIG_ROBOT_GELLO_USB
#include "gello_usb.h"
#else
#include <zephyr/drivers/uart.h>
#endif
#include <zephyr/net/socket.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <errno.h>
#include <string.h>

/* UART5 J5, external automatic TTL half-duplex interface.
 * IRQ RX, bounded read-only requests. CAN/control tasks have higher priority. */
#define GELLO_PORT 5021
#define REPLY_SIZE GELLO_DUAL_REPLY_SIZE
#ifdef CONFIG_ROBOT_GELLO_UART
static const struct device *const uart=DEVICE_DT_GET(DT_ALIAS(gello_uart));
#endif
K_SEM_DEFINE(gello_begin,0,1);
K_MSGQ_DEFINE(gello_rx,1,256,1);
static struct k_spinlock sample_lock;
static struct gello_tracker trackers[GELLO_SERVO_COUNT];
static int16_t velocities[GELLO_SERVO_COUNT];
static uint64_t stamps[GELLO_SERVO_COUNT];
static uint32_t scans, replies, timeouts, bad, jumps;
static atomic_t overflow;
static bool available;
/* SWD-only UART bring-up mailbox. Passive means no TX at any baud.
 * It never changes robot state, and resets to normal 1 Mbps on MCU boot. */
volatile uint32_t gello_debug_passive;
volatile uint32_t gello_debug_baud=1000000;
volatile uint32_t gello_debug_rx_total, gello_debug_uart_errors;
volatile uint8_t gello_debug_rx[256];

void gello_uart_start(void) { k_sem_give(&gello_begin); }
#ifdef CONFIG_ROBOT_GELLO_UART
static void rx_irq(const struct device *dev,void *data) {
    ARG_UNUSED(data);
    uart_irq_update(dev);
    int error=uart_err_check(dev);
    if(error>0)gello_debug_uart_errors++;
    unsigned received=0;
    while(received<32 && uart_irq_rx_ready(dev)) {
        uint8_t bytes[16];int n=uart_fifo_read(dev,bytes,sizeof(bytes));
        if(n<=0)break;
        received+=(unsigned)n;
        for(int i=0;i<n;i++) {
            gello_debug_rx[gello_debug_rx_total++%256]=bytes[i];
            if(!error && !gello_debug_passive && k_msgq_put(&gello_rx,&bytes[i],K_NO_WAIT))atomic_inc(&overflow);
        }
    }
    /* A foreign continuous stream or framing error must not monopolize
     * interrupts. The next bounded read transaction rearms RX IRQ. */
    if(error || received>=32)uart_irq_rx_disable(dev);
}
#endif
static bool read_one(uint8_t id) {
    int index=gello_servo_index(id);
    if(index<0)return false;
#ifdef CONFIG_ROBOT_GELLO_USB
    struct gello_sample sample;uint32_t invalid=0;
    bool received=gello_usb_read(id,&sample,&invalid);
    k_spinlock_key_t key=k_spin_lock(&sample_lock);
    bad+=invalid;
    bool accepted=false;
    if(received) {
        accepted=gello_track(&trackers[index],sample.raw);
        if(accepted) { velocities[index]=sample.velocity;stamps[index]=robot_us();replies++; }
        else jumps++;
    } else timeouts++;
    k_spin_unlock(&sample_lock,key);
    return accepted;
#else
    k_msgq_purge(&gello_rx);
    uint32_t errors_before=gello_debug_uart_errors;
    uart_irq_rx_enable(uart);
    struct gello_parser parser={0};struct gello_sample sample;
    uint8_t request[8];gello_read_request(id,request);
    for(int i=0;i<8;i++)uart_poll_out(uart,request[i]);
    uint64_t deadline=robot_us()+3000;
    unsigned consumed=0;
    while(robot_us()<deadline && consumed++<64) {
        uint8_t byte;uint64_t now=robot_us();
        if(now>=deadline)break;
        if(k_msgq_get(&gello_rx,&byte,K_USEC(deadline-now)))break;
        if(gello_parse(&parser,byte,&sample) && sample.id==id) {
            if(gello_debug_uart_errors!=errors_before)break;
            k_spinlock_key_t key=k_spin_lock(&sample_lock);
            bad+=parser.bad;
            bool accepted=gello_track(&trackers[index],sample.raw);
            if(accepted) { velocities[index]=sample.velocity;stamps[index]=robot_us();replies++; }
            else jumps++;
            k_spin_unlock(&sample_lock,key);
            return accepted;
        }
    }
    k_spinlock_key_t key=k_spin_lock(&sample_lock);
    timeouts++;bad+=parser.bad;
    k_spin_unlock(&sample_lock,key);
    return false;
#endif
}
static void collect(void *a,void *b,void *c) {
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
    k_sem_take(&gello_begin,K_FOREVER);
#ifdef CONFIG_ROBOT_GELLO_USB
    int ret=gello_usb_start();
    if(ret) { printk("GELLO USB J11 init failed: %d\n",ret);return; }
    printk("GELLO USB J11 waiting for CDC adapter; UDP5021 read-only\n");
    k_spinlock_key_t key;
#else
    if(!device_is_ready(uart)||uart_irq_callback_user_data_set(uart,rx_irq,NULL)) {
        printk("GELLO UART5 init failed\n");return;
    }
    uart_irq_rx_enable(uart);
    k_spinlock_key_t key=k_spin_lock(&sample_lock);available=true;k_spin_unlock(&sample_lock,key);
    printk("GELLO J5 UART5 1000000 8N1 IDs1-7 read-only; UDP5021\n");
    uint32_t current_baud=1000000;
#endif
    for(;;) {
#ifdef CONFIG_ROBOT_GELLO_USB
        bool ready=gello_usb_ready() && !gello_debug_passive;
        key=k_spin_lock(&sample_lock);available=ready;
        if(!ready)memset(stamps,0,sizeof(stamps));
        k_spin_unlock(&sample_lock,key);
        if(!ready) { k_sleep(K_MSEC(10));continue; }
#else
        if(gello_debug_passive) {
            uint32_t requested=gello_debug_baud;
            if(requested==115200 || requested==500000 || requested==921600 || requested==1000000 || requested==2000000 || requested==3000000) {
                if(requested!=current_baud) {
                    struct uart_config cfg={.baudrate=requested,.parity=UART_CFG_PARITY_NONE,
                        .stop_bits=UART_CFG_STOP_BITS_1,.data_bits=UART_CFG_DATA_BITS_8,.flow_ctrl=UART_CFG_FLOW_CTRL_NONE};
                    uart_irq_rx_disable(uart);
                    if(!uart_configure(uart,&cfg))current_baud=requested;
                    uart_irq_rx_enable(uart);
                }
            }
            key=k_spin_lock(&sample_lock);available=false;memset(stamps,0,sizeof(stamps));k_spin_unlock(&sample_lock,key);
            k_msgq_purge(&gello_rx);k_sleep(K_MSEC(2));continue;
        }
        /* A diagnostic baud cannot accidentally become an active TX baud. */
        if(current_baud!=1000000) {
            struct uart_config cfg={.baudrate=1000000,.parity=UART_CFG_PARITY_NONE,
                .stop_bits=UART_CFG_STOP_BITS_1,.data_bits=UART_CFG_DATA_BITS_8,.flow_ctrl=UART_CFG_FLOW_CTRL_NONE};
            uart_irq_rx_disable(uart);int ret=uart_configure(uart,&cfg);uart_irq_rx_enable(uart);
            if(ret) { k_sleep(K_MSEC(10));continue; }
            current_baud=1000000;
        }
        key=k_spin_lock(&sample_lock);available=true;k_spin_unlock(&sample_lock,key);
#endif
        uint64_t start=robot_us();
        unsigned count=IS_ENABLED(CONFIG_ROBOT_GELLO_DUAL)?14:7;
        for(unsigned step=0;step<count;step++) {
            /* Interleave groups so a missing second master cannot starve the first. */
            unsigned index=count==14 ? (step/2)+(step%2)*7 : step;
            uint8_t id=gello_servo_id(index);
            if(!read_one(id)) {
                /* A confirmed large step needs a second physical read. */
                key=k_spin_lock(&sample_lock);bool retry=trackers[index].pending;k_spin_unlock(&sample_lock,key);
                if(retry)(void)read_one(id);
            }
        }
        key=k_spin_lock(&sample_lock);scans++;k_spin_unlock(&sample_lock,key);
        uint64_t now=robot_us();
        if(now-start<5000)k_usleep(5000-(now-start));else k_usleep(100);
    }
}
static size_t encode(uint8_t out[REPLY_SIZE],const uint8_t nonce[4],bool dual) {
    k_spinlock_key_t key=k_spin_lock(&robot_lock);uint64_t boot=robot.boot;k_spin_unlock(&robot_lock,key);
    struct gello_wire_sample samples[GELLO_SERVO_COUNT];
    key=k_spin_lock(&sample_lock);uint64_t now=robot_us();
    for(unsigned i=0;i<GELLO_SERVO_COUNT;i++) {
        uint64_t age=stamps[i] && now>=stamps[i]?now-stamps[i]:UINT32_MAX;
        samples[i]=(struct gello_wire_sample){.raw=trackers[i].raw,.velocity=velocities[i],
            .ticks=trackers[i].ticks,.age_us=age>UINT32_MAX?UINT32_MAX:(uint32_t)age,.valid=trackers[i].valid};
    }
    uint32_t counters[5]={replies,timeouts,bad,jumps,(uint32_t)atomic_get(&overflow)};
    size_t size=gello_encode(out,dual,sys_get_le32(nonce),boot,scans,now,CONFIG_ROBOT_NODE_ID,available,samples,counters);
    k_spin_unlock(&sample_lock,key);return size;
}
static void query_task(void *a,void *b,void *c) {
    ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
    struct sockaddr_in addr={.sin_family=AF_INET,.sin_port=htons(GELLO_PORT)};
    struct in_addr host;zsock_inet_pton(AF_INET,CONFIG_NET_CONFIG_PEER_IPV4_ADDR,&host);
    int fd=zsock_socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    if(fd<0)return;
    if(zsock_bind(fd,(struct sockaddr*)&addr,sizeof(addr))) { zsock_close(fd);return; }
    uint64_t last=0;
    for(;;) {
        uint8_t input[9],reply[REPLY_SIZE];struct sockaddr_in from;socklen_t nfrom=sizeof(from);
        int n=zsock_recvfrom(fd,input,sizeof(input),0,(struct sockaddr*)&from,&nfrom);
        if(n<0) { k_sleep(K_MSEC(10));continue; }
        uint64_t now=robot_us();
        if(n!=8 || (memcmp(input,"GQ01",4) && memcmp(input,"GQ02",4)) || from.sin_addr.s_addr!=host.s_addr || now-last<2000)continue;
        last=now;size_t size=encode(reply,input+4,!memcmp(input,"GQ02",4));
        (void)zsock_sendto(fd,reply,size,ZSOCK_MSG_DONTWAIT,(struct sockaddr*)&from,nfrom);
    }
}
K_THREAD_DEFINE(gello_collect_thread,3072,collect,NULL,NULL,NULL,7,0,0);
K_THREAD_DEFINE(gello_query_thread,3072,query_task,NULL,NULL,NULL,8,0,0);
