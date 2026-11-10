/* One Waveshare CH343 CDC ACM adapter on J11. Only Feetech READ is sent.
 * All USB waits run below the arm/CAN threads; none owns robot_lock.
 * USB device lifetime is protected against unplug during a transfer by
 * link_lock, which the class removal hook takes before allowing free(). */
#include "gello_usb.h"
#include "gello_usb_desc.h"
#include <zephyr/usb/usbh.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include "usbh_device.h"
#include <errno.h>
#include <string.h>

USBH_CONTROLLER_DEFINE(gello_host,DEVICE_DT_GET(DT_NODELABEL(zephyr_uhc0)));
K_MUTEX_DEFINE(link_lock);
K_SEM_DEFINE(transfer_done,0,1);
static struct usb_device *link_device;
static struct gello_usb_endpoints endpoints;
static atomic_t link_ready;
/* Read-only SWD diagnostics: 0=not started, 1=host ready, 2=enumerating,
 * 3=CDC ready, 4=removed; negative values are errno. */
volatile int32_t gello_usb_state;
volatile uint32_t gello_usb_vid_pid,gello_usb_rx_bytes,gello_usb_tx_bytes;
volatile uint32_t gello_usb_errors,gello_usb_timeouts,gello_usb_connections;

static int transfer_callback(struct usb_device *const udev,struct uhc_transfer *const xfer)
{
    ARG_UNUSED(udev);ARG_UNUSED(xfer);
    k_sem_give(&transfer_done);
    return 0;
}

/* Caller holds link_lock. On cancellation never release DMA storage before
 * completion. If the controller cannot cancel, only this low-priority
 * collector waits; stale samples naturally expire in the existing reader. */
static int transfer(struct usb_device *dev,uint8_t ep,const uint8_t *out,size_t size,
                    uint8_t *in,k_timeout_t timeout,const uint8_t *setup)
{
    struct uhc_transfer *x=usbh_xfer_alloc(dev,ep,transfer_callback,NULL);
    if(!x)return -ENOMEM;
    struct net_buf *b=size?usbh_xfer_buf_alloc(dev,size):NULL;
    if(size && !b) { usbh_xfer_free(dev,x);return -ENOMEM; }
    int ret=b?usbh_xfer_buf_add(dev,x,b):0;
    if(ret)goto free;
    if(out)net_buf_add_mem(b,out,size);
    if(setup)memcpy(x->setup_pkt,setup,8);
    k_sem_reset(&transfer_done);
    ret=usbh_xfer_enqueue(dev,x);
    if(ret) {
        if(x->queued) {
            atomic_clear(&link_ready);gello_usb_state=ret;
            (void)usbh_xfer_dequeue(dev,x);
            k_sem_take(&transfer_done,K_FOREVER);
        }
        goto free;
    }
    if(k_sem_take(&transfer_done,timeout)) {
        gello_usb_timeouts++;
        ret=usbh_xfer_dequeue(dev,x);
        if(ret) { atomic_clear(&link_ready);gello_usb_state=ret; }
        /* Cancellation completion is delivered by the independent USB
         * completion thread. Do not free a buffer while hardware owns it. */
        k_sem_take(&transfer_done,K_FOREVER);
        ret=-ETIMEDOUT;
        goto free;
    }
    ret=x->err;
    if(!ret) {
        if(in) {
            if(b->len>size) { ret=-EOVERFLOW;goto free; }
            memcpy(in,b->data,b->len);gello_usb_rx_bytes+=b->len;
            ret=b->len;
        } else { gello_usb_tx_bytes+=size;ret=(int)size; }
    }
free:
    usbh_xfer_free(dev,x);if(b)usbh_xfer_buf_free(dev,b);
    if(ret<0 && ret!=-ETIMEDOUT)gello_usb_errors++;
    return ret;
}

static int class_init(struct usbh_class_data *const data)
{
    ARG_UNUSED(data);return 0;
}
static int class_probe(struct usbh_class_data *const data,struct usb_device *const dev,
                       const uint8_t iface)
{
    ARG_UNUSED(data);
    /* Match the whole single-serial WCH device, not an arbitrary robot USB
     * peripheral. Actual CDC interfaces/endpoints are parsed, not assumed. */
    if(iface!=255 || dev->dev_desc.idVendor!=0x1a86)return -ENOTSUP;
    const uint8_t *cfg=dev->cfg_desc;
    struct gello_usb_endpoints found;
    gello_usb_vid_pid=((uint32_t)dev->dev_desc.idVendor<<16)|dev->dev_desc.idProduct;
    if(!cfg || !gello_usb_describe(cfg,sys_get_le16(cfg+2),&found)) {
        gello_usb_state=-ENOTSUP;
        printk("GELLO USB %04x:%04x has no single CDC ACM function\n",
               dev->dev_desc.idVendor,dev->dev_desc.idProduct);
        return -ENOTSUP;
    }
    gello_usb_state=2;
    /* CDC SET_LINE_CODING: 1,000,000 bps, one stop, no parity, eight bits. */
    const uint8_t coding[7]={0x40,0x42,0x0f,0,0,0,8};
    const uint8_t set_line[8]={0x21,0x20,0,0,found.control_iface,0,7,0};
    const uint8_t set_control[8]={0x21,0x22,3,0,found.control_iface,0,0,0};
    k_mutex_lock(&link_lock,K_FOREVER);
    int ret=transfer(dev,0,coding,sizeof(coding),NULL,K_MSEC(500),set_line);
    if(ret==7)ret=transfer(dev,0,NULL,0,NULL,K_MSEC(500),set_control);
    if(ret) {
        gello_usb_state=ret;
        printk("GELLO USB CDC setup failed: %d\n",ret);
        k_mutex_unlock(&link_lock);
        return -ENOTSUP;
    }
    link_device=dev;endpoints=found;atomic_set(&link_ready,1);
    gello_usb_state=3;gello_usb_connections++;
    k_mutex_unlock(&link_lock);
    printk("GELLO USB J11 %04x:%04x CDC %u/%u EP %02x/%02x 1Mbps read-only\n",
           dev->dev_desc.idVendor,dev->dev_desc.idProduct,
           found.control_iface,found.data_iface,found.in,found.out);
    return 0;
}
static int class_removed(struct usbh_class_data *const data)
{
    ARG_UNUSED(data);
    atomic_clear(&link_ready);
    k_mutex_lock(&link_lock,K_FOREVER);
    link_device=NULL;gello_usb_state=4;
    k_mutex_unlock(&link_lock);
    printk("GELLO USB removed\n");return 0;
}
static struct usbh_class_api gello_class_api={
    .init=class_init,.probe=class_probe,.removed=class_removed,
};
USBH_DEFINE_CLASS(gello_cdc,&gello_class_api,NULL,NULL);

static void lower_usb_priority(const struct k_thread *thread,void *unused)
{
    ARG_UNUSED(unused);
    const char *name=k_thread_name_get((k_tid_t)thread);
    if(name && (!strcmp(name,"usbh") || !strcmp(name,"usbh_bus") ||
                !strcmp(name,"uhc_mcux_ehci")))
        k_thread_priority_set((k_tid_t)thread,6);
}
int gello_usb_start(void)
{
    int ret=usbh_init(&gello_host);
    if(!ret) {
        /* Host defaults are cooperative; robot control must preempt them. */
        k_thread_foreach_unlocked(lower_usb_priority,NULL);
        gello_usb_state=1;
        ret=usbh_enable(&gello_host);
    }
    if(ret)gello_usb_state=ret;
    return ret;
}
bool gello_usb_ready(void) { return atomic_get(&link_ready)!=0; }

bool gello_usb_read(uint8_t id,struct gello_sample *sample,uint32_t *bad)
{
    bool ok=false;
    struct gello_parser parser={0};
    if(!sample || !bad || gello_servo_index(id)<0)return false;
    k_mutex_lock(&link_lock,K_FOREVER);
    struct usb_device *dev=link_device;
    if(!dev || !gello_usb_ready())goto out;
    uint8_t request[8];gello_read_request(id,request);
    if(transfer(dev,endpoints.out,request,sizeof(request),NULL,K_MSEC(20),NULL)!=8)goto out;
    int64_t deadline=k_uptime_get()+8;
    while(gello_usb_ready() && k_uptime_get()<deadline) {
        uint8_t bytes[512];
        int n=transfer(dev,endpoints.in,NULL,endpoints.in_mps,bytes,K_MSEC(2),NULL);
        if(n<0)break;
        for(int i=0;i<n;i++) {
            if(gello_parse(&parser,bytes[i],sample) && sample->id==id) { ok=true;break; }
        }
        if(ok)break;
    }
out:
    *bad=parser.bad;k_mutex_unlock(&link_lock);return ok;
}
