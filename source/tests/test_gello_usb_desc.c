#include "gello/gello_usb_desc.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
/* Non-default interface and endpoint numbers deliberately exercise binding
 * by descriptors, not hard-coded CH343 endpoint assumptions. */
static const uint8_t fixture[]={
 9,2,62,0,2,1,0,0x80,50,
 9,4,2,0,1,2,2,1,0,
 5,0x24,0,0x10,1,
 4,0x24,2,2,
 5,0x24,6,2,3,
 7,5,0x83,3,8,0,16,
 9,4,3,0,2,10,0,0,0,
 7,5,0x84,2,64,0,0,
 7,5,0x05,2,64,0,0,
};
int main(void)
{
    uint8_t b[sizeof(fixture)];memcpy(b,fixture,sizeof(b));
    b[2]=sizeof(b);struct gello_usb_endpoints ep;
    assert(gello_usb_describe(b,sizeof(b),&ep));
    assert(ep.control_iface==2 && ep.data_iface==3 && ep.in==0x84 && ep.out==5 && ep.in_mps==64);
    for(size_t n=0;n<sizeof(b);n++)assert(!gello_usb_describe(b,n,&ep));
    uint8_t original[sizeof(b)];memcpy(original,b,sizeof(b));
    const size_t lengths[]={9,18,23,27,32,39,48,55};
    for(size_t i=0;i<sizeof(lengths)/sizeof(lengths[0]);i++) {
        memcpy(b,original,sizeof(b));b[lengths[i]]=0;
        assert(!gello_usb_describe(b,sizeof(b),&ep));
    }
    memcpy(b,original,sizeof(b));b[31]=4;assert(!gello_usb_describe(b,sizeof(b),&ep));
    memcpy(b,original,sizeof(b));b[44]=3;assert(!gello_usb_describe(b,sizeof(b),&ep));
    memcpy(b,original,sizeof(b));b[50]=0x80;assert(!gello_usb_describe(b,sizeof(b),&ep));
    memcpy(b,original,sizeof(b));b[57]=0x85;assert(!gello_usb_describe(b,sizeof(b),&ep));
    memcpy(b,original,sizeof(b));b[52]=0;assert(!gello_usb_describe(b,sizeof(b),&ep));
    puts("USB CDC descriptor binding/truncation/malformed/wrong-interface/endpoint tests passed");
}
