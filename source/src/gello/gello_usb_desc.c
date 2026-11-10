#include "gello_usb_desc.h"
#include <string.h>
static uint16_t le16(const uint8_t *p) { return p[0] | (uint16_t)p[1]<<8; }
bool gello_usb_describe(const uint8_t *p,size_t n,struct gello_usb_endpoints *ep)
{
    if(!p || !ep || n<9 || p[0]!=9 || p[1]!=2 || le16(p+2)!=n)return false;
    struct gello_usb_endpoints e={0};
    unsigned control_count=0;bool union_found=false,data_found=false;
    uint8_t iface=255,alt=255,cls=255;
    for(size_t i=9;i<n;) {
        const uint8_t *d=p+i;size_t len=d[0];
        if(n-i<2 || len<2 || len>n-i)return false;
        if(d[1]==4) {
            if(len<9)return false;
            iface=d[2];alt=d[3];cls=d[5];
            if(!alt && cls==2 && d[6]==2) { e.control_iface=iface;control_count++; }
        } else if(d[1]==0x24 && len>=3 && d[2]==6 && !alt && cls==2) {
            if(len!=5 || d[3]!=iface || union_found)return false;
            e.data_iface=d[4];union_found=true;
        }
        i+=len;
    }
    if(control_count!=1 || !union_found || e.control_iface==e.data_iface)return false;
    iface=alt=cls=255;
    for(size_t i=9;i<n;) {
        const uint8_t *d=p+i;size_t len=d[0];
        if(d[1]==4) {
            iface=d[2];alt=d[3];cls=d[5];
            if(!alt && iface==e.data_iface) {
                if(cls!=10 || data_found)return false;
                data_found=true;
            }
        } else if(d[1]==5 && iface==e.data_iface && !alt) {
            if(len<7)return false;
            if((d[3]&3)!=2 || !(d[2]&15) || (d[2]&0x70))return false;
            uint16_t mps=le16(d+4);
            if(mps!=8 && mps!=16 && mps!=32 && mps!=64 && mps!=512)return false;
            if(d[2]&0x80) { if(e.in)return false;e.in=d[2];e.in_mps=mps; }
            else { if(e.out)return false;e.out=d[2]; }
        }
        i+=len;
    }
    if(!data_found || !e.in || !e.out)return false;
    *ep=e;return true;
}
