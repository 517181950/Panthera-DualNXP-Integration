#include "homing.h"
#include "model_data.h"
#include <math.h>
#include <string.h>
static void mv(const float r[3][3],const float v[3],float o[3]) {
    for(int i=0;i<3;i++) o[i]=r[i][0]*v[0]+r[i][1]*v[1]+r[i][2]*v[2];
}
void home_gravity(const float q[6],float tau[6]) {
    float r[3][3]={{1,0,0},{0,1,0},{0,0,1}},p[3]={0};
    float origin[6][3],axis[6][3],weighted[6][3];
    for(int j=0;j<6;j++) {
        float shift[3],v[3],rot[3][3],next[3][3];
        mv(r,home_xyz[j],shift); mv(r,home_axis[j],axis[j]);
        for(int i=0;i<3;i++) origin[j][i]=(p[i]+=shift[i]);
        const float *a=home_axis[j]; float c=cosf(q[j]),s=sinf(q[j]);
        float skew[3][3]={{0,-a[2],a[1]},{a[2],0,-a[0]},{-a[1],a[0],0}};
        for(int i=0;i<3;i++) for(int k=0;k<3;k++) rot[i][k]=(i==k?c:0)+(1-c)*a[i]*a[k]+s*skew[i][k];
        for(int i=0;i<3;i++) for(int k=0;k<3;k++) {
            next[i][k]=0; for(int t=0;t<3;t++) next[i][k]+=r[i][t]*rot[t][k];
        }
        memcpy(r,next,sizeof(r)); mv(r,&home_theta[j][1],v);
        for(int i=0;i<3;i++) weighted[j][i]=home_theta[j][0]*p[i]+v[i];
    }
    for(int j=0;j<6;j++) {
        tau[j]=0;
        for(int k=j;k<6;k++) {
            float x=weighted[k][0]-home_theta[k][0]*origin[j][0];
            float y=weighted[k][1]-home_theta[k][0]*origin[j][1];
            tau[j]+=9.81f*(axis[j][0]*y-axis[j][1]*x);
        }
    }
}
