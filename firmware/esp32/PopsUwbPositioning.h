#pragma once
#include <Arduino.h>
#include <math.h>

// Independent of BU04 transport. Distances and coordinates are METERS.
// Four anchor observations plus the resulting mower position are the five
// information records passed through the positioning pipeline.
namespace PopsUWB {
constexpr int N = 4;
struct Anchor { float x,y,z; float bias=0; float scale=1; };
struct Range { float meters=NAN; uint32_t timeMs=0; bool valid=false; };
struct Fix { float x=NAN,y=NAN,rms=NAN; uint8_t used=0; bool valid=false; };
struct Calibration { float offset=0, scale=1; }; // corrected = raw*scale + offset

inline bool calibrate(const float* raw,const float* truth,int n,Calibration& out) {
  if(n<2) return false;
  double sx=0,sy=0,sxx=0,sxy=0;
  for(int i=0;i<n;i++) {
    if(!isfinite(raw[i])||!isfinite(truth[i])||raw[i]<=0||truth[i]<=0) return false;
    sx+=raw[i]; sy+=truth[i]; sxx+=raw[i]*raw[i]; sxy+=raw[i]*truth[i];
  }
  double den=n*sxx-sx*sx;
  if(fabs(den)<1e-9) return false;
  double scale=(n*sxy-sx*sy)/den, offset=(sy-scale*sx)/n;
  if(!isfinite(scale)||scale<0.8||scale>1.2||fabs(offset)>2.0) return false;
  out.scale=scale;out.offset=offset;return true;
}

// Solve horizontal X/Y for a tag at known height tagZ.
// Uses pairwise linear seed and iterative robust nonlinear least squares.
// No heap allocation. At least three fresh observations required.
inline Fix solve(const Anchor a[N],const Range r[N],float tagZ,
                 uint32_t nowMs,uint32_t maxAgeMs=500) {
  Fix out;
  int ids[N],m=0;
  float d[N];
  for(int i=0;i<N;i++){
    if(!r[i].valid||!isfinite(r[i].meters)||r[i].meters<=0 ||
       (uint32_t)(nowMs-r[i].timeMs)>maxAgeMs) continue;
    d[i]=r[i].meters*a[i].scale+a[i].bias;
    if(!isfinite(d[i])||d[i]<=0)continue;
    ids[m++]=i;
  }
  out.used=m;
  if(m<3)return out;
  // Linearized equations relative to first valid anchor.
  int k=ids[0]; double aa=0,ab=0,bb=0,ac=0,bc=0;
  double zk=tagZ-a[k].z;
  double qk=d[k]*d[k]-zk*zk;
  for(int j=1;j<m;j++){
    int i=ids[j];
    double zi=tagZ-a[i].z,qi=d[i]*d[i]-zi*zi;
    double u=2*(a[i].x-a[k].x),v=2*(a[i].y-a[k].y);
    double w=qk-qi+a[i].x*a[i].x-a[k].x*a[k].x+
             a[i].y*a[i].y-a[k].y*a[k].y;
    aa+=u*u;ab+=u*v;bb+=v*v;ac+=u*w;bc+=v*w;
  }
  double det=aa*bb-ab*ab;
  if(fabs(det)<1e-8)return out;
  double x=(ac*bb-bc*ab)/det,y=(aa*bc-ab*ac)/det;
  // Huber weighted Gauss-Newton, 8 iterations.
  for(int iteration=0;iteration<8;iteration++){
    double h00=0,h01=0,h11=0,g0=0,g1=0;
    for(int j=0;j<m;j++){
      int i=ids[j];
      double dx=x-a[i].x,dy=y-a[i].y,dz=tagZ-a[i].z;
      double pred=sqrt(dx*dx+dy*dy+dz*dz);
      if(pred<1e-6)return out;
      double err=pred-d[i];
      double weight=fabs(err)<=0.25?1.0:0.25/fabs(err);
      double jx=dx/pred,jy=dy/pred;
      h00+=weight*jx*jx;h01+=weight*jx*jy;h11+=weight*jy*jy;
      g0+=weight*jx*err;g1+=weight*jy*err;
    }
    double hdet=h00*h11-h01*h01;
    if(fabs(hdet)<1e-10)return out;
    double sx=(g0*h11-g1*h01)/hdet;
    double sy=(h00*g1-h01*g0)/hdet;
    x-=sx;y-=sy;
    if(sx*sx+sy*sy<1e-8)break;
  }
  double ss=0;
  for(int j=0;j<m;j++){
    int i=ids[j];
    double dx=x-a[i].x,dy=y-a[i].y,dz=tagZ-a[i].z;
    double err=sqrt(dx*dx+dy*dy+dz*dz)-d[i];
    ss+=err*err;
  }
  out.x=x;out.y=y;out.rms=sqrt(ss/m);
  out.valid=isfinite(x)&&isfinite(y)&&isfinite(out.rms)&&out.rms<=0.30;
  return out;
}
} // namespace PopsUWB
