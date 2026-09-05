# Two-body + J2-secular propagation from TLE mean elements; az/el/Doppler for an observer.
import math, sys, re
from datetime import datetime, timezone, timedelta
MU=398600.4418; RE=6378.137; J2=1.08262668e-3; C=299792458.0
def parse(fn):
    L=open(fn).read().splitlines(); out=[]
    for k in range(0,len(L)-2,3):
        name,l1,l2=L[k].strip(),L[k+1],L[k+2]
        yy=int(l1[18:20]); doy=float(l1[20:32])
        epoch=datetime(2000+yy,1,1,tzinfo=timezone.utc)+timedelta(days=doy-1)
        inc=math.radians(float(l2[8:16])); raan=math.radians(float(l2[17:25]))
        ecc=float("0."+l2[26:33].strip()); argp=math.radians(float(l2[34:42]))
        M=math.radians(float(l2[43:51])); n=float(l2[52:63])*2*math.pi/86400
        out.append((name,epoch,inc,raan,ecc,argp,M,n))
    return out
def gmst(t):
    jd=2451545.0+(t-datetime(2000,1,1,12,tzinfo=timezone.utc)).total_seconds()/86400
    T=(jd-2451545.0)/36525
    g=280.46061837+360.98564736629*(jd-2451545.0)+0.000387933*T*T
    return math.radians(g%360)
def ecef(sat,t):
    name,epoch,inc,raan,ecc,argp,M0,n=sat
    dt=(t-epoch).total_seconds()
    a=(MU/n**2)**(1/3); p=a*(1-ecc**2)
    # J2 secular
    fac=1.5*J2*(RE/p)**2*n
    raan_dot=-fac*math.cos(inc); argp_dot=fac*(2-2.5*math.sin(inc)**2)
    M=M0+n*dt; Om=raan+raan_dot*dt; w=argp+argp_dot*dt
    E=M
    for _ in range(30): E=E-(E-ecc*math.sin(E)-M)/(1-ecc*math.cos(E))
    nu=2*math.atan2(math.sqrt(1+ecc)*math.sin(E/2),math.sqrt(1-ecc)*math.cos(E/2))
    r=a*(1-ecc*math.cos(E))
    xo,yo=r*math.cos(nu),r*math.sin(nu)
    cw,sw,cO,sO,ci,si=math.cos(w),math.sin(w),math.cos(Om),math.sin(Om),math.cos(inc),math.sin(inc)
    x=(cw*cO-sw*sO*ci)*xo+(-sw*cO-cw*sO*ci)*yo
    y=(cw*sO+sw*cO*ci)*xo+(-sw*sO+cw*cO*ci)*yo
    z=(sw*si)*xo+(cw*si)*yo
    g=gmst(t)
    return (x*math.cos(g)+y*math.sin(g), -x*math.sin(g)+y*math.cos(g), z)
def obs_ecef(lat,lon,h):
    lat,lon=math.radians(lat),math.radians(lon); f=1/298.257223563; e2=f*(2-f)
    N=RE/math.sqrt(1-e2*math.sin(lat)**2)
    return ((N+h)*math.cos(lat)*math.cos(lon),(N+h)*math.cos(lat)*math.sin(lon),(N*(1-e2)+h)*math.sin(lat)),lat,lon
def azel(sat,t,o,lat,lon):
    s=ecef(sat,t); d=[s[i]-o[i] for i in range(3)]
    sl,cl,sn,cn=math.sin(lat),math.cos(lat),math.sin(lon),math.cos(lon)
    e=-sn*d[0]+cn*d[1]; nn=-sl*cn*d[0]-sl*sn*d[1]+cl*d[2]; u=cl*cn*d[0]+cl*sn*d[1]+sl*d[2]
    rng=math.sqrt(sum(v*v for v in d))
    return math.degrees(math.atan2(e,nn))%360, math.degrees(math.asin(u/rng)), rng
def main():
    tfile,tstr,fc=sys.argv[1],sys.argv[2],float(sys.argv[3])
    t=datetime.fromisoformat(tstr.replace("Z","+00:00"))
    lat,lon,h=50.7686,6.0727,0.271
    o,la,lo=obs_ecef(lat,lon,h)
    rows=[]
    for sat in parse(tfile):
        az,el,r0=azel(sat,t,o,la,lo)
        _,_,r1=azel(sat,t+timedelta(seconds=1),o,la,lo)
        dop=-(r1-r0)*1000/C*fc
        m=re.search(r"PRN (E?\d+)",sat[0]); prn=m.group(1) if m else sat[0]
        rows.append((el,prn,az,dop,sat[0]))
    rows.sort(reverse=True)
    print(f"{'PRN':>5} {'el':>6} {'az':>6} {'doppler':>9}  name")
    for el,prn,az,dop,name in rows:
        if el>-5: print(f"{prn:>5} {el:6.1f} {az:6.1f} {dop:9.0f}  {name}")
main()
