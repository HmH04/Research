from LE__SDK__ import * 

print ' ' 
print '#####################################################'
print '## LaserEye (STUD LIDAR Prototype IV) Program      ##'
print '##  * LE_checkPCD (2018/10/18)                     ##'
print_SDK_VER()
print '##  * designed and programmed by STUD LIDAR Team.  ##'
print '##  * Copyright 2018 ETRI. All rights reserved.    ##'
print '#####################################################'

import socket
import sys
import cv2
import numpy as np 
import struct as st
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import struct
import csv

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.getsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF)
opt = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO, 30) 
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO, struct.pack('LL', LE_timeout, 0))
opt = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO, 30)

server_address = ('', 8888)
print >>sys.stderr, 'starting up on %s port %s' % server_address
sock.bind(server_address)

print ' '

############################################################################

# Number of Packet
##num_to_read_UDP=2200
num_to_read_UDP=int(sys.argv[1])
messlist=[]


try:
    for x in xrange(num_to_read_UDP):
        data, addr = sock.recvfrom(2048)
        messlist.append(data)
        pend=st.unpack('HHH', data[1200:1206])
        #print '(Cont, %4d B)|'%(len(data)), "pend[0]= %d, pend[1]= %d, pend[2]= %d"%(pend[0], pend[1], pend[2])
        len(messlist)
        
except KeyboardInterrupt:
    print '\n\n--- Exit by CTRL+C ---\n\n'
    sock.close() 

print 'Binding Complete -- '

print ' '

##

p12xlist=[] # X
p12ylist=[] # Y
p12rawlist=[] # 12 raw data starting from the pixel cordinate (p12xlist, p12ylist)

CIDlist=[] # UPD ID in a frame 
UIDlist=[] # UDP count 
FCDlist=[] # Factory Coded Data 

pUDPindex=0

'''
for p in range(0, len(messlist)):
#if len(data)==1206:
    p12x=np.array([st.unpack('H', messlist[p][(0+i*100):(2+i*100)]) for i in range(12)])
    p12y=np.array([st.unpack('H', messlist[p][(2+i*100):(4+i*100)]) for i in range(12)])
    p12raw=np.array([st.unpack('=HBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHB',
                                    messlist[p][(4+i*100):(100+i*100)]) for i in range(12)])
    CID=st.unpack('H', messlist[p][1200:1202])
    UID=st.unpack('H', messlist[p][1202:1204])
    FCD=st.unpack('H', messlist[p][1204:1206])
        
    p12xlist.append(p12x)
    p12ylist.append(p12y)
    p12rawlist.append(p12raw)
    
    CIDlist.append(CID)
    UIDlist.append(UID)
    FCDlist.append(FCD)
    
    #fnum+=1
    ####print 'Parsing %dth UDP, %dth UDP in a frame, starting at X,Y(%d,%d)'%(CID[0], UID[0],p12x[0],p12y[0])
'''

##

# for each frame 
hlist=[] # frame X,Y head list
dlist=[] # frame Poinmt Cloud data list
ulist=[] # UDP tag list
tlist=[] 

# Loop 
fnum=0
kkk=0
q=0
#32list=[] # point list in a single frame
p32head=[] # head list in a single frame
p32data=[] # point cloud data list in a single frame
p32utag=[] # UDP tag list
p32tdata=[]

OnlyData=[]

p32p12x=[]
p32p12y=[]

#ptimerawlist=[]


ptttlist=[]

pUDPindex=-1

Rptlist=[]

print ' Go to the Data Capture '
print ' '

for p in range(0, len(messlist)):
    p12x=np.array([st.unpack('H', messlist[p][(0+i*100):(2+i*100)]) for i in range(12)])
    p12y=np.array([st.unpack('H', messlist[p][(2+i*100):(4+i*100)]) for i in range(12)])
    p12raw=np.array([st.unpack('=HBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHBHB',
                                    messlist[p][(4+i*100):(100+i*100)]) for i in range(12)])
    tail=st.unpack('HHH', messlist[p][1200:1206])
    #print '(Cont, %4d B)|'%(len(data)), "tail[0]= %d, tail[1]= %d, tail[2]= %d"%(tail[0], tail[1], tail[2])
    
    if tail[0] < pUDPindex:
        hlist.append(p32head)
        dlist.append(p32data)
        ulist.append(p32utag)
         #tlist.append(p32tdata)
        print 'the end of ',fnum,'th frame: UDP count=', len(ulist[fnum]),'dlist size=', np.array(dlist[fnum]).shape
        
        if len(ulist[fnum])==200:
            for u in range(0,len(ulist[fnum])):
                p32data[u]
                for i in range(0,12):
                    Tdata=p32data[u]
                    for k in range(0,32):
                        pttt=Tdata[i][0+k*2]
                        ptttlist.append(pttt)


            print 'Only Time Data length',len(ptttlist)
            Rptlist.append(ptttlist)
            print len(Rptlist)
            dtable = Rptlist
            with open(sys.argv[2],"w+") as my_csv:
            #with open("LE_XX.csv","w+") as my_csv:
                csvWriter = csv.writer(my_csv, delimiter=',')
                csvWriter.writerows(np.array(dtable).T)
            ptttlist=[] # Before Data Clear

        fnum+=1 # next frame
        p32head=[[p12x,p12y]]
        p32data=[p12raw]
        p32utag=[tail]        
    else:
        p32head.append([p12x,p12y])
        p32data.append(p12raw)
        p32utag.append(tail) 
           
    pUDPindex=tail[0] # frame count
    

dlist.append(p32data)
hlist.append(p32head)
ulist.append(p32utag)
# tlist.append(p32tdata)

print 'the end of ',fnum,'th frame: UDP count=', len(ulist[fnum]),'dlist size=', np.array(dlist[fnum]).shape





