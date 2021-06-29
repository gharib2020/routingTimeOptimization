BEGIN{
       avgThroughput[0]=0
       avgFCT[0]=0
       avgSuccess[0]=0
       str=""
       indx=0
       conNo=0
       w1=0
       throughput[0][0]=0
       goodput[0][0]=0
       FCT[0][0]=0
       failure[0][0]=0
for (i = 0; i <= 1; i+=0.1)
        for (j = 1; j <= 10; j++)
            { 
	        throughput[i][j]=0
      		goodput[i][j]=0
       		FCT[i][j]=0
       		failure[i][j]=0
            }
}

{ 
   if($12=="w1="){
	   w1=$13
           conNo=$5
	}
   if($9=="index="){
	   indx=$10
	}
   if($2=="flows"){
	   failure[w1][indx]=$6-$1
	}
   if($2=="throughput:"){
           throughput[w1][indx]=$3
           goodput[w1][indx]=$7
           if($11!="-nan")
             {
               FCT[w1][indx]=(($11*(conNo-failure[w1][indx]))+500*failure[w1][indx])/conNo           
            }
          else
            {
              FCT[w1][indx]=500
            }
        }
   if($1=="##########################################"){
           
        } 
}

END{

print "********************************"
print "*  no. of flows:"conNo"            *"
print "********************************"
for (i = 0; i <= 1; i+=0.1)
      {
       avgThroughput[i]=0
       avgFCT[i]=0
       avgSuccess[i]=0
        for (j = 0; j <= 19; j++)
            {
              avgThroughput[i]+=throughput[i][j]
              avgFCT[i]+=FCT[i][j]
              avgSuccess[i]+=conNo-failure[i][j]
            }
      }

for (i = 0; i <= 1; i+=0.1)
      {
        print avgThroughput[i]/20
      }
print "***********"

for (i = 0; i <= 1; i+=0.1)
      {
         print avgFCT[i]/20
      }
print "***********"

for (i = 0; i <= 1; i+=0.1)
      {
        print(avgSuccess[i])/(20*conNo)
      }
print "\n"
}
