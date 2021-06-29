BEGIN{
       indx=0
       node=0
       throughput=0
       goodput=0
       FCT=0
       failure=0
       w=0
}

{
   if($12=="w3="){
	   w=$13
           node=$5
	}
   if($9=="index="){
	   indx=$10
	}
   if($2=="flows"){
	   failure=$6-$1
	}
   if($2=="throughput:"){
           throughput=$3
           goodput=$7
           FCT=$11
        }
   if($1=="##########################################"){
           print indx"   "w"   "node "  "throughput"        " goodput"        " failure"   " FCT
        } 
}

END{

}
