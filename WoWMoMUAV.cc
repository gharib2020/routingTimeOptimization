/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mohammed Gharib <mohammed.algharib@nau.edu>
 *
 */

//
// This example simulate an UAV network, with nWifi UAV agents.
// Agents are randomly positioned in an X*Y*Z area and moves 
// according to a chosen mobility model. The mobility models could be:
// 1-RWP, 2-GaussMarkov
// They communicate through a wifi-based ad hoc network. 
// The mobility model has chosen by the parameter protocol and could be:
// 1-OLSR, 2-AODV, 3-DSDV, 4-DSR. 
// Author used manet-routing-compare.cc and tcp-large-transfer.cc
// as its baseline examples and developed his code based on them.

/* 
//  Usage (e.g.): ./waf --run UAVrouting
*/

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <bits/stdc++.h>

#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/wifi-module.h"


using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("infocomWorkshop");

static double txp=7.5;
static double TxRange=138.8;//Transmit range: for txp=7.5 dbm it is 138.8 m
static const uint16_t port = 50000;
static const uint32_t totalRxBytes = 5000000;//5MB
static Ipv4InterfaceContainer adhocInterfaces;
static NetDeviceContainer adhocDevices;
static NodeContainer adhocNodes;
// Perform series of 1040 byte writes (this is a multiple of 26 since
// we want to detect data splicing in the output stream)
static const uint32_t writeSize = 1040;
static uint8_t data[writeSize];
static MobilityHelper mobilityAdhoc;
static int const nWifi=100;//number of UAVs
static Ptr<Socket> mobilityTrackingSocket[nWifi];
static double posMatrix[nWifi][12]={};//each row includes three sets of (time,x,y,z) 
static int traceCount=0;
static double totalTime=1100;
static double pathLenWeight=0.5;// 0 <= pathLenWeight <= 1. pathLenWeight=1 leads to shortest path, pathLenWeight=0 leads to the path with longest lifeTime
static double pathLifetimeWeight=1-pathLenWeight;
static double routCheckTime=0.3;//the rout is checked each routCheckTime second
bool adj[nWifi][nWifi]={};
double updateTime=1;//sec
double warmup=100;//sec

double avgNoOfNeighb=0;
double varNoOfNeighb=0;
double avgConnectivity=0;
double varConnectivity=0;
double physicalConnectivity=0;
bool visited[nWifi][nWifi]={};
double timeToVisitAll[nWifi]={};
bool visitedAll[nWifi]={};
double fullConnectivityTime=0;
bool fullyConnectedNetwork=false;
uint32_t keyPathLength=0;
int keyPathLengthCount=0;

void setupMobility(double, double,double,NodeContainer,uint32_t,int64_t);
YansWifiPhyHelper setupWifiPhy(double);
Ipv4InterfaceContainer setupIP(NetDeviceContainer);
void setupMobilityTrack(NodeContainer);
double distance(int,int);
void traceUAVpositions();
void updateAdj();
void calcNoOfNeigh();
void calcPhyConnectivity();
void calcConnectivity();
void updateTimeToVisitAll();
uint32_t noOfOnes(bool[nWifi]);//returns no. of 1s in an array
bool isConnected(bool[nWifi][nWifi]);//returns true if the network is connected based on given matrix
double connectivityProbability(bool [nWifi][nWifi]);//returns the number of connected nodes per all nodes
void keyPathLen(int, int);//calculates key-path length between the source and the destination
void BFS(bool [nWifi][nWifi],int,int, int *,int [nWifi]);


//###################### main() ###########################
int main (int argc, char *argv[])
{
  int nSinks=0;
  int64_t streamIndex = 0; // used to get consistent mobility across scenarios
  uint32_t mobilityModel=1;//1-3d RWP, 2-3d GaussMarkov, 3-2d RWp, 4- 2d G-M   

  double X=500.0;
  double Y=500.0;
  double Z=100.0;
  
  CommandLine cmd;
  cmd.AddValue("nSinks", "No. of sinks to echo", nSinks);
  cmd.AddValue("streamIndex", "Stream Index to echo", streamIndex);
  cmd.AddValue("totalTime", "Total time to echo", totalTime);
  cmd.AddValue("mobilityModel", "Mobility model to echo (1-3d RWP, 2-3d GaussMarkov, 3-2d RWp, 4- 2d G-M): ", mobilityModel);
  //cmd.AddValue("nWifi","No. of UAVs to echo",nWifi);
  cmd.AddValue("txp","Transmition power to echo",txp);
  cmd.AddValue("X","Area width to echo",X);
  cmd.AddValue("Y","Area Length to echo",Y);
  cmd.AddValue("Z","Area height to echo",Z);
  cmd.AddValue("routCheckTime","routCheckTime to echo",routCheckTime);
  cmd.AddValue("pathLenWeight","Path length weight to echo",pathLenWeight);
  cmd.Parse (argc, argv);

  std::string phyMode ("DsssRate11Mbps");
  const std::string rate="2048bps";

  std::string mobilityName;

switch (mobilityModel)
    {
    case 1:
      mobilityName = "3D-RWP";
      break;
    case 2:
      mobilityName = "3D-G-M";
      break;
    case 3:
      mobilityName = "2D-RWP";
      break;
    case 4:
      mobilityName = "3-D-G-M";
      break;
    default:
      NS_FATAL_ERROR ("No such model:" << mobilityModel);
    }

  std::cout<<"Total time: "<<totalTime<<", no. UAVs: "<<nWifi<<"Trans. Range: "<<TxRange<<", X: "<<X<<", Y: "<<Y<<", Z: "<<Z<<", Mobility model: "<<mobilityName<<", stream Index: "<<streamIndex<<std::endl; 

  std::stringstream ss;
  ss<<"traceFiles/UAV"<<nWifi<<"_"<<mobilityName<< "_"<<streamIndex;
  std::string tr_name (ss.str ());
  
  NS_LOG_UNCOND ("Starting the simulation...");

  // initialize the tx buffer.
  for(uint32_t i = 0; i < writeSize; ++i)
    {
      char m = toascii (97 + i % 26);
      data[i] = m;
    }
    
    ns3::PacketMetadata::Enable ();
  // Here, we will explicitly create the nodes. 
  // This will be used to install the network
  // interfaces and connect them with a channel.
  adhocNodes.Create (nWifi);

  // We create the channels first without any IP addressing information
  // First make and configure the helper, so that it will put the appropriate
  // attributes on the network interfaces and channels we are about to install.
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  //disable rate control  
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));

  //setup wifi physical attributes
  YansWifiPhyHelper wifiPhy; 
  wifiPhy= setupWifiPhy(txp);  
  //NS_LOG_UNCOND ("Setting up wifi physical attributes...");
  //Set Non-unicastMode rate to unicast mode
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (phyMode));

  // Add a mac 
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");  
  
  NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, adhocNodes);

  //setup mobility
  setupMobility(X,Y,Z,adhocNodes,mobilityModel,streamIndex);  
  NS_LOG_UNCOND ("Setting up Mobility...");

  InternetStackHelper internet;
  internet.Install (adhocNodes);
  NS_LOG_UNCOND ("Setting up routing protocol...");
  // Later, we add IP addresses.
  adhocInterfaces=setupIP(adhocDevices);
  NS_LOG_UNCOND ("Setting up IP address for nodes...");

  setupMobilityTrack(adhocNodes);
 
  /////////////////////////////////////////////////////////////////////////// 
  // Send a file of 5000MB over a connection to the sink
  // Should observe SYN exchange, a lot of data segments and ACKS, and FIN 
  // exchange.  FIN exchange isn't quite compliant with TCP spec (see release
  // notes for more info)
  ///////////////////////////////////////////////////////////////////////////

  // Create the connections...

  //Flow UAVflow[nSinks];
  //std::cout<<"Setting up "<<nSinks<<" connections..."<<std::endl;
  
  //for (int i = 0; i < nSinks; i++)
   //  {
     //   UAVflow[i].setupConnection(i*2,i*2+1,i);
     //}
 
  //Ask for ASCII and pcap traces of network traffic
  
  AsciiTraceHelper ascii;
  //Ptr<FlowMonitor> flowmon;

  //wifiPhy.EnablePcapAll (tr_name);
  //wifiPhy.EnableAsciiAll (ascii.CreateFileStream (tr_name+".tr"));
  //MobilityHelper::EnableAsciiAll (ascii.CreateFileStream (tr_name + ".mob"));

  AnimationInterface anim (tr_name+".xml");
  anim.SetMaxPktsPerTraceFile (200000000);

  //FlowMonitorHelper flowmonHelper;
  //flowmon = flowmonHelper.InstallAll (); 
  
  // Finally, set up the simulator to run.  The 'totalTime' second hard limit is a
  // failsafe in case some change above causes the simulation to never end
  Simulator::Stop (Seconds (totalTime));
  Simulator::Run ();
  
  //flowmon->SerializeToXmlFile ((tr_name + ".flowmon").c_str(), true, true);

  NS_LOG_UNCOND ("End of simulation...");

  double density;
  density=(X*Y*Z)/(3.0/4.0*M_PI*pow(TxRange,3));//how many spheres fit the entire space
  density=nWifi/density;//the density of nodes per sphere
  std::cout<<"X: "<<X<<" , Y: "<<Y<<" , Z: "<<Z<<" , density: "<<density<<std::endl;
  std::cout<<"Avg. no. of neighbors: "<<avgNoOfNeighb/((totalTime-warmup)/updateTime)<<" , with the standard deviation: "<<sqrt(varNoOfNeighb/((totalTime-warmup)/updateTime))<<std::endl;
  std::cout<<"Physical connectivity probability: "<<physicalConnectivity/((totalTime-warmup)/updateTime)<<std::endl;
  std::cout<<"Time of full key-connectivity: "<<fullConnectivityTime<<std::endl;
  if (keyPathLengthCount==0)
    std::cout<<"Avg. key-path length: There is no key-path"<<std::endl;
  else 
    std::cout<<"Avg. key-path length: "<<keyPathLength/double(keyPathLengthCount)<<" , There were "<<keyPathLengthCount<<" key-path out of "<<((totalTime-warmup)/updateTime)<<" tests"<<std::endl;
  double avgTTVAll=0;//time to visit all
  for(int i=0;i<nWifi;i++)
    {
     if(visitedAll[i])
        avgTTVAll+=timeToVisitAll[i];
    }
        
     std::cout<<"no of nodes which visited all: "<<noOfOnes(visitedAll)<<" , with the avg. time: "<<avgTTVAll/double(noOfOnes(visitedAll)) <<std::endl;

  Simulator::Destroy ();
return 0;
  
}
//###################### end of main() ###########################


void 
BFS(bool netGraph[nWifi][nWifi],int src,int dst, int *path,int dis[nWifi])
{
  int predecessor[nWifi];//the predecessor of each node toward the src
  bool reachDst=false;
  bool noPath=false;
  int current=src;
  list<int> currentLevel;
  for(int i=0;i<nWifi;i++)//marking all vertices as unvisited
     {
       predecessor[i]=-1;
       dis[i]=-1;
     }
  dis[src]=0;
  while(!reachDst && !noPath)
   {
     for(int i=0;i<nWifi;i++)
        {
          if(netGraph[current][i])
             {
               if(i==dst)
                 {
                   reachDst=true;
                 }
               if(predecessor[i]==-1)// it is not seen yet
                 {
                   currentLevel.push_back(i);
                   predecessor[i]=current;
                   dis[i]=dis[current]+1;
                 }
             }           
        } 
     if(!currentLevel.empty())
        {  
          current=currentLevel.front();
          currentLevel.pop_front();
        }
     else
       noPath=true;
   }
  if(!reachDst)//there is no path
    {
      path[0]=-1;
    }
  else
    {
      path[dis[dst]]=dst;
      for(int i=dis[dst];i>0;i--)
         {
           path[i-1]=predecessor[path[i]];
         }
    }
}


void 
traceUAVpositions()
{
  Vector currentPos;
  Ptr<MobilityModel> mobility;
  for(int i=0;i<nWifi;i++)
     {
       mobility = mobilityTrackingSocket[i]->GetNode()->GetObject<MobilityModel>();
       currentPos=mobility->GetPosition();

       posMatrix[i][0]=posMatrix[i][4];
       posMatrix[i][1]=posMatrix[i][5];
       posMatrix[i][2]=posMatrix[i][6];
       posMatrix[i][3]=posMatrix[i][7];

       posMatrix[i][4]=posMatrix[i][8];
       posMatrix[i][5]=posMatrix[i][9];
       posMatrix[i][6]=posMatrix[i][10];
       posMatrix[i][7]=posMatrix[i][11];

       posMatrix[i][8]=double(Simulator::Now ().GetSeconds ());
       posMatrix[i][9]=currentPos.x;
       posMatrix[i][10]=currentPos.y;
       posMatrix[i][11]=currentPos.z;
     }
 traceCount++;
 if(traceCount>=3)
   {
     Simulator::Schedule (Seconds (routCheckTime), &traceUAVpositions);
   }
}

void
updateAdj()
{
 for (int i=0;i<nWifi-1;i++)//to reset the matrix
   for (int j=i;j<nWifi;j++)
     {
       adj[i][j]=false;
       adj[j][i]=false;
     }

 for (int i=0;i<nWifi-1;i++)
   for (int j=i;j<nWifi;j++)
     {
       if (distance(i,j)<= TxRange)
         {
           adj[i][j]=true;
           adj[j][i]=true;
         }
     }
 calcNoOfNeigh();
 updateTimeToVisitAll();
 calcConnectivity();
 calcPhyConnectivity();
 keyPathLen(0, nWifi-1);
 Simulator::Schedule (Seconds (updateTime), &updateAdj);
}

double 
distance(int src,int dst)
{
  double dist=0;

  Ptr<MobilityModel> mobilitySrc;
  Vector currentPosSrc;
  mobilitySrc = mobilityTrackingSocket[src]->GetNode()->GetObject<MobilityModel>();
  currentPosSrc=mobilitySrc->GetPosition();

  Ptr<MobilityModel> mobilityDst;
  mobilityDst = mobilityTrackingSocket[dst]->GetNode()->GetObject<MobilityModel>();
  Vector currentPosDst;
  currentPosDst=mobilityDst->GetPosition();

  dist=sqrt(pow((currentPosSrc.x-currentPosDst.x),2)+pow((currentPosSrc.y-currentPosDst.y),2)+pow((currentPosSrc.z-currentPosDst.z),2)); 
  return dist;
}

void 
setupMobilityTrack(NodeContainer adhocNodes)
{
  for (int i=0;i<nWifi;i++)
      {
         mobilityTrackingSocket[i] = Socket::CreateSocket (adhocNodes.Get (i), TcpSocketFactory::GetTypeId ());
         mobilityTrackingSocket[i]->Bind ();
      }
 Simulator::Schedule (Seconds (warmup), &updateAdj);
/* traceUAVpositions();//to initilize the posMatrix
 traceUAVpositions();//
 traceUAVpositions();//*/
}




void
setupMobility(double X, double Y, double Z, NodeContainer adhocNodes,uint32_t mobilityModel,int64_t streamIndex)
{
NS_LOG_FUNCTION("setupMobility");

  ObjectFactory pos;
  std::stringstream sX;
  std::stringstream sY;
  std::stringstream sZ;
  sX<<"ns3::UniformRandomVariable[Min=0.0|Max="<<X<<"]";
  sY<<"ns3::UniformRandomVariable[Min=0.0|Max="<<Y<<"]";
  sZ<<"ns3::UniformRandomVariable[Min=0.0|Max="<<Z<<"]";

  int nodeSpeed=50;  //in m/s
  int nodePause = 0; //in s
  double direction=6.283185307; // in radian
  double pitch=0.05; // in radian

  Ptr<PositionAllocator> taPositionAlloc;

  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";
  std::stringstream ssPause;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";
  std::stringstream ssDirection;
  ssDirection << "ns3::UniformRandomVariable[Min=0|Max=" << direction << "]";
  std::stringstream ssPitch;
  ssPitch << "ns3::UniformRandomVariable[Min="<< pitch <<"|Max=" << pitch << "]";
  std::stringstream ssNormVelocity;
  ssNormVelocity <<"ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]";
  std::stringstream ssNormDirection;
  ssNormDirection <<"ns3::NormalRandomVariable[Mean=0.0|Variance=0.2|Bound=0.4]";
  std::stringstream ssNormPitch;
  ssNormPitch <<"ns3::NormalRandomVariable[Mean=0.0|Variance=0.02|Bound=0.04]";

  switch (mobilityModel)
    {
    case 1:        
      pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
      pos.Set ("X", StringValue (sX.str()));
      pos.Set ("Y", StringValue (sY.str()));
      pos.Set ("Z", StringValue (sZ.str()));

      taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
      streamIndex += taPositionAlloc->AssignStreams (streamIndex);

      mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc));
      break;
    case 2:
      pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
      pos.Set ("X", StringValue (sX.str()));
      pos.Set ("Y", StringValue (sY.str()));
      pos.Set ("Z", StringValue (sZ.str()));

      taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
      streamIndex += taPositionAlloc->AssignStreams (streamIndex);

      mobilityAdhoc.SetMobilityModel ("ns3::GaussMarkovMobilityModel",
                     "Bounds", BoxValue (Box (0, X, 0, Y, 0, Z)),
                     "TimeStep", TimeValue (Seconds (0.5)),
                     "Alpha", DoubleValue (0.85),
                     "MeanVelocity", StringValue (ssSpeed.str()),
                     "MeanDirection", StringValue (ssDirection.str()),
                     "MeanPitch", StringValue (ssPitch.str()),
                     "NormalVelocity", StringValue (ssNormVelocity.str()),
                     "NormalDirection", StringValue (ssNormDirection.str()),
                     "NormalPitch", StringValue (ssNormPitch.str()));
      break;
    case 3:        
      pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
      pos.Set ("X", StringValue (sX.str()));
      pos.Set ("Y", StringValue (sY.str()));
      //pos.Set ("Z", StringValue (sZ.str()));

      taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
      streamIndex += taPositionAlloc->AssignStreams (streamIndex);

      mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc));
      break;
    case 4:
      pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
      pos.Set ("X", StringValue (sX.str()));
      pos.Set ("Y", StringValue (sY.str()));
      //pos.Set ("Z", StringValue (sZ.str()));

      taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
      streamIndex += taPositionAlloc->AssignStreams (streamIndex);

      mobilityAdhoc.SetMobilityModel ("ns3::GaussMarkovMobilityModel",
                     "Bounds", BoxValue (Box (0, X, 0, Y, 0, Z)),
                     "TimeStep", TimeValue (Seconds (0.5)),
                     "Alpha", DoubleValue (0.85),
                     "MeanVelocity", StringValue (ssSpeed.str()),
                     "MeanDirection", StringValue (ssDirection.str()),
                     "MeanPitch", StringValue (ssPitch.str()),
                     "NormalVelocity", StringValue (ssNormVelocity.str()),
                     "NormalDirection", StringValue (ssNormDirection.str()),
                     "NormalPitch", StringValue (ssNormPitch.str()));
      break;
/*    case 3:
      mobilityAdhoc.SetMobilityModel ("ns3::SteadyStateRandomWaypointMobilityModel",
                                  //"Speed", StringValue (ssSpeed.str ()),
                                  //"Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAllocSSRWP));
      
      break;
    case 4:
      mobilityAdhoc.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                              "Mode", StringValue ("Time"),
                              "Time", StringValue ("2s"),
                              "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"),
                              "Bounds", BoxValue (Box (0, X, 0, Y, 0, Z)));
*/
    default:
      NS_FATAL_ERROR ("No such model:" << mobilityModel);
    }
  


/*If you want to fix the node positions

mobilityAdhoc.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
        Vector node1_Position(0.1, 0.1, 0.0);
	Vector node2_Position(50.0, 0.1,0.0);
	Vector node3_Position(100.0, 0.1, 0.0);	

	ListPositionAllocator myListPositionAllocator;
	myListPositionAllocator.Add(node1_Position);
	myListPositionAllocator.Add(node2_Position);
	myListPositionAllocator.Add(node3_Position);
	
	mobilityAdhoc.SetPositionAllocator(&myListPositionAllocator);
*/

  mobilityAdhoc.SetPositionAllocator (taPositionAlloc);
  mobilityAdhoc.Install (adhocNodes);
  streamIndex += mobilityAdhoc.AssignStreams (adhocNodes, streamIndex);
  NS_UNUSED (streamIndex); // From this point, streamIndex is unused
}

YansWifiPhyHelper
setupWifiPhy(double txp)
{
  YansWifiPhyHelper wifiPhy; 
  wifiPhy =  YansWifiPhyHelper::Default ();
  
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  
  wifiPhy.SetChannel (wifiChannel.Create ());

  wifiPhy.Set ("TxPowerStart",DoubleValue (txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txp));
  return wifiPhy;
}

Ipv4InterfaceContainer 
setupIP(NetDeviceContainer adhocDevices)
{
  Ipv4AddressHelper addressAdhoc;
  addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer adhocInterfaces;
  adhocInterfaces = addressAdhoc.Assign (adhocDevices);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
  return adhocInterfaces;
}

void 
calcNoOfNeigh()
{
 uint32_t tempNoOfNeighb=0;
 for (int i=0;i<nWifi;i++)
    tempNoOfNeighb+=noOfOnes(adj[i]);
 avgNoOfNeighb+=tempNoOfNeighb/nWifi;
 double tempVarNoOfNeighb=0;
 for (int i=0;i<nWifi;i++)
    tempVarNoOfNeighb+=(pow(noOfOnes(adj[i])-tempNoOfNeighb/double(nWifi),2));
 varNoOfNeighb+=tempVarNoOfNeighb/nWifi;
}

uint32_t
noOfOnes(bool array[nWifi])
{
 uint32_t ones=0;
 for(int j=0;j<nWifi;j++)
    {
       if (array[j])
          ones++;
    }
 return ones;
}

void 
updateTimeToVisitAll()
{
 for(int i=0;i<nWifi;i++)//to update visited matrix
    {
      for(int j=0;j<nWifi;j++)
        {
          if(i==j || adj[i][j])
             visited[i][j]=true;
        }
     }
  for(int i=0;i<nWifi;i++)//to check if any of nodes visited all others
      if(noOfOnes(visited[i])>=nWifi && timeToVisitAll[i]==0)
        { 
         timeToVisitAll[i]=double(Simulator::Now ().GetSeconds ())-warmup;
         visitedAll[i]=true;
        }
}

void 
calcConnectivity()
{
 
 if(!fullyConnectedNetwork && isConnected(visited))
   { 
     fullConnectivityTime=double(Simulator::Now ().GetSeconds ())-warmup;
     fullyConnectedNetwork=true;
   }
}

bool
isConnected(bool matrix[nWifi][nWifi])
{
//returns true if the network is connected based on the given matrix
 bool accessed[nWifi]={};
 list<int> currentLevel;
 currentLevel.push_back(0);
 int current;
 while(!currentLevel.empty() && noOfOnes(accessed)<=nWifi)
   {
     current=currentLevel.front();
     currentLevel.pop_front();
     accessed[current]=true;
     for (int i=0;i<nWifi;i++)
       {
         if(matrix[current][i] && i!=current && !accessed[i])
          {
            currentLevel.push_back(i);
            accessed[i]=true;
          }    
      }      
   }
 if(noOfOnes(accessed)>=nWifi)
   return true;
 else
   return false;
}

double
connectivityProbability(bool matrix[nWifi][nWifi])
{
//returns true if the network is connected based on the given matrix
 bool accessed[nWifi]={};
 list<int> currentLevel;
 currentLevel.push_back(0);
 int current;
 while(!currentLevel.empty() && noOfOnes(accessed)<=nWifi)
   {
     current=currentLevel.front();
     currentLevel.pop_front();
     accessed[current]=true;
     for (int i=0;i<nWifi;i++)
       {
         if(matrix[current][i] && i!=current && !accessed[i])
          {
            currentLevel.push_back(i);
            accessed[i]=true;
          }    
      }      
   }
 return noOfOnes(accessed)/double(nWifi);
}

void
calcPhyConnectivity()
{
 physicalConnectivity+=connectivityProbability(adj);
}

void 
keyPathLen(int src, int dst)
{
 int path[nWifi];
 for(int i=0;i<nWifi;i++)
   path[i]=-1;
 int dis[nWifi];//the distance from the src
 BFS(visited, src, dst,path,dis);
 if (dis[dst]>0)
   { 
     keyPathLength+=dis[dst];
     keyPathLengthCount++;
   }
}
