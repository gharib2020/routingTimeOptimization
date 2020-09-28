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

NS_LOG_COMPONENT_DEFINE ("UAVrouting");

void setupMobility(double, double,double,NodeContainer,uint32_t);
void setupRoutingProtocol(uint32_t,NodeContainer);
YansWifiPhyHelper setupWifiPhy(double);
Ipv4InterfaceContainer setupIP(NetDeviceContainer);
void setupConnectionSocket(int,int,double,uint16_t,Ipv4InterfaceContainer,NodeContainer);
void setupConnectionOnOff(int,int,double,uint16_t,Ipv4InterfaceContainer,NodeContainer);
void setupConnectionOnOffSocket(int,int,double,uint16_t,Ipv4InterfaceContainer,NodeContainer);
void StartFlow (Ptr<Socket>, Ipv4Address, uint16_t);
void WriteUntilBufferFull (Ptr<Socket>, uint32_t);
void SetupPacketReceive (Ipv4Address, Ptr<Node>,Ipv4Address);
void ReceivePacket (Ptr<Socket>);
void accept(Ptr<Socket> ,const ns3::Address&);



// The number of bytes to send in this simulation.
static const uint32_t totalTxBytes = 3000000;
static uint32_t currentTxBytes = 0;
static uint32_t currentRxBytes = 0;
static uint32_t packetsReceived=0;
// Perform series of 1040 byte writes (this is a multiple of 26 since
// we want to detect data splicing in the output stream)
static const uint32_t writeSize = 1040;
static uint16_t port = 50000;
uint8_t data[writeSize];
const std::string rate="2048bps";


// These are for starting the writing process, and handling the sending 
// socket's notification upcalls (events).  These two together more or less
// implement a sending "Application", although not a proper ns3::Application
// subclass.


static void CwndTracer (uint32_t oldval, uint32_t newval)
{
  NS_LOG_INFO ("Moving cwnd from " << oldval << " to " << newval);
}

//###################### main() ###########################
int main (int argc, char *argv[])
{
  
  int nWifi=5;
  int nSinks=1; 
  double txp=7.5;
  double totalTime=1000;
  uint32_t mobilityModel=1;//1-RWP, 2-GaussMarkov 
  uint32_t routingProtocol=2;//1-OLSR, 2-AODV, 3-DSDV, 4-DSR
  double X=100.0;
  double Y=50.0;
  double Z=10.0;
  std::string phyMode ("DsssRate11Mbps");
  

  ns3::PacketMetadata::Enable (); 

  std::stringstream ss;
  ss<<"traceFiles/testUAV"<<nWifi<<"Con"<<nSinks;
  std::string tr_name (ss.str ());
  
  CommandLine cmd;
  cmd.Parse (argc, argv);
  
  NS_LOG_UNCOND ("Starting the simulation...");

  // initialize the tx buffer.
  for(uint32_t i = 0; i < writeSize; ++i)
    {
      char m = toascii (97 + i % 26);
      data[i] = m;
    }
    
  // Here, we will explicitly create the nodes. 
  // This will be used to install the network
  // interfaces and connect them with a channel.
  NodeContainer adhocNodes;
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
  NS_LOG_UNCOND ("Setting up wifi physical attributes...");
  //Set Non-unicastMode rate to unicast mode
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (phyMode));

  // Add a mac 
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");  
  
  NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, adhocNodes);

  //setup mobility
  setupMobility(X,Y,Z,adhocNodes,mobilityModel);  
  NS_LOG_UNCOND ("Setting up Mobility...");
  //setup routing protocol
  setupRoutingProtocol(routingProtocol,adhocNodes);
  NS_LOG_UNCOND ("Setting up routing protocol...");
  // Later, we add IP addresses.
  Ipv4InterfaceContainer adhocInterfaces;
  adhocInterfaces=setupIP(adhocDevices);
  NS_LOG_UNCOND ("Setting up IP address for nodes...");
  
  ///////////////////////////////////////////////////////////////////////////
  // Simulation 1
  //
  // Send 2000000 bytes over a connection to server port 50000 at time 0
  // Should observe SYN exchange, a lot of data segments and ACKS, and FIN 
  // exchange.  FIN exchange isn't quite compliant with TCP spec (see release
  // notes for more info)
  //
  ///////////////////////////////////////////////////////////////////////////



  // Create the connections...
  
  
  for (int i = 0; i < nSinks; i++)
     {        
        setupConnectionSocket(i,i+nSinks,totalTime,port,adhocInterfaces,adhocNodes);
     }


  // One can toggle the comment for the following line on or off to see  
  // the effects of finite send buffer modelling.  One can also change 
  // the size of said buffer.

  //localSocket->SetAttribute("SndBufSize", UintegerValue(4096));

  //Ask for ASCII and pcap traces of network traffic
  
  AsciiTraceHelper ascii;
  Ptr<FlowMonitor> flowmon;

  wifiPhy.EnablePcapAll (tr_name);
  wifiPhy.EnableAsciiAll (ascii.CreateFileStream (tr_name+".tr"));
  MobilityHelper::EnableAsciiAll (ascii.CreateFileStream (tr_name + ".mob"));

  AnimationInterface anim (tr_name+".xml");
  anim.SetMaxPktsPerTraceFile (2000000);

  FlowMonitorHelper flowmonHelper;
  flowmon = flowmonHelper.InstallAll (); 
  
  // Finally, set up the simulator to run.  The 'totalTime' second hard limit is a
  // failsafe in case some change above causes the simulation to never end
  Simulator::Stop (Seconds (totalTime));
  Simulator::Run ();
  
  flowmon->SerializeToXmlFile ((tr_name + ".flowmon").c_str(), true, true);

  NS_LOG_UNCOND ("Ending the simulation...");
  Simulator::Destroy ();

  
}
//###################### end of main() ###########################
void accept(Ptr<Socket> socket,const ns3::Address& from)
{

    std::cout<<"Connection accepted"<< std::endl;
    socket->SetRecvCallback (MakeCallback (&ReceivePacket));

}

void ReceivePacket (Ptr<Socket> socket)
 {
    
   Ptr<Packet> packet = socket->Recv ();
   packetsReceived+=1;
   SocketIpTosTag tosTag;
   if (packet->RemovePacketTag (tosTag))
     {
       NS_LOG_INFO (" TOS = " << (uint32_t)tosTag.GetTos ());
     }
   SocketIpTtlTag ttlTag;
   if (packet->RemovePacketTag (ttlTag))
     {
       NS_LOG_INFO (" TTL = " << (uint32_t)ttlTag.GetTtl ());
     }
     if(packetsReceived==400)
       {
          //socket->Close();
        }
     std::stringstream ssFlow;
     ssFlow<<"No. of received packets= "<<packetsReceived<<" and No. of received Bytes= "<<currentRxBytes;
     //NS_LOG_UNCOND (ssFlow.str());
 }

void
SetupPacketReceive (Ipv4Address addr, Ptr<Node> node,Ipv4Address sourceAddress)
{

  TypeId tid = TypeId::LookupByName ("ns3::TcpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, port);
  sink->Bind (local);
  sink->Connect (InetSocketAddress (sourceAddress, port));
  sink->SetRecvCallback (MakeCallback (&ReceivePacket));
  ReceivePacket(sink);
}

void
StartFlow (Ptr<Socket> localSocket,
                Ipv4Address servAddress,
                uint16_t port)
{
//begin implementation of sending "Application"
    localSocket->Connect (InetSocketAddress (servAddress, port)); //connect
  // tell the tcp implementation to call WriteUntilBufferFull again
  // if we blocked and new tx buffer space becomes available

  localSocket->SetSendCallback (MakeCallback (&WriteUntilBufferFull));
  WriteUntilBufferFull (localSocket, localSocket->GetTxAvailable ());
  
}

void
WriteUntilBufferFull (Ptr<Socket> localSocket, uint32_t txSpace)
{ 
  std::stringstream ssFlow;
  while (currentTxBytes < totalTxBytes && localSocket->GetTxAvailable () > 0) 
    { 
      currentRxBytes+=currentRxBytes;    
      uint32_t left = totalTxBytes - currentTxBytes;
      uint32_t dataOffset = currentTxBytes % writeSize;
      uint32_t toWrite = writeSize - dataOffset;
      toWrite = std::min (toWrite, left);
      toWrite = std::min (toWrite, localSocket->GetTxAvailable ());
      int amountSent = localSocket->Send (&data[dataOffset], toWrite, 0);
      if(amountSent < 0)
        {
          // we will be called again when new tx space becomes available.
          //ssFlow<<"Negative amount sent"<<std::endl;
          //NS_LOG_UNCOND (ssFlow.str ());   
          return;  
        }
      currentTxBytes += amountSent;
    }
 // ssFlow<<"time: "<<Simulator::Now ().GetSeconds ()<<" the connection is closing, "<<std::endl;
 
  if (currentTxBytes==totalTxBytes)
     {
       // ssFlow<<totalTxBytes;
        //NS_LOG_UNCOND (ssFlow.str ());
        localSocket->Close ();
     }
}

void
setupMobility(double X, double Y, double Z, NodeContainer adhocNodes,uint32_t mobilityModel)
{
NS_LOG_FUNCTION("setupMobility");
MobilityHelper mobilityAdhoc;

  int64_t streamIndex = 1; // used to get consistent mobility across scenarios

  ObjectFactory pos;
  std::stringstream sX;
  std::stringstream sY;
  std::stringstream sZ;
  sX<<"ns3::UniformRandomVariable[Min=0.0|Max="<<X<<"]";
  sY<<"ns3::UniformRandomVariable[Min=0.0|Max="<<Y<<"]";
  sZ<<"ns3::UniformRandomVariable[Min=0.0|Max="<<Z<<"]";
  
  pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
  pos.Set ("X", StringValue (sX.str()));
  pos.Set ("Y", StringValue (sY.str()));
  pos.Set ("Z", StringValue (sZ.str()));

  Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
  streamIndex += taPositionAlloc->AssignStreams (streamIndex);

  int nodeSpeed=20;  //in m/s
  int nodePause = 0; //in s
  double direction=6.283185307; // in radian
  double pitch=0.05; // in radian

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
      mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc));
      break;
    case 2:
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

void setupRoutingProtocol(uint32_t protocol,NodeContainer adhocNodes)
{
NS_LOG_FUNCTION("setupMobility");
AodvHelper aodv;
  OlsrHelper olsr;
  DsdvHelper dsdv;
  DsrHelper dsr;
  DsrMainHelper dsrMain;
  Ipv4ListRoutingHelper list;
  

  switch (protocol)
    {
    case 1:
      list.Add (olsr, 100);
      //protocolName = "OLSR";
      break;
    case 2:
      list.Add (aodv, 100);
      //protocolName = "AODV";
      break;
    case 3:
      list.Add (dsdv, 100);
      //protocolName = "DSDV";
      break;
    case 4:
      //protocolName = "DSR";
      break;
    default:
      NS_FATAL_ERROR ("No such protocol:" << protocol);
    }

// Now add ip/tcp stack to all nodes.
InternetStackHelper internet;
  if (protocol < 4)
    {
      internet.SetRoutingHelper (list);
      internet.InstallAll();// (adhocNodes);
    }
  else if (protocol == 4)
    {
      internet.Install (adhocNodes);
      dsrMain.Install (dsr, adhocNodes);
    }
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
  return adhocInterfaces;
}

void
setupConnectionOnOffSocket(int source,int destination,double totalTime,uint16_t  port,Ipv4InterfaceContainer adhocInterfaces,NodeContainer adhocNodes)
{
  //setup onoff helper
  Config::SetDefault ("ns3::TcpL4Protocol::SocketType", StringValue ("ns3::TcpNewReno"));
  Config::SetDefault  ("ns3::OnOffApplication::PacketSize",StringValue ("536"));
  Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue (rate));
  OnOffHelper onoff1 ("ns3::TcpSocketFactory",Address ());
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
  //end of onOff setup

  onoff1.SetAttribute ("Remote", AddressValue(InetSocketAddress (adhocInterfaces.GetAddress (destination), port)));
  

  ApplicationContainer apps; 
  apps=onoff1.Install (adhocNodes.Get (source));
  Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
  apps.Start (Seconds (var->GetValue (0.0,1.0)));
  apps.Stop (Seconds (totalTime));

// Create and bind the sink socket...
  TypeId tid = TypeId::LookupByName ("ns3::TcpNewReno");
  Config::Set ("/NodeList/*/$ns3::TcpL4Protocol/SocketType", TypeIdValue (tid));
  Ptr<Socket> sinkSocket; 
  sinkSocket = Socket::CreateSocket (adhocNodes.Get (destination), TcpSocketFactory::GetTypeId ());
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), port);
  sinkSocket->Bind(local);
  sinkSocket->SetAcceptCallback (MakeNullCallback<bool, Ptr<Socket>,const Address &> (),MakeCallback(&accept));
  sinkSocket->Listen();
  //sinkSocket->SetRecvCallback (MakeCallback (&ReceivePacket));
  
}

void 
setupConnectionSocket(int source,int destination,double totalTime,uint16_t  port,Ipv4InterfaceContainer adhocInterfaces,NodeContainer adhocNodes)
{
/*
  PacketSinkHelper sink ("ns3::TcpSocketFactory",
                         InetSocketAddress (Ipv4Address::GetAny (), port));
  ApplicationContainer apps = sink.Install (adhocNodes.Get (destination));
  apps.Start (Seconds (0.0));
  apps.Stop (Seconds (totalTime));
*/

// Create and bind the sink socket...
  TypeId tid = TypeId::LookupByName ("ns3::TcpNewReno");
  Config::Set ("/NodeList/*/$ns3::TcpL4Protocol/SocketType", TypeIdValue (tid));
  Ptr<Socket> sinkSocket; 
  sinkSocket = Socket::CreateSocket (adhocNodes.Get (destination), TcpSocketFactory::GetTypeId ());
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), port);
  sinkSocket->Bind(local);
  sinkSocket->SetAcceptCallback (MakeNullCallback<bool, Ptr<Socket>,const Address &> (),MakeCallback(&accept));
  sinkSocket->Listen();

  // Create a source to send packets.  Instead of a full Application
  // and the helper APIs you might see in other example files, this example
  // will use sockets directly and register some socket callbacks as a sending
  // "Application".

  // Create and bind the socket...
  Ptr<Socket> localSocket =
  Socket::CreateSocket (adhocNodes.Get (source), TcpSocketFactory::GetTypeId ());
  localSocket->Bind ();
  //sinkSocket->SetAcceptCallback(true,localSocket,adhocInterfaces.GetAddress (source),MakeCallback (&ReceivePacket),adhocInterfaces.GetAddress (destination));
  // Trace changes to the congestion window
  Config::ConnectWithoutContext ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow", MakeCallback (&CwndTracer));

  // ...and schedule the sending "Application"; This is similar to what an 
  // ns3::Application subclass would do internally.
  Simulator::ScheduleNow (&StartFlow, localSocket,
                       adhocInterfaces.GetAddress (destination), port);
}

void
setupConnectionOnOff(int source,int destination,double totalTime,uint16_t  port,Ipv4InterfaceContainer adhocInterfaces,NodeContainer adhocNodes)
{
  //setup onoff helper
  //Config::SetDefault ("ns3::TcpL4Protocol::SocketType", StringValue ("ns3::TcpNewReno"));
  Config::SetDefault  ("ns3::OnOffApplication::PacketSize",StringValue ("536"));
  Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue (rate));
  OnOffHelper onoff1 ("ns3::TcpSocketFactory",Address ());
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
  //end of onOff setup

  PacketSinkHelper sink ("ns3::TcpSocketFactory",
                         InetSocketAddress (Ipv4Address::GetAny (), port));
  ApplicationContainer sinkApps = sink.Install (adhocNodes.Get (destination));
  sinkApps.Start (Seconds (0.0));
  sinkApps.Stop (Seconds (totalTime));
//  Ptr<Socket> sink = SetupPacketReceive (adhocInterfaces.GetAddress (destination), adhocNodes.Get (destination));
  
  onoff1.SetAttribute ("Remote", AddressValue(InetSocketAddress (adhocInterfaces.GetAddress (destination), port)));
  

  ApplicationContainer apps; 
  apps=onoff1.Install (adhocNodes.Get (source));
  Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
  apps.Start (Seconds (var->GetValue (0.0,1.0)));
  apps.Stop (Seconds (totalTime));
}
