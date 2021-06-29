#include <bits/stdc++.h> 
#include <fstream>
#include <iostream>
#include <vector>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/drop-tail-queue.h"
#include <bits/stdc++.h> 

//#include "ns3/nSinks-monitor-helper.h"

using namespace ns3;
using namespace dsr;
using namespace std; 

NodeContainer adhocNodes;
        //adhocNodes.Create (4);
int const nWifis = 10;
struct Flow
{
 public:

	int source;
	int dest;
	int v =nWifis ;
	vector<int> result;
        Ptr<Socket> RecvSocket[nWifis];
        Ptr<Socket> SendSocket[nWifis];
        //NodeContainer adhocNodes;
        //adhocNodes.Create (4);
	vector<int> shortestpath(std::vector<int> adj[], int s, int dest, int const v, float trust[]);
        Ptr<Socket> InstallRecSocket (Ipv4Address addr, Ptr<Node> node);
        Ptr<Socket> InstallSendSocket (Ipv4Address addr, Ptr<Node> node);
        void ReceivePacket (Ptr<Socket> socket);
        int findNextNode(int);
        void ReceiveFinalPacket (Ptr <Socket> socket);
        void accept(Ptr<Socket> s, const Address & addr);
        void SendSrc (Ptr<Socket>, uint32_t);
        void printname() 
         { 
             cout << "the source: " << source << " the dest is: "<< dest; 
        } 
};


NS_LOG_COMPONENT_DEFINE ("manet-routing-compare");
static const uint32_t writeSize = 1040;
uint8_t data[writeSize];
uint32_t port = 9;
static const uint32_t totalTxBytes = 2000000;
  Ipv4InterfaceContainer adhocInterfaces;
//int const nWifis = 4;

  //Ptr<Socket> RecvSocket[nWifis];
  //Ptr<Socket> SendSocket[nWifis];
uint8_t posarr[nWifis];
double distance_pro;
double txRange = 138.3;
//double distTable[4][4];    
bool distTable[nWifis][nWifis];
//double trustTable[nWifis][nWifis];
float trustTable[nWifis];
vector<int> flow_result;
int len;


uint32_t currentTxBytes =0;
//private:
  //std::vector<int> printShortestDistance(std::vector<int> adj[], int s, int dest, int v);
  //bool BFS(std::vector<int> adj[], int src, int dest, int v, int pred[], int dist[]); 
   //uint32_t port;
  uint32_t bytesTotal = 0; //total
  uint32_t packetsReceived = 0;

  std::string m_CSVfileName = "manet-routing.output.csv";
 //m_protocolName = "protocol";
  int m_nFlows;
uint32_t m_protocol = 2; 
  std::string m_protocolName;
  double m_txp;
  bool m_traceMobility = false;



//dstaddr = adhocInterfaces.GetAddress (1);



  //Ptr<Socket> InstallRecSocket (Ipv4Address addr, Ptr<Node> node);
  //Ptr<Socket> InstallSendSocket (Ipv4Address addr, Ptr<Node> node);
  //void SendSrc (Ptr<Socket>, uint32_t);
  //void InterReceiveSocket(Ptr<Socket> socket);
  //void SendStuff (Ptr<Socket> socket, Ipv4Address addr);
  //void ReceivePacket (Ptr<Socket> socket);
  //void ReceiveFinalPacket (Ptr <Socket> socket);
  //void accept(Ptr<Socket> s, const Address & addr);
  void CheckThroughput ();
  double calDistance(Ptr<Node> node1, Ptr<Node> node2);
  void add_edge(std::vector<int> adj[], int src, int dest); 
  float * calTrustScore(float trustTable);



static inline std::string
PrintReceivedPacket (Ptr<Socket> socket, Ptr<Packet> packet, Address senderAddress)
{
  std::ostringstream oss;

  oss << Simulator::Now ().GetSeconds () << " " << socket->GetNode ()->GetId ();

  if (InetSocketAddress::IsMatchingType (senderAddress))
    {
      InetSocketAddress addr = InetSocketAddress::ConvertFrom (senderAddress);
      oss << " received one packet from " << addr.GetIpv4 ();
    }
  else
    {
      oss << " received one packet!";
    }
  return oss.str ();
}




bool BFS(std::vector<int> adj[], int src, int dest, int const v, 
         int pred[], int dist[], float trust[]) 
{ 
    // a queue to maintain queue of vertices whose 
    // adjacency list is to be scanned as per normal 
    // DFS algorithm 
    list<int> queue; 
  
    // boolean array visited[] which stores the 
    // information whether ith vertex is reached 
    // at least once in the Breadth first search 
    bool visited[v]; 
  
    // initially all vertices are unvisited 
    // so v[i] for all i is false 
    // and as no path is yet constructed 
    // dist[i] for all i set to infinity 
    for (int i = 0; i < v; i++) { 
        visited[i] = false; 
        dist[i] = INT_MAX; 
        pred[i] = -1; 
    } 
  
    // now source is first to be visited and 
    // distance from source to itself should be 0 
    visited[src] = true; 
    dist[src] = 0; 
    queue.push_back(src); 
  
    // standard BFS algorithm 
    while (!queue.empty()) { 
        int u = queue.front(); 
        int len = adj[u].size();
        queue.pop_front(); 
        for (int i = 0; i < len; i++) { 
 //we just consider a threshold for the trust score. Here the threshold is 0.4  
            if (visited[adj[u][i]] == false && trust[adj[u][i]] > 0.4) { 
                visited[adj[u][i]] = true; 
                dist[adj[u][i]] = dist[u] + 1; 
                pred[adj[u][i]] = u; 
                queue.push_back(adj[u][i]); 
  
                // We stop BFS when we find 
                // destination. 
                if (adj[u][i] == dest) 
                    return true; 
            } 
        } 
    } 
  
    return false; 
} 

vector<int> Flow:: shortestpath(vector<int> adj[], int s, 
                           int dest, int const v, float trust[]) 
{ 
    // predecessor[i] array stores predecessor of 
    // i and distance array stores distance of i 
    // from s 
    int pred[v], dist[v];
    std::vector<int> path;
int value =0; 
  
    if (BFS(adj, s, dest, v, pred, dist, trust) == false) { 
        cout << "Given source and destination"
             << " are not connected"; 
	fill(path.begin(), path.end(), value);
        return path; 
    } 
  
    // vector path stores the shortest path 
    //std::vector<int> path; 
    int crawl = dest; 
    path.push_back(crawl); 
    while (pred[crawl] != -1) { 
        path.push_back(pred[crawl]); 
        crawl = pred[crawl]; 
    } 
  
    // distance from source is in distance array 
    cout << "Shortest path length is : "
         << dist[dest]; 
  
    // printing path from source to destination 
    cout << "\nPath is::\n"; 
    for (int i = path.size() - 1; i >= 0; i--) 
       cout << path[i] << " ";
cout<<endl;
for (int i =  path.size() - 1; i >= 0; i--)
    {
        //std::cout<<"for loop i="<<i<<std::endl;
        RecvSocket[path[i]] = InstallRecSocket (adhocInterfaces.GetAddress (path[i]), adhocNodes.Get (path[i]));
        SendSocket[path[i]] = InstallSendSocket (adhocInterfaces.GetAddress (path[i]), adhocNodes.Get (path[i]));
    }
return path; 
} 

void add_edge(vector<int> adj[], int src, int dest) 
{ 
    adj[src].push_back(dest); 
    adj[dest].push_back(src); 
} 



double calDistance( Ptr<Node> node1,  Ptr<Node> node2){
        Vector pos1, pos2;
        double dist;
        Ptr<MobilityModel> position1 = node1->GetObject<MobilityModel> ();
        NS_ASSERT (position1 != 0);
        pos1 = position1->GetPosition ();
        Ptr<MobilityModel> position2 = node2->GetObject<MobilityModel> ();
        NS_ASSERT (position2 != 0);
        pos2 = position2->GetPosition ();

        dist = pow (pos1.x - pos2.x, 2.0) + pow (pos1.y - pos2.y, 2.0) + pow (pos1.z - pos2.z, 2.0);
        return sqrt (dist);

}



float * cal_trust(float trust[]){

	int n_mali = 0.5 * nWifis;
	int arr[n_mali];
        int newitem;
	srand((unsigned) time(0));
for(int i=0;i<n_mali;i++)
{
    bool unique;
    do
    {
      unique=true;
      newitem= (rand() % nWifis);
      for(int i1=0;i1<i;i1++)
      {
         if(arr[i1]==newitem)
         {
           unique=false;     
            break;
         }
      }
    }while(!unique);
    arr[i]=newitem;

}    

double start = 0;
    double end = .4;

    double range = (end - start);
        //cout<<"Random numbers generated between 0 and 0.4:"<<endl;
        for(int i=0;i<n_mali;i++){
           float rnd = float((range * rand()) / (RAND_MAX + 1.0));
           trust[arr[i]] =rnd; 
	  //cout<<"the trust value is:"<<arr[i];	
	}
	for(int i=0; i<nWifis;i++){
		cout<<"the trust value is:"<<trust[i]<<"\n";
	}
   return trust;
}


void Flow::ReceivePacket (Ptr<Socket> socket)
{
//std::cout<<"BytesTotal in ReceivePacket"<<std::endl;
  Ptr<Packet> packet;
 
  Address senderAddress;
  int i;
  while ((packet = socket->RecvFrom (senderAddress)))
    {
       packet->RemoveAllPacketTags ();
      //bytesTotal += packet->GetSize ();
      //packetsReceived += 1;
      //NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
      i = socket->GetNode()->GetId();
//      Ipv4Address interAddr = adhocInterfaces.GetAddress (i);

      int nextNode=findNextNode(i);
      Ipv4Address interNextAddr = adhocInterfaces.GetAddress (nextNode);
      SendSocket[i]->Connect (InetSocketAddress (interNextAddr, port)); //connect ??
      //int amountSent=
      SendSocket[i]->SendTo (packet, 0, InetSocketAddress (interNextAddr,port));
      //std::cout<<"bytes sent "<<amountSent<<std::endl;
      //if(amountSent < 0)
        {
          // we will be called again when new tx space becomes available.
          //return;
        }
      //std::cout<<"Received from "<<socket->GetPeerName(senderAddress)<<" current node "<< i<<"current address "<<interAddr<<"interSocket is "<<SendSocket[i]<<"next address is: "<<interNextAddr<<std::endl;
    }


}




void Flow::ReceiveFinalPacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address senderAddress;
  while ((packet = socket->RecvFrom (senderAddress)))
    {
      bytesTotal += packet->GetSize ();
      packetsReceived += 1;
      NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
    }
}

int Flow::findNextNode(int n){

  int sresult = result.size();

   for(int j=0; j < sresult; j++){

	if( result[j] == n)
             return result[j-1];
    }

return -1;

}

void Flow::accept (Ptr<Socket> sink, const Address & addr){
  int j = sink->GetNode()->GetId();
//std::cout<<"Accept for node "<<j<<std::endl;
 if(j==flow_result[0]){
  sink->SetRecvCallback (MakeCallback (&Flow::ReceiveFinalPacket,this));
  
  }
 else
  sink->SetRecvCallback (MakeCallback (&Flow::ReceivePacket,this));
  

}


Ptr<Socket> Flow::InstallRecSocket (Ipv4Address addr, Ptr<Node> node)
{
  //  std::cout<<"InstallRecSocket, addr: "<<addr<<", node: "<<node<<std::endl;
  TypeId tid = TypeId::LookupByName ("ns3::TcpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, port);
  sink->Bind (local);
  sink->Listen();

  sink->SetAcceptCallback (MakeNullCallback<bool, Ptr< Socket >, const Address &> (),
                              MakeCallback (&Flow::accept,this));

  //sink->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceivePacket, this));

  return sink;
}

Ptr<Socket> Flow::InstallSendSocket (Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::TcpSocketFactory");
  Ptr<Socket> srcSocket = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, port);
  srcSocket->Bind (local);
  //sink->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceivePacket, this));

  return srcSocket;
}


void Flow::SendSrc (Ptr<Socket> localSocket, uint32_t txSpace)
{
 //std::cout<<"SendSrc is called"<<std::endl;
 //int j;
 Ipv4Address dstaddr = adhocInterfaces.GetAddress (flow_result[len-1]); 
 localSocket->Connect (InetSocketAddress (dstaddr, port)); //connect ??
  while (currentTxBytes < totalTxBytes && localSocket->GetTxAvailable () > 0) 
    {
  uint32_t left = totalTxBytes - currentTxBytes;
  uint32_t dataOffset = currentTxBytes % writeSize;
  uint32_t toWrite = writeSize - dataOffset;
  //j = localSocket->GetNode()->GetId();
  toWrite = std::min (toWrite, left);
  toWrite = std::min (toWrite, localSocket->GetTxAvailable ());
  int amountSent=localSocket->SendTo ((&data[dataOffset]), toWrite, 0, InetSocketAddress (dstaddr,port));
  //std::cout<<"bytes sent "<<amountSent<<" to "<<dstaddr <<std::endl;
      if(amountSent < 0)
        {
          // we will be called again when new tx space becomes available.
          Simulator::Schedule (Seconds (0.1), &Flow::SendSrc,this, localSocket,localSocket->GetTxAvailable ());
          return;
        }
  //RecvSocket[j+1]->SetRecvCallback (MakeCallback (&RoutingExperiment::ReceivePacket, this));
    }

//std::cout << "add=" << dstaddr  << std::endl;
//localSocket->Close ();

  // tell the tcp implementation to call WriteUntilBufferFull again
  // if we blocked and new tx buffer space becomes available
  //localSocket->SetSendCallback (MakeCallback (&WriteUntilBufferFull));
  //WriteUntilBufferFull (localSocket, localSocket->GetTxAvailable ());
}

void CheckThroughput ()
{
  double kbs = (bytesTotal * 8.0) / 1000;
  bytesTotal = 0;

  std::ofstream out (m_CSVfileName.c_str (), std::ios::app);

  out << (Simulator::Now ()).GetSeconds () << ","
      << kbs << ","
      << packetsReceived << ","
      << m_nFlows << ","
      << m_protocolName << ","
      << m_txp << ""
      << std::endl;

  out.close ();
  packetsReceived = 0;
  Simulator::Schedule (Seconds (1.0), &CheckThroughput);
}


int main(int argc, char *argv[]) {
//RoutingExperiment experiment;
  //std::string CSVfileName = experiment.CommandSetup (argc,argv);

CommandLine cmd;
  cmd.AddValue ("CSVfileName", "The name of the CSV output file name", m_CSVfileName);
  cmd.AddValue ("traceMobility", "Enable mobility tracing", m_traceMobility);
  cmd.AddValue ("protocol", "1=OLSR;2=AODV;3=DSDV;4=DSR", m_protocol);
  cmd.Parse (argc, argv);
  //return m_CSVfileName;

  //blank out the last output file and write the column headers
  std::ofstream out (m_CSVfileName.c_str ());
  out << "SimulationSecond," <<
  "ReceiveRate," <<
  "PacketsReceived," <<
  "NumberOfSinks," <<
  "RoutingProtocol," <<
  "TransmissionPower" <<
  std::endl;
  out.close ();
  // initialize the tx buffer.
  for(uint32_t i = 0; i < writeSize; ++i)
    {
      char m = toascii (97 + i % 26);
      data[i] = m;
    }

  int nFlows = 2;
  double txp = 7.5;


Packet::EnablePrinting ();
  m_nFlows = nFlows;
  m_txp = txp;
  //m_CSVfileName = CSVfileName;

  //int nWifis = 4;


  double TotalTime = 200.0;
  std::string rate ("2048bps");
  std::string phyMode ("DsssRate11Mbps");
  std::string tr_name ("manet-routing-compare");
  int nodeSpeed = 20; //in m/s
  int nodePause = 0; //in s
  m_protocolName = "protocol";

  Config::SetDefault  ("ns3::OnOffApplication::PacketSize",StringValue ("64"));//total??
  Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue (rate));

  //Set Non-unicastMode rate to unicast mode
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (phyMode));

 //NodeContainer adhocNodes = NodeContainer (nSrc, nDst, nRtr1);  
//NodeContainer adhocNodes;
 adhocNodes.Create (nWifis);

  // setting up wifi phy and channel using helpers
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add a mac and disable rate control
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));

  wifiPhy.Set ("TxPowerStart",DoubleValue (txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txp));

  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, adhocNodes);




  MobilityHelper mobilityAdhoc;
 int64_t streamIndex = 0; // used to get consistent mobility across scenarios

  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
  pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
  pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
  pos.Set ("Z", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"));

  Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
  streamIndex += taPositionAlloc->AssignStreams (streamIndex);

  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";
  std::stringstream ssPause;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";
  mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc));
  mobilityAdhoc.SetPositionAllocator (taPositionAlloc);


/*
 mobilityAdhoc.SetPositionAllocator ("ns3::GridPositionAllocator",
                                     "MinX", DoubleValue (-00.0),
                                     "MinY", DoubleValue (-100.0),
                                     "DeltaX", DoubleValue (138.2),
                                     "DeltaY", DoubleValue (0.0),
                                     "GridWidth", UintegerValue (200),
                                     "LayoutType", StringValue ("RowFirst"));
mobilityAdhoc.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
*/

  mobilityAdhoc.Install (adhocNodes);

//uint8_t pos-arr[4];

 // iterate our nodes and print their position.
      /*for (NodeContainer::Iterator j = adhocNodes.Begin (); j != adhocNodes.End (); ++j)
        //for(int j = 0; j < 4; j++)
        {
//adhocNodes.Get (i)          
          Ptr<Node> object = *j;
          Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
          NS_ASSERT (position != 0);
          Vector pos = position->GetPosition ();
          //posarr[j] = pos.x;
          //int & numRef = pos.size();
          //std::cout << "position="<< numRef<< std::endl;
          //pos[j] = adhocInterfaces.GetPosition (j);
         //std::cout << "position="<< posarr[j] << std::endl;
           std::cout << "x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z << std::endl;
        }
*/
//"<<pos.GetLength()<< std::endl;

 
  streamIndex += mobilityAdhoc.AssignStreams (adhocNodes, streamIndex);
  NS_UNUSED (streamIndex); // From this point, streamIndex is unused

InternetStackHelper internet;

      internet.Install (adhocNodes);


 

  NS_LOG_INFO ("assigning ip address");

  Ipv4AddressHelper addressAdhoc;
  addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");

  adhocInterfaces = addressAdhoc.Assign (adhocDevices);
   //double trustTable [4][4]={{0.8, 1, 0.1, 0.2},
                // {0.8, 1, 0.1, 0.2},
                // {0.8, 1, 0.1, 0.2},
                 //{0.8, 1, 0.1, 0.2}};
 
for (int i = 0; i <nWifis; i++)
{
    for (int j = 0; j <nWifis; j++){
        distance_pro = calDistance(adhocNodes.Get (i), adhocNodes.Get (j));
        //if (distance_pro > 0 && distance_pro < txRange && trustTable[i][j] > 0.4 ){
        if (distance_pro > 0 && distance_pro < txRange){
        
         //distTable[i][j] = distance;
         distTable[i][j] = 1;
                
         std::cout << "the distance between" << i << ", and " << j << ", is: " << distance_pro << std::endl;       

        }
        
        else{
        
         //distTable[i][j] = -1;
         distTable[i][j] = 0;

         std::cout << "No Path Exists! the distance between" << i << ", and " << j << ", is: " << distance_pro << std::endl;
        }

    }
}

for(int i = 0; i < nWifis; i++){
        for(int j = 0; j < nWifis; j++){
            std::cout << "matrix element ["<< i << "][" << j << "] is: " << distTable[i][j] << std::endl; 
        }
        std::cout << "\n" << std::endl;;
    }


    //int v = 5;
  
    // array of vectors is used to store the graph 
    // in the form of an adjacency list 
    std::vector<int> adj[nWifis]; 

for(int i = 0; i < nWifis; i++){
        for(int j = 0; j < nWifis; j++){
                
                if(distTable[i][j] == 1)
                        add_edge(adj, i, j);

        }
        //std::cout << "\n" << std::endl;;
    }
  
    // Creating graph given in the above diagram. 
    // add_edge function takes adjacency list, source 
    // and destination vertex as argument and forms 
    // an edge between them. 
  /*  add_edge(adj, 0, 1); 
    add_edge(adj, 0, 3); 
    add_edge(adj, 1, 2); 
    add_edge(adj, 2, 4);

*/

//float trustTable[nWifis] = {0.8,1,0.8,0.6};
float trustTable[nWifis] = {0.8,1,0.7,0.6,1,0.7,0.8,0.9,0.7,1};
float *final_trustTable = cal_trust(trustTable);

//for (int i=0; i<nWifis; i++){
 //std::cout<<"the trust table is: "<<trustTable[i]<<std::endl;
//}

Flow flow[m_nFlows];    
//flow f1,f2,f3,f4,f5;
    flow[0].source = 3;
    flow[0].dest = 5;
    flow[1].source = 3;
    flow[1].dest = 9;
    flow[0].printname();
    //f[1].printname();
    //f1r = f[0].result;
     //std::cout<<"the trust table is: "<<trustTable[f[0].dest]<<std::endl;

   flow_result = flow[0].shortestpath(adj, flow[0].source, flow[0].dest,  nWifis, final_trustTable);
   len = flow_result.size();
   std::cout<<"the size is: "<<len<<std::endl;

   flow[1].printname();
     
   flow[1].result = flow[1].shortestpath(adj, flow[1].source, flow[1].dest,  nWifis, final_trustTable); 
     
   //flow[0].SendSocket[flow[0].source]->SetSendCallback (MakeCallback(&Flow::SendSrc,flow[0]));

  //std::cout<<"the size is: "<<f1r<<std::endl;

    

/*for (int i = f1r.size() - 1; i >= 0; i--)
    {
        //std::cout<<"for loop i="<<i<<std::endl;
        RecvSocket[i] = InstallRecSocket (adhocInterfaces.GetAddress (i), adhocNodes.Get (i));
        SendSocket[i] = InstallSendSocket (adhocInterfaces.GetAddress (i), adhocNodes.Get (i));
    }
*/

 // Create and bind the socket...
        //std::cout << "text=" << std::endl; 
   //cout<<"the size is: "<<f1r.size()<<std::endl;
  
  //f[0].SendSocket[f1r[len1]]->SetSendCallback (MakeCallback(&flow::SendSrc,f[0]));
  //Simulator::Schedule (Seconds (0.1), &flow::SendSrc, f[0],f[0].SendSocket[f1r[len1]],f[0].SendSocket[f1r[len1]]->GetTxAvailable ());


std::stringstream ss;
  ss << nWifis;
  std::string nodes = ss.str ();

  std::stringstream ss2;
  ss2 << nodeSpeed;
  std::string sNodeSpeed = ss2.str ();

  std::stringstream ss3;
  ss3 << nodePause;
  std::string sNodePause = ss3.str ();

  std::stringstream ss4;
  ss4 << rate;
  std::string sRate = ss4.str ();
  AsciiTraceHelper ascii;
  MobilityHelper::EnableAsciiAll (ascii.CreateFileStream (tr_name + ".mob"));

  //Ptr<FlowMonitor> nFlowsmon;
  //FlowMonitorHelper nFlowsmonHelper;
  //nFlowsmon = nFlowsmonHelper.InstallAll ();


  NS_LOG_INFO ("Run Simulation.");

  CheckThroughput ();
  wifiPhy.EnablePcapAll ("wifiphy");

  Simulator::Stop (Seconds (TotalTime));
  Simulator::Run ();

  //nFlowsmon->SerializeToXmlFile ((tr_name + ".nFlowsmon").c_str(), false, false);

  Simulator::Destroy ();



  //experiment.Run (nFlows, txp, CSVfileName);

}





   

