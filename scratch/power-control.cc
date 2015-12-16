/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 Dalian University of Technology
 * Copyright (c) 2014 University of Electronic Science and Technology of China
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
 * Author: Junling Bu <linlinjavaer@gmail.com>
 * Author: Weizhe Zhang <wzz_zhang@163.com>
 */
#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/node-container.h"
#include "ns3/net-device-container.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/command-line.h"
#include "ns3/mobility-model.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include "ns3/string.h"
#include "ns3/double.h"
#include "ns3/object-map.h"
#include "ns3/regular-wifi-mac.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/llc-snap-header.h"
#include "ns3/wave-net-device.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/core-module.h"
#include "ns3/yans-wifi-phy.h"
#include "ns3/seq-ts-header.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("highway-static");

class StatsTag : public Tag
{
public:
  StatsTag (void)
    : m_nodeId (0),
      m_sendTime (Seconds (0)),
      m_posX (0),
      m_posY (0)
  {
  }
  StatsTag (uint32_t nodeId, Time sendTime, double posX, double posY)
    : m_nodeId (nodeId),
      m_sendTime (sendTime),
      m_posX (posX),
      m_posY (posY)
  {
  }
  virtual ~StatsTag (void)
  {
  }

  uint32_t GetNodeId (void)
  {
    return m_nodeId;
  }

  Time GetSendTime (void)
  {
    return m_sendTime;
  }
  double GetX (void)
  {
    return m_posX;
  }
  double GetY (void)
  {
    return m_posY;
  }
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);
  virtual void Print (std::ostream &os) const;

private:
  uint32_t m_nodeId;
  Time m_sendTime;
  double m_posX;
  double m_posY;
};
TypeId
StatsTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::StatsTag")
    .SetParent<Tag> ()
    .AddConstructor<StatsTag> ()
  ;
  return tid;
}
TypeId
StatsTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
uint32_t
StatsTag::GetSerializedSize (void) const
{
  return sizeof (uint32_t) + sizeof (uint64_t) + sizeof (uint64_t) + sizeof (uint64_t);
}
void
StatsTag::Serialize (TagBuffer i) const
{
  i.WriteU32 (m_nodeId);
  i.WriteU64 (m_sendTime.GetMicroSeconds ());
  i.WriteU64 (m_posX);
  i.WriteU64 (m_posY);
}
void
StatsTag::Deserialize (TagBuffer i)
{
  m_nodeId = i.ReadU32 ();
  m_sendTime = MicroSeconds (i.ReadU64 ());
  m_posX = i.ReadU64 ();
  m_posY = i.ReadU64 ();
}
void
StatsTag::Print (std::ostream &os) const
{
  os << "node=" << m_nodeId << " sendTime=" << m_sendTime
     << " posX:" << m_posX << " posY:" << m_posY;
}



class NeighborTag : public Tag
{
public:
  NeighborTag (void)
    : m_num(0)
  {
  }

  NeighborTag (uint32_t num)
    : m_num(num)
  {
  }

  virtual ~NeighborTag (void)
  {
  }

  uint32_t GetNum (void)
  {
    return m_num;
  }

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);
  virtual void Print (std::ostream &os) const;

private:
  uint32_t m_num;
};
TypeId
NeighborTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NeighborTag")
    .SetParent<Tag> ()
    .AddConstructor<NeighborTag> ()
  ;
  return tid;
}
TypeId
NeighborTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
uint32_t
NeighborTag::GetSerializedSize (void) const
{
  return sizeof (uint32_t);
}
void
NeighborTag::Serialize (TagBuffer i) const
{
  i.WriteU32 (m_num);
}
void
NeighborTag::Deserialize (TagBuffer i)
{
  m_num = i.ReadU32 ();
}
void
NeighborTag::Print (std::ostream &os) const
{
  os << "num=" << m_num ;
}


const static uint16_t WSMP_PROT_NUMBER = 0x88DC;

bool Configure (int argc, char **argv);
void Usage (void);
int Run (void);
void Stats (void);

void CreateWaveNodes (uint32_t delta);
void SendWsmpPackets (Ptr<WaveNetDevice> sender, uint32_t channelNumber);
bool Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
void Init (int argc, char **argv);
void InitStats (void);
void Stats (uint32_t randomNumber);
//void StatQueuedPackets (void);
//void StatSingleQueue (Ptr<WaveEdcaTxopN> edca);
void GetPosition(Ptr<Node> node);
//static void CourseChange (std::ostream *os, std::string foo, Ptr<const MobilityModel> model);

NodeContainer nodes;
NetDeviceContainer devices; 
uint32_t nodesNumber;
uint32_t frequencySafety;
uint32_t simulationTime;
uint32_t sizeSafety;
double delta;
Ptr<UniformRandomVariable> rngSafety;
Ptr<UniformRandomVariable> rngOther;
Ptr<UniformRandomVariable> rngNodes;
uint32_t safetyPacketID;
  // we will check whether the packet is received by the neighbors
  // in the transmission range

//std::map<std::pair<uint32_t,Time> , std::vector<uint32_t> *> broadcastPackets;
std::map<double, uint32_t> receiveNum;
std::map<double, uint32_t> receiversNum;     
std::map<uint32_t, uint32_t> statsDelay;

std::map<uint32_t, std::map<uint32_t, uint32_t> > receiveStats;

uint32_t receiveTotal;
uint32_t queueSafety;
uint64_t timeSafety;        
std::ofstream outfile;
std::string traceFile;
std::string sumoData;


void
Init ()
{
  nodesNumber = 5000 / delta;
  //nodesNumber = 200;
  frequencySafety = 10;         // 10Hz, 100ms send one safety packet
  simulationTime = 10;          // make it run 100s
  sizeSafety = 500;             // 100 bytes small size
  safetyPacketID  = 0;
  //receiveSafety  = 0;
  queueSafety = 0;
  timeSafety = 0;
  //sumoData = "highway.mobility.tcl";
  //traceFile = "state.log"; 
  for (uint32_t i=0; i<nodesNumber; ++i)
  {
    for (uint32_t j=0; j<nodesNumber; ++j)
    {
      if (i == j)
      {
        continue;
      }
      receiveStats[i][j] = 0;
    }
  }

}


/************************************************************************
* Configure channel paremeters
*************************************************************************/
void 
SetChannel(void)
{
  YansWifiChannelHelper waveChannel;// = YansWifiChannelHelper::Default ();
  waveChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  waveChannel.AddPropagationLoss ("ns3::TwoRayGroundPropagationLossModel");
  YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
  wavePhy.Set("TxPowerLevels", UintegerValue (9));
  wavePhy.Set("TxPowerStart",  DoubleValue (-20));
  wavePhy.Set("TxPowerEnd",  DoubleValue (30));
  wavePhy.Set("ChannelNumber",  UintegerValue (CCH));
  wavePhy.SetChannel (waveChannel.Create ());
  wavePhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();
  devices = waveHelper.Install (wavePhy, waveMac, nodes);
  //wavePhy.EnablePcap ("wave-simple-device", devices);
}


/************************************************************************
* Create wave nodes and import traffic data from sumo software.
*************************************************************************/
void
CreateWaveNodes (void)
{
  //RngSeedManager::SetSeed (2);
  //RngSeedManager::SetRun (17);

  //nodes = NodeContainer ();
  //nodes.Create (nodesNumber);                                       //create wave nodes
  //Ns2MobilityHelper ns2 = Ns2MobilityHelper (sumoData);             //derive sumo data
  //ns2.Install ();
  rngNodes = CreateObject<UniformRandomVariable> ();
  rngNodes->SetStream (5);
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  //int j = 0;
  for(int i = 1; i <= 1000; ++i)
  {
    int x = i * 5;
    //int vehNum = 0;
    RngSeedManager::SetSeed (6+i);
    RngSeedManager::SetRun (7+i);
    
    //double uniform_var = rngNodes->GetValue(0,1);
    //double p = (double)x/25000.0 + 1.0/40.0;
    int d = delta;
    if(x%d == 0)
    {
      //++nodesNumber;
      //std::cout << x << " " << ++j << std::endl;
      positionAlloc->Add (Vector (x, 0.0, 0.0));
    }
  }
  /////////////////////////////////std::cout << nodesNumber << std::endl;
  nodes = NodeContainer ();
  nodes.Create (nodesNumber);      
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);

  // Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
  //              MakeBoundCallback (&CourseChange, &outfile));
  for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i)
  {
    Ptr<Node> node_src = (*i);
    Ptr<MobilityModel> model_src = node_src->GetObject<MobilityModel> ();
    Vector pos_src = model_src->GetPosition ();
    if((pos_src.x < 1000) || (pos_src.x > 4000))
    {
      continue;
    }
    for(NodeContainer::Iterator j = nodes.Begin (); j != nodes.End (); ++j)    
    {
      
      if(i == j)
      {
        continue;
      }
      Ptr<Node> node_dest = (*j);
      Ptr<MobilityModel> model_dest = node_dest->GetObject<MobilityModel> ();  
      double distance = model_src->GetDistanceFrom (model_dest);
      if(distance <= 1000)
      {
        receiversNum[distance] += (simulationTime * frequencySafety); 
      }
      if(distance <= 500)
      {
        receiveTotal += (simulationTime * frequencySafety);    
      }
    }  
  }

  for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i)
  {
    Ptr<Node> node = (*i);
    Ptr<MobilityModel> model = node->GetObject<MobilityModel> ();
    Vector pos = model->GetPosition ();
    NS_LOG_DEBUG ( node->GetId() <<" position: " << pos);
  }
  SetChannel();                                                     //set channel paremeter
  for (uint32_t i = 0; i != devices.GetN (); ++i)
  {
    devices.Get (i)->SetReceiveCallback (MakeCallback (&Receive));
  }

 
}


void
CalculateTxPower ()
{
  for (uint32_t i=0; i<nodesNumber; ++i)
  {
    for (uint32_t j=0; j<nodesNumber; ++j)
    {
      if (i == j)
      {
        continue;
      }
      Time now = Now ();
      
      std::cout <<"time:" << now.GetSeconds() << " node " << i << " receive "<< j << ": " << receiveStats[i][j] << std::endl;
      receiveStats[i][j] = 0;
    }
  }
}

/***************************************************************************
* Receive a packet. To calculate PRR,receiveTotal++ when function is called 
* To calculate PDR,To calculate PDR,receiveSafety++ only if every neighbors
* of senders receive the packet.
****************************************************************************/
bool
Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  //////////////////////////////std::cout << "receive" << std::endl;
  StatsTag tag;
  double delay;
  bool result;
  uint32_t src_id;
  uint32_t dest_id;
  result = pkt->FindFirstMatchingByteTag (tag);
  if (!result)
  {
    NS_FATAL_ERROR ("the packet here shall have a stats tag");
  }
  Time now = Now ();
  Time sendTime = tag.GetSendTime ();
  //uint32_t packetId = tag.GetPacketId (); 
  delay = (double) (now - sendTime).GetMicroSeconds () / 1000;
  delay = std::ceil(delay);
  //std::cout << delay << std::endl;
  
  Vector src_pos = Vector (tag.GetX (), tag.GetY (),0);
  Ptr<Node> node = dev->GetNode();

  Ptr<MobilityModel> model = node->GetObject<MobilityModel> ();
  Vector dest_pos = model->GetPosition ();
   // std::cout << src_pos << " 2" << std::endl;

  src_id = tag.GetNodeId();
  dest_id = node->GetId();
  receiveStats[dest_id][src_id]++;

  if((src_pos.x <= 4000) && (src_pos.x >= 1000))
  {  
    if(src_pos.y == dest_pos.y)
    {
      double distance = CalculateDistance (dest_pos, src_pos);

      ++receiveNum[distance]; 
      //std::cout << receiveNum[distance] << " " << receiversNum[distance] <<std::endl;
      //++statsDelay[delay];
      if((delay <= 50)&&(distance <= 500))
      {
        ++statsDelay[delay];
      }
    }

  }
  Ptr<WaveNetDevice> wave_dev = DynamicCast<WaveNetDevice> (dev);
  uint32_t var = wave_dev->GetLocalDensity() + now.GetSeconds() + node->GetId();
  wave_dev->CalculateLocalDensity(var);

   // local_density = CalculateDensity ();
   // dev.SetLocalDensity(local_density);
  return true;
}
 
void
SendWsmpPackets (Ptr<WaveNetDevice> sender, uint32_t channelNumber)
{
  Time now = Now ();
  
  Ptr<Packet> packet = Create<Packet> (sizeSafety);
  Ptr<Node> src = sender->GetNode();
  Ptr<MobilityModel> model_src = src->GetObject<MobilityModel> ();                     
  Vector src_pos = model_src->GetPosition (); 
  StatsTag tag = StatsTag (safetyPacketID, now, src_pos.x, src_pos.y);
  /////////////////////////std::cout << channelNumber << std::endl;
  packet->AddByteTag (tag);
  if((src_pos.x <= 4000) && (src_pos.x >= 1000))
  {
   
     safetyPacketID++;
  }
  ///////////////////////////////std::cout <<"Time :"<<Now().GetMilliSeconds() << "ms" << std::endl;
  // SeqTsHeader seqTs2;
  // seqTs2.SetSeq (1);
  // packet->AddHeader (seqTs2);
  const Address dest = Mac48Address::GetBroadcast ();
  WifiMode wave_mode = WifiModeFactory::CreateWifiMode ("OfdmRate6MbpsBW10MHz",
                                     WIFI_MOD_CLASS_OFDM,
                                     true,
                                     10000000, 6000000,
                                     WIFI_CODE_RATE_1_2,
                                     4);  
  //wifi_mode = WifiMode();
  //std::cout << wifi_mode.GetCodeRate() << wifi_mode.  
  uint8_t txPower = sender->CalculateTxPower();
  txPower = 100;
  TxInfo info = TxInfo (channelNumber, 7, wave_mode, 0, txPower);  
  //std::cout << "nodeID: " << sender->GetNode()->GetId() << " time: "<< now.GetMicroSeconds() << " txPower " << (int32_t)txPower << std::endl;
  sender->SendX (packet, dest, WSMP_PROT_NUMBER, info);
  

}


void
InitStats (void)
{
 // used for sending packets randomly
  rngSafety = CreateObject<UniformRandomVariable> ();
  rngSafety->SetStream (1);
  rngOther = CreateObject<UniformRandomVariable> ();
  rngOther->SetStream (3);

  safetyPacketID = 0;
  queueSafety = 0;
  timeSafety = 0;
  receiveTotal = 0;
  receiveNum.clear();
  receiversNum.clear();
  statsDelay.clear();
 

  //Simulator::ScheduleDestroy (&StatQueuedPackets);
}



/***************************************************************************
* Configure the channel. We use the continuous access in CCH channel define
* in IEEE 1609.4.
****************************************************************************/
void
Configuration (void)
{ 
  NetDeviceContainer::Iterator i;
  for (i = devices.Begin (); i != devices.End (); ++i)
  {
    Ptr<WaveNetDevice> sender = DynamicCast<WaveNetDevice> (*i);
    
    for (uint32_t time = 0; time != simulationTime; ++time)
    {
      if (time != 0)
      {
        Simulator::Schedule (Seconds (time), CalculateTxPower);
      }
      for (uint32_t sends = 0; sends != frequencySafety; ++sends)
      {
        //Simulator::Schedule (Seconds (time), CalculateTxPower, sender, CCH);   
        Simulator::Schedule (Seconds (rngSafety->GetValue (time, time + 1)), SendWsmpPackets, sender, CCH);
      }
      
    }
  }

}


void
Stats (void)
{
  // first show stats information
  NS_LOG_UNCOND (" safety packet: ");
  NS_LOG_UNCOND ("  sends = " << safetyPacketID);
  //NS_LOG_UNCOND ("  queues = " << queueSafety);
  // second show performance result
  NS_LOG_UNCOND (" performance result:");
  std::map<double,uint32_t>::iterator i;
  std::map<uint32_t, uint32_t>::iterator j;
  for(i = receiversNum.begin(); i != receiversNum.end(); ++i)
  {
     double distance = i->first;
     //std::cout << distance << "m " << receiveNum[distance] << " " << i->second << std::endl;
     //NS_LOG_UNCOND (" distance:" << i->first  << "receive rate:" << (double)(receiveNum[distance])/(i->second));
     std::cout<<" distance:" << i->first  << " receive rate:" << (double)(receiveNum[distance])/(i->second)<<std::endl;;
  }

  for(uint32_t k = 1; k < 50; ++k)
  {
    statsDelay[k+1] += statsDelay[k];
    //NS_LOG_UNCOND (" delay:" << j << " receiveNum:" << statsDelay[j]);
  }
  for(j = statsDelay.begin (); j != statsDelay.end (); ++j)
  {
    //NS_LOG_UNCOND (" delay:" << j->first << " receive rate:" << (double) (j->second) / (double)receiveTotal);
    std::cout<<" delay:" << j->first << " receive rate:" << (double) (j->second) / (double)receiveTotal<<std::endl;
  }
  //std::cout << "safetyPacketID: " << safetyPacketID << std::endl;
  //NS_LOG_UNCOND ("  safetyPDR = " << safetyPDR << " safetyPRR = " << safetyPRR);
  //double delaySafety = timeSafety / receiveS  fety / 1000.0;
  //NS_LOG_UNCOND ("  delaySafety = " << delaySafety << "ms");
}

void
GetTime(void)
{
  time_t now;   
  struct tm *timenow;       
  time(&now);   
  timenow = localtime(&now);   
  NS_LOG_UNCOND ("current time is  " << asctime(timenow));
}

int
Run (void)
{
  
  //NS_LOG_UNCOND ("configuration:");
  //NS_LOG_UNCOND ("configuration: delta: " << delta << "m density: " << 1000/delta << " vehicles/lane/km  total: 2km");
  //NS_LOG_UNCOND ("nodes number: " << nodesNumber << ", simulation time: 30s, safety packet size: 200bytes");
  //outfile.open(traceFile);
  //transmissionRange = GetTransmissionRange();          //get transmission range
  //transmissionRange = 180;
  
  Init ();
  InitStats ();
  //NS_LOG_UNCOND ("nodes number: " << nodesNumber << ", simulation time: " << simulationTime << "s, safety packet size: " << sizeSafety << "bytes");
  CreateWaveNodes ();
  NS_LOG_UNCOND ("nodes number: " << nodesNumber << ", simulation time: " << simulationTime << "s, safety packet size: " << sizeSafety << "bytes");
  Configuration ();
  Simulator::Stop (Seconds (simulationTime));
  Simulator::Run ();
  Simulator::Destroy ();
  //Stats ();
  GetTime();
  //outfile.close();
  return 0;
}

int 
main()
{
  //LogComponentEnable ("power-control", LOG_LEVEL_DEBUG);
  GetTime();
  // delta = 1000/25; 
  // Run();
   //delta = 1000/100;
  // Run();
  //delta = 1000/200;
  delta = 200;
  Run();
}
 