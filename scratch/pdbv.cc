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
#include "ns3/event-id.h"

#define LAMBDA 1000
#define MAXLEN 620
//#define THRESHOLD 70
const double e = 2.718281828459;
const int rxThreshold = -94;
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


class PacketTag : public Tag
{
public:
  PacketTag (void):
    m_packetId (0),
    m_hop (0),
    m_power (0)
  {
  }
  PacketTag (uint32_t packetId, uint32_t hop, uint32_t power):
    m_packetId (packetId),
    m_hop (hop),
    m_power (power)
  {
  }
  virtual ~PacketTag (void)
  {
  }
  uint32_t GetPacketId (void)
  {
    return m_packetId;
  }
  uint32_t GetHop (void)
  {
    return m_hop;
  }
  uint32_t GetPower (void)
  {
    return m_power;
  }
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);
  virtual void Print (std::ostream &os) const;

private:
  uint32_t m_packetId;
  uint32_t m_hop;
  uint32_t m_power;
};
TypeId
PacketTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PacketTag")
    .SetParent<Tag> ()
    .AddConstructor<PacketTag> ()
  ;
  return tid;
}
TypeId
PacketTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
uint32_t
PacketTag::GetSerializedSize (void) const
{
  return sizeof (uint32_t) + sizeof (uint32_t) + sizeof (uint32_t);
}
void
PacketTag::Serialize (TagBuffer i) const
{
  i.WriteU32 (m_packetId);
  i.WriteU32 (m_hop);
  i.WriteU32 (m_power);
}
void
PacketTag::Deserialize (TagBuffer i)
{
  m_packetId = i.ReadU32 ();
  m_hop = i.ReadU32 ();
  m_power = i.ReadU32 ();
}
void
PacketTag::Print (std::ostream &os) const
{
  os << "packet=" << m_packetId << " hop=" << m_hop
     << " power:" << m_power;
}

class SrcNodeTag : public Tag
{
public:
  SrcNodeTag (void):
    m_nodeId (0),
    m_posX (0),
    m_posY (0)
  {
  }
  SrcNodeTag (uint32_t nodeId, double posX, double posY):
    m_nodeId (nodeId),
    m_posX (posX),
    m_posY (posY)
  {
  }
  virtual ~SrcNodeTag (void)
  {
  }
  uint32_t GetNodeId (void)
  {
    return m_nodeId;
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
  double m_posX;
  double m_posY;
};
TypeId
SrcNodeTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SrcNodeTag")
    .SetParent<Tag> ()
    .AddConstructor<SrcNodeTag> ()
  ;
  return tid;
}
TypeId
SrcNodeTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
uint32_t
SrcNodeTag::GetSerializedSize (void) const
{
  return sizeof (uint32_t) + sizeof (double) + sizeof (double);
}
void
SrcNodeTag::Serialize (TagBuffer i) const
{
  i.WriteU32 (m_nodeId);
  i.WriteU64 (m_posX);
  i.WriteU64 (m_posY);
}
void
SrcNodeTag::Deserialize (TagBuffer i)
{
  m_nodeId = i.ReadU32 ();
  m_posX = i.ReadU64 ();
  m_posY = i.ReadU64 ();
}
void
SrcNodeTag::Print (std::ostream &os) const
{
  os << "node:" << m_nodeId << " x:" << m_posX
     << " y:" << m_posY;
}

class RelayNodeTag : public Tag
{
public:
  RelayNodeTag (void):
    m_nodeId (0),
    m_posX (0),
    m_posY (0)
  {
  }
  RelayNodeTag (uint32_t nodeId, double posX, double posY):
    m_nodeId (nodeId),
    m_posX (posX),
    m_posY (posY)
  {
  }
  virtual ~RelayNodeTag (void)
  {
  }
  uint32_t GetNodeId (void)
  {
    return m_nodeId;
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
  double m_posX;
  double m_posY;
};
TypeId
RelayNodeTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RelayNodeTag")
    .SetParent<Tag> ()
    .AddConstructor<RelayNodeTag> ()
  ;
  return tid;
}
TypeId
RelayNodeTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
uint32_t
RelayNodeTag::GetSerializedSize (void) const
{
  return sizeof (uint32_t) + sizeof (double) + sizeof (double);
}
void
RelayNodeTag::Serialize (TagBuffer i) const
{
  i.WriteU32 (m_nodeId);
  i.WriteU64 (m_posX);
  i.WriteU64 (m_posY);
}
void
RelayNodeTag::Deserialize (TagBuffer i)
{
  m_nodeId = i.ReadU32 ();
  m_posX = i.ReadU64 ();
  m_posY = i.ReadU64 ();
}
void
RelayNodeTag::Print (std::ostream &os) const
{
  os << "last hop node:" << m_nodeId << " x:" << m_posX
     << " y:" << m_posY;
}

class TimeTag : public Tag
{
public:
  TimeTag (void):
    m_sendTime (Seconds (0)),
    m_validTime (0)
  {
  }
  TimeTag (Time sendTime, uint32_t validTime):
    m_sendTime (sendTime),
    m_validTime (validTime)
  {
  }
  virtual ~TimeTag (void)
  {
  }
  Time GetSendTime (void)
  {
    return m_sendTime;
  }

  uint32_t GetValidTime (void)
  {
    return m_validTime;
  }
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);
  virtual void Print (std::ostream &os) const;

private:
  Time m_sendTime;
  uint32_t m_validTime;
};
TypeId
TimeTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TimeTag")
    .SetParent<Tag> ()
    .AddConstructor<TimeTag> ()
  ;
  return tid;
}
TypeId
TimeTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
uint32_t
TimeTag::GetSerializedSize (void) const
{
  return sizeof (double) + sizeof (uint32_t);
}
void
TimeTag::Serialize (TagBuffer i) const
{
  i.WriteU64 (m_sendTime.GetMicroSeconds ());
  i.WriteU32 (m_validTime);
  
}
void
TimeTag::Deserialize (TagBuffer i)
{
  m_sendTime = MicroSeconds (i.ReadU64 ());
  m_validTime = i.ReadU32 ();
}
void
TimeTag::Print (std::ostream &os) const
{
  os << "sendTime" << m_sendTime.GetMilliSeconds() << " valid Time:" << m_validTime;
}

class RangeTag : public Tag
{
public:
  RangeTag (void):
    m_dirX (0),
    m_dirY (0),
    m_distance (0)
  {
  }
  RangeTag (double dirX, double dirY, uint32_t distance):
    m_dirX (dirX),
    m_dirY (dirY),
    m_distance (distance)
  {
  }
  virtual ~RangeTag (void)
  {
  }
  double GetDirX (void)
  {
    return m_dirX;
  }
  double GetDirY (void)
  {
    return m_dirX;
  }
  uint32_t GetDistance (void)
  {
    return m_distance;
  }
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);
  virtual void Print (std::ostream &os) const;

private:
  double m_dirX;
  double m_dirY;
  uint32_t m_distance;
};
TypeId
RangeTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RangeTag")
    .SetParent<Tag> ()
    .AddConstructor<RangeTag> ()
  ;
  return tid;
}
TypeId
RangeTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
uint32_t
RangeTag::GetSerializedSize (void) const
{
  return sizeof (double) + sizeof (double) + sizeof (uint32_t);
}
void
RangeTag::Serialize (TagBuffer i) const
{
  i.WriteU64 (m_dirX);
  i.WriteU64 (m_dirY);
  i.WriteU32 (m_distance);
  
}
void
RangeTag::Deserialize (TagBuffer i)
{
  m_dirX = i.ReadU64 ();
  m_dirY = i.ReadU64 ();
  m_distance = i.ReadU32 ();
}
void
RangeTag::Print (std::ostream &os) const
{
  os << "dir X:" << m_dirX << " dir Y:" << m_dirY
     << " distance:" << m_distance;
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
bool ReceiveMultiHopPacket (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
bool ReceiveWsmpPacket(Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);void ProduceMultihopPacket (Ptr<WaveNetDevice> sender);
bool ReceiveAck (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
void SendMultihopPacket (Ptr<WaveNetDevice> sender,  Ptr<Packet> packet);
void RetransmissionMultihopPacket (Ptr<WaveNetDevice> sender,  Ptr<Packet> packet);
void GetPosition(Ptr<Node> node);
double CalTxRange (int txpower);
//static void CourseChange (std::ostream *os, std::string foo, Ptr<const MobilityModel> model);

NodeContainer nodes;
NetDeviceContainer devices; 
uint32_t nodesNumber;
uint32_t frequencySafety;
uint32_t simulationTime;
uint32_t sizeSafety;
uint32_t sizeRelay;
uint32_t sizeAck;
double delta;
Ptr<UniformRandomVariable> rngSafety;
Ptr<UniformRandomVariable> rngOther;
Ptr<UniformRandomVariable> rngNodes;
Ptr<UniformRandomVariable> rngWaitTime;
uint32_t safetyPacketID;
uint32_t relayPacketID;

std::map<uint32_t, std::map<uint32_t, uint32_t> > receiveStats;
std::map<Ptr<Node>, std::map<Ptr<Node>, uint32_t> > receiveStat;
std::map<Ptr<Node>, std::map<Ptr<Node>, double> > receiveDistance;
std::map<Ptr<Node>, double> distanceMax;
std::map<Ptr<Node>, double> nodeDensity;
std::map<Ptr<Node>, int> nodePower;
std::map<Ptr<Node>, std::map<Ptr<Node>, uint32_t> > nodeRelay;
//std::map<Ptr<Node>, std::vector<uint32_t> > nodeRelayPacket;
std::map<Ptr<Node>, std::map<uint32_t, EventId> > nodeRelayEvent;
std::map<Ptr<Node>, std::map<uint32_t, EventId> > nodeRetransmissionEvent;

std::map<uint32_t, uint32_t> totalDelay;
std::map<uint32_t, uint32_t> meanDelay;
std::map<uint32_t, uint32_t> receiveNum;
std::map<uint32_t, uint32_t> relayNum;
std::map<uint32_t, uint32_t> validRelayNum;
std::map<Ptr<Node>, std::map<uint32_t, uint32_t> > nodeReceiveNum;

int txPower;
int configPower;
double density;
uint32_t receiversThres;
bool powerControl;
int retrNum;
  // we will check whether the packet is received by the neighbors
  // in the transmission range

//std::map<std::pair<uint32_t,Time> , std::vector<uint32_t> *> broadcastPackets;

double throughput;      
std::ofstream outfile;
std::string traceFile;
std::string sumoData;

std::ofstream fout("position.log");

void
Init ()
{
  //nodesNumber = 5000 / delta;
  nodesNumber = 0;
  frequencySafety = 10;         // 10Hz, 100ms send one safety packet
  simulationTime = 20;          // make it run 100s
  sizeSafety = 400;             // 100 bytes small size
  sizeRelay = 1200;
  sizeAck = 50;
  safetyPacketID  = 0;
  relayPacketID = 0;
  //receiveSafety  = 0;

  txPower = 30;

  nodeRelay.clear();
  nodeRelayEvent.clear();
  nodeRetransmissionEvent.clear();
  totalDelay.clear();
  meanDelay.clear();
  receiveNum.clear();
  relayNum.clear();
  validRelayNum.clear();
  nodeReceiveNum.clear();
}


/************************************************************************
* Configure channel paremeters
*************************************************************************/
void 
SetChannel(void)
{
  YansWifiChannelHelper waveChannel;// = YansWifiChannelHelper::Default ();
  waveChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  waveChannel.AddPropagationLoss ("ns3::TwoRayGroundPropagationLossModel",
                            "Frequency", DoubleValue (5.89e9),
                            "HeightAboveZ", DoubleValue (0.5));
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
// void
// CreateWaveNodes (void)
// {
//   //RngSeedManager::SetSeed (2);
//   //RngSeedManager::SetRun (17);

//   //nodes = NodeContainer ();
//   //nodes.Create (nodesNumber);                                       //create wave nodes
//   //Ns2MobilityHelper ns2 = Ns2MobilityHelper (sumoData);             //derive sumo data
//   //ns2.Install ();
//   rngNodes = CreateObject<UniformRandomVariable> ();
//   rngNodes->SetStream (5);
//   MobilityHelper mobility;
//   Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
//   //int j = 0;
//   double p = density / 800;
//   for(int i = 1; i <= 400; ++i)
//   {
//     int x = i * 5;
//     //int vehNum = 0;
//     RngSeedManager::SetSeed (6+i);
//     RngSeedManager::SetRun (7+i);

//     //int d = delta;
//     for (uint32_t j = 0; j < 4; ++j)
//     {
//       double uniform_var = rngNodes->GetValue(0,1);
//       if(uniform_var <= p)
//       {
//         ++nodesNumber;
//         //std::cout << x << " " << ++j << std::endl;
//         positionAlloc->Add (Vector (x, j*3, 0.0));
//       }
//     }
//   }
//   for(int i = 401; i <= 500; ++i)
//   {
//     int x = i * 5;
//     //int vehNum = 0;
//     RngSeedManager::SetSeed (6+i);
//     RngSeedManager::SetRun (7+i);

//     //int d = delta;
//     for (uint32_t j = 0; j < 4; ++j)
//     {
//       double uniform_var = rngNodes->GetValue(0,1);
//       if(uniform_var <= 1)
//       {
//         ++nodesNumber;
//         //std::cout << x << " " << ++j << std::endl;
//         positionAlloc->Add (Vector (x, j*3, 0.0));
//       }
//     }
//   }
//   for(int i = 501; i <= 1000; ++i)
//   {
//     int x = i * 5;
//     //int vehNum = 0;
//     RngSeedManager::SetSeed (6+i);
//     RngSeedManager::SetRun (7+i);

//     //int d = delta;
//     for (uint32_t j = 0; j < 4; ++j)
//     {
//       double uniform_var = rngNodes->GetValue(0,1);
//       //std::cout << (double)(1000-i)/500 << std::endl;
//       if(uniform_var <= (double)(1000-i)/500)
//       {
//         ++nodesNumber;
//         //std::cout << x << " " << ++j << std::endl;
//         positionAlloc->Add (Vector (x, j*3, 0.0));
//       }
//     }
//   }
//   /////////////////////////////////std::cout << nodesNumber << std::endl;
//   nodes = NodeContainer ();
//   nodes.Create (nodesNumber);      
//   mobility.SetPositionAllocator (positionAlloc);
//   mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
//   mobility.Install (nodes);

//   // Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
//   //              MakeBoundCallback (&CourseChange, &outfile));

//   fout << "###################################################################" << std::endl;

//   for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i)
//   {
//     Ptr<Node> node_src = (*i);
//     //init node configure
//     nodeDensity[node_src] = 0;          
//     nodePower[node_src] = 30;
//     distanceMax[node_src] = 0;
//     Ptr<MobilityModel> model_src = node_src->GetObject<MobilityModel> ();
//     Vector pos_src = model_src->GetPosition ();

//     fout << "node id: "<< node_src->GetId()
//          << " node position: " << pos_src << std::endl;

//     if((pos_src.x < 1000) || (pos_src.x > 4000))
//     {
//       continue;
//     }
 
//   }

//   SetChannel();                                                     //set channel paremeter
//   for (uint32_t i = 0; i != devices.GetN (); ++i)
//   {
//     devices.Get (i)->SetReceiveCallback (MakeCallback (&Receive));
//   }
//}

// void CreateWaveNodes (void)
// {


//   //RngSeedManager::SetSeed (2);
//   //RngSeedManager::SetRun (17);

//   //nodes = NodeContainer ();
//   //nodes.Create (nodesNumber);                                       //create wave nodes
//   //Ns2MobilityHelper ns2 = Ns2MobilityHelper (sumoData);             //derive sumo data
//   //ns2.Install ();
  
//   MobilityHelper mobility;
//   Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
//   //int j = 0;

//   for(int i = 1; i <= 10; ++i)
//   {
//     double x = i * 400;
//     //int vehNum = 0;
//     RngSeedManager::SetSeed (6+i);
//     RngSeedManager::SetRun (7+i);
//     positionAlloc->Add (Vector (x,0.0, 0.0));
//     ++nodesNumber;
//     //int d = delta;
    
//   }
//   /////////////////////////////////std::cout << nodesNumber << std::endl;
//   nodes = NodeContainer ();
//   nodes.Create (nodesNumber);      
//   mobility.SetPositionAllocator (positionAlloc);
//   mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
//   mobility.Install (nodes);

//   // Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
//   //              MakeBoundCallback (&CourseChange, &outfile));

//   fout << "###################################################################" << std::endl;

//   for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i)
//   {
//     Ptr<Node> node_src = (*i);
//     //init node configure
//     nodeDensity[node_src] = 0;          
//     nodePower[node_src] = 30;
//     distanceMax[node_src] = 0;
//     Ptr<MobilityModel> model_src = node_src->GetObject<MobilityModel> ();
//     Vector pos_src = model_src->GetPosition ();

//     fout << "node id: "<< node_src->GetId()
//          << " node position: " << pos_src << std::endl;
 
//   }

//   SetChannel();                                                     //set channel paremeter
//   for (uint32_t i = 0; i != devices.GetN (); ++i)
//   {
//     devices.Get (i)->SetReceiveCallback (MakeCallback (&Receive));
//   }


// }

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
  double p = density / 800;
  for(int i = 1; i <= 1000; ++i)
  {
    int x = i * 5;
    //int vehNum = 0;
    RngSeedManager::SetSeed (6+i);
    RngSeedManager::SetRun (7+i);

    //int d = delta;
    for (uint32_t i = 0; i < 4; ++i)
    {
      double uniform_var = rngNodes->GetValue(0,1);
      if(uniform_var <= p)
      {
        ++nodesNumber;
        //std::cout << x << " " << ++j << std::endl;
        positionAlloc->Add (Vector (x, i*3, 0.0));
      }
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

  fout << "###################################################################" << std::endl;

  for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i)
  {
    Ptr<Node> node_src = (*i);
    //init node configure
    nodeDensity[node_src] = 0;          
    nodePower[node_src] = 30;
    distanceMax[node_src] = 0;
    Ptr<MobilityModel> model_src = node_src->GetObject<MobilityModel> ();
    Vector pos_src = model_src->GetPosition ();

    fout << "node id: "<< node_src->GetId()
         << " node position: " << pos_src << std::endl;
 
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
  Time now = Now (); 
  //std::cout << "Time: " << now.GetSeconds() << "s" << std::endl;
  // double meanDensity = 0;
  // double total = 0;
  // double ii = 0;
  //int lastPower = (int)nodePower[nodes.Get(nodesNumber/2)];
  for (NodeContainer::Iterator i = nodes.Begin (); i != nodes.End (); ++i)
  {
    Ptr<Node> receiveNode = (*i);
    
    uint32_t neighborNum = 0;
    /// = receiveStat[receiveNode].size();
 
    Ptr<MobilityModel> model = receiveNode->GetObject<MobilityModel> ();
    //Vector receivePos = model->GetPosition ();
    
    //std::cout << "node id: "<< receiveNode->GetId() << " max distance: " << distanceMax[receiveNode] << " neighbors number: " << neighborNum << " local density: " << nodeDensity[receiveNode] << std::endl;


    std::vector<double> distanceVec;
    for (NodeContainer::Iterator j = nodes.Begin (); j != nodes.End (); ++j)
    {
      if(i == j)
      {
        continue;
      }
      Ptr<Node> sendNode = (*j);
      if ((receiveDistance[receiveNode][sendNode] <= distanceMax[receiveNode])&&(receiveStat[receiveNode][sendNode]>0))
      {
        // if(receiveDistance[receiveNode][sendNode] <= distanceMax[receiveNode])
        //   std::cout << "equal" << std::endl;
        neighborNum++;
        distanceVec.push_back(receiveDistance[receiveNode][sendNode]);
      }

    }

    sort(distanceVec.begin(), distanceVec.end());
    //distanceMax[receiveNode] = distanceVec.back();
    //std::cout << receiveNode->GetId() <<" "<< distanceMax[receiveNode] << " " << distanceVec.back() << std::endl;
    // if(receiveNode->GetId() == nodesNumber/2)
    // {
    //   for(std::vector<double>::iterator j = distanceVec.begin(); j != distanceVec.end(); ++j)
    //   {
    //     fout << *j << std::endl;
    //   }
      
    // }
    nodeDensity[receiveNode] = LAMBDA * neighborNum / (distanceMax[receiveNode] * 2);
    

    // if(receiveNode->GetId() == nodesNumber/2)
    // {
    //   std::cout << neighborNum << " " << distanceMax[receiveNode] << std::endl;
    // }
    // if ((receivePos.x > 1000)&&(receivePos.x < 4000))
    // {
    //   total += nodeDensity[receiveNode];
    //   ii++;
    // }

    double targetDistance;
    if (neighborNum > receiversThres)
    {
      targetDistance = distanceVec[receiversThres];                             
    }
    else
    {
      targetDistance = 1000 * receiversThres / nodeDensity[receiveNode] / 2;
    }
    nodePower[receiveNode] =  rxThreshold - 10 * std::log10((1.259*1.259*0.5*0.5*0.5*0.5) / (targetDistance*targetDistance*targetDistance*targetDistance));
    // if (receiveNode->GetId() == 121)
    // {
    //   std::cout << "neighbor num:" << neighborNum << " density:" << nodeDensity[receiveNode] << " target distance:" << targetDistance << " power: " << nodePower[receiveNode] << std::endl;
    // }
    if (nodePower[receiveNode] > 30)
    {
      nodePower[receiveNode] = 30;
    }
    else if (nodePower[receiveNode] < 0)
    {
      nodePower[receiveNode] = 0;
    }
    if (!powerControl)
    {
      nodePower[receiveNode] = configPower;
    }
    //std::cout << "node id: "<< receiveNode->GetId() << " target distance: " << targetDistance << " tx power: " << (int)nodePower[receiveNode] << std::endl;  
    receiveStat[receiveNode].clear();
    receiveDistance[receiveNode].clear();
    distanceVec.clear();
    distanceMax[receiveNode] = 0;
    //std::cout << receiveNode->GetId() << ": " << (int)nodePower[receiveNode] << std::endl;
  }

  //meanDensity = total / ii;

  // std::cout << "time: " << Now().GetSeconds() <<"s; tx power: "<< lastPower << "dbm;" << std::endl;
  // std::cout << "Mean Estimate density: " << meanDensity << "; Middle node density: " << nodeDensity[nodes.Get(nodesNumber/2)] << std::endl;
  //std::cout << nodeDensity[nodes.Get(nodesNumber/2)] << std::endl;

  //std::cout << "*************************" << std::endl;
}

double CalTxRange (int txpower)
{
  double temp = (double)(rxThreshold- txpower)/10.0;
  //std::cout << temp << std::endl;
  temp = std::pow(10,temp);
  temp = (1.259*1.259*0.5*0.5*0.5*0.5) / temp;
  temp = std::pow(temp, 0.25);
  temp *= 0.88;
  return temp;
}

bool
Receive (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  if (pkt->GetSize() > 500)
  {
    ReceiveMultiHopPacket (dev, pkt, mode, sender);
  }
  else if (pkt->GetSize() > 100)
  {
    ReceiveWsmpPacket (dev, pkt, mode, sender);
  }
  else if (pkt->GetSize() < 80)
  {
    ReceiveAck (dev, pkt, mode, sender);
  }
  return true;
}

bool
ReceiveAck (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  //std::cout << "receive ack" << std::endl;
  PacketTag packetTag;
  SrcNodeTag srcTag;
  RelayNodeTag relayTag;
  pkt->FindFirstMatchingByteTag (packetTag);
  pkt->FindFirstMatchingByteTag (srcTag);
  pkt->FindFirstMatchingByteTag (relayTag);
  uint32_t packetId = packetTag.GetPacketId ();
  Ptr<Node> node = dev->GetNode();
  
  uint32_t relayNodeId = relayTag.GetNodeId ();
  Ptr<Node> relayNode = nodes.Get(relayNodeId);

  Ptr<MobilityModel> relay_model = relayNode->GetObject<MobilityModel> ();
  Ptr<MobilityModel> model = node->GetObject<MobilityModel> ();
  Vector relay_pos = relay_model->GetPosition ();
  Vector pos = model->GetPosition ();
  std::map<uint32_t, EventId>::iterator it = nodeRelayEvent[node].find(packetId);
  std::map<uint32_t, EventId>::iterator jt = nodeRetransmissionEvent[node].find(packetId);
  // 
  
  if(relay_pos.x < pos.x)
  {
    return false;
  }
  // if (packetId == 10)
  // {
  //   std::cout << node->GetId() << " receive ack from " << relayNodeId << std::endl;
  // }
  //std::cout <<  node->GetId() << " receive ack from " << relayNodeId << std::endl;
  if(it != nodeRelayEvent[node].end())  //other node realay
  {
    //std::cout << "cancel " << packetId << std::endl;
    nodeRelayEvent[node][packetId].Cancel ();
    
    //nodeRelayEvent[node].erase(it);
    //std::cout << (double)now.GetMilliSeconds()/1000 << "s " << dest_id << " Cancel relay "<< std::endl;
    //return false;
  }
  if(jt != nodeRetransmissionEvent[node].end())  //cancel retransmission
  {
    nodeRetransmissionEvent[node][packetId].Cancel ();
    //nodeRetransmissionEvent[node].erase(jt);
    //std::cout << (double)now.GetMilliSeconds()/1000 << "s " << dest_id << " Cancel relay "<< std::endl;
    //std::cout << "cancel retransmission " << std::endl;
    //return false;
    // if ((packetId == 3) )//&& (node->GetId() > 160))
    // {
    //   std::cout << node->GetId() << " cancel retransmission, next hop is  " << relayNodeId << std::endl;
    // }
  }
  return true;
}

bool 
ReceiveMultiHopPacket (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  //std::cout << "receive packet" << std::endl;
  Time now = Now();
  PacketTag packetTag;
  SrcNodeTag srcTag;
  RelayNodeTag relayTag;
  TimeTag timeTag;
  RangeTag rangeTage;
  pkt->FindFirstMatchingByteTag (packetTag);
  pkt->FindFirstMatchingByteTag (srcTag);
  pkt->FindFirstMatchingByteTag (relayTag);
  pkt->FindFirstMatchingByteTag (timeTag);
  pkt->FindFirstMatchingByteTag (rangeTage);
  uint32_t hop = packetTag.GetHop();
  uint32_t packetId = packetTag.GetPacketId();
  uint32_t last_power = packetTag.GetPower();

  Time sendTime = timeTag.GetSendTime ();
  uint32_t delay = (now-sendTime).GetMilliSeconds();
  Vector last_pos;
  Vector src_pos;
  Ptr<Node> sendNode;
  uint32_t last_id = 0;
  if (hop == 0)
  {
    last_pos = Vector (srcTag.GetX (), srcTag.GetY (), 0);
    last_id = srcTag.GetNodeId ();
    sendNode = nodes.Get(last_id);
  }
  else if (hop > 0)
  {
    last_pos = Vector (relayTag.GetX (), relayTag.GetY (), 0);
    last_id = relayTag.GetNodeId ();
    sendNode = nodes.Get(last_id);
  }
  Ptr<Node> node = dev->GetNode();
  

  
  uint32_t this_power = nodePower[node];

  // if (nodeRelay[node][sendNode] == packetId) //retransmission
  // {
  //   std::cout << "already received this packet" << std::endl;
  //   return false;                                               //do not relay
  // }
  // else
  // {
  //   nodeRelay[node][sendNode] = packetId;
  // }
  src_pos = Vector (srcTag.GetX (), srcTag.GetY (), 0);;
  
  uint32_t dest_id = node->GetId ();

  Ptr<MobilityModel> model = node->GetObject<MobilityModel> ();
  Vector dest_pos = model->GetPosition ();
  double distance = CalculateDistance (dest_pos, last_pos);
  double src_distance = CalculateDistance (dest_pos, src_pos);
  //std::cout << "receiving" << std::endl;
  if((dest_pos.x < last_pos.x) || (dest_pos.x < src_pos.x))
  {
    return false;
  }
  
  if (src_distance > rangeTage.GetDistance())
  {
    receiveNum[packetId]++;
    totalDelay[packetId] += delay;
    validRelayNum[packetId] = hop;
    Ptr<Packet> ack = Create<Packet> (sizeAck);
    RelayNodeTag relayNTag = RelayNodeTag(dest_id,dest_pos.x,dest_pos.y);
//std::cout << "add ack tag" << std::endl;
    ack->AddByteTag (packetTag);
    ack->AddByteTag (srcTag);
    ack->AddByteTag (relayTag);
    const Address dest_addr = Mac48Address::GetBroadcast ();
    WifiMode wave_mode = WifiModeFactory::CreateWifiMode ("OfdmRate6MbpsBW10MHz",
                                     WIFI_MOD_CLASS_OFDM,
                                     true,
                                     10000000, 6000000,
                                     WIFI_CODE_RATE_1_2,
                                     4);
    //std::cout  << " ack send by " << dest_id << std::endl;
    TxInfo ackInfo = TxInfo (CCH, 7, wave_mode, 0, 30); 
    Ptr<WaveNetDevice> end_dev = DynamicCast<WaveNetDevice> (dev);
    end_dev->SendX (ack, dest_addr, WSMP_PROT_NUMBER, ackInfo);
    return false;                                               //do not relay
  }
  nodeReceiveNum[node][packetId]++;
  if (nodeReceiveNum[node][packetId] > 1)
  {
    return false;
  }
  std::map<uint32_t, EventId>::iterator it = nodeRelayEvent[node].find(packetId);
  if(it != nodeRelayEvent[node].end())  //other node realay
  {
    nodeRelayEvent[node][packetId].Cancel ();
    //nodeRelayEvent[node].erase(it);
    //std::cout << (double)now.GetMilliSeconds()/1000 << "s " << dest_id << " Cancel relay "<< std::endl;
    return false;
  }
//   if (dest_id == 9)
//   { 
//     //std::cout << "asdasd" << std::endl;
//     receiveNum[packetId]++;
//     totalDelay[packetId] += delay;
//     validRelayNum[packetId] = hop;
//     Ptr<Packet> ack = Create<Packet> (sizeAck);
//     RelayNodeTag relayNTag = RelayNodeTag(dest_id,dest_pos.x,dest_pos.y);
// //std::cout << "add ack tag" << std::endl;
//     ack->AddByteTag (packetTag);
//     ack->AddByteTag (srcTag);
//     ack->AddByteTag (relayTag);
//     const Address dest_addr = Mac48Address::GetBroadcast ();
//     WifiMode wave_mode = WifiModeFactory::CreateWifiMode ("OfdmRate6MbpsBW10MHz",
//                                      WIFI_MOD_CLASS_OFDM,
//                                      true,
//                                      10000000, 6000000,
//                                      WIFI_CODE_RATE_1_2,
//                                      4);
//     std::cout  << " ack send by " << dest_id << std::endl;
//     TxInfo ackInfo = TxInfo (CCH, 7, wave_mode, 0, 30); 
//     Ptr<WaveNetDevice> end_dev = DynamicCast<WaveNetDevice> (dev);
//     end_dev->SendX (ack, dest_addr, WSMP_PROT_NUMBER, ackInfo);
//     return false;     
//   } 

  //std::cout << "aaa" << std::endl;
  
  //nodePower[receiveNode] =  rxThreshold - 10 * std::log10((1.259*1.259*0.5*0.5*0.5*0.5) / (targetDistance*targetDistance*targetDistance*targetDistance));
  double Dm = CalTxRange(last_power) + CalTxRange(30);
  double Dr = distance + CalTxRange(this_power);
  uint32_t seg = std::ceil((Dm-Dr)/20);
  uint32_t CWmin = ((seg-1)*20/Dm) * 50;
  uint32_t CWmax = (Dr/Dm) * 50; 
  double waitTime = rngWaitTime->GetValue(CWmin,CWmax);
  //waitTime = rngWaitTime->GetValue(0,CWmax);
  // 
  if(waitTime <= 0)
  {
    return false;   //too far from last node
  }
  //std::cout <<(double)now.GetMilliSeconds()/1000 << "s " << dest_id << " prepare to relay, waite time is:" << waitTime << "ms hop:" << hop << std::endl;
  //std::cout << "ccc" << std::endl;
  Ptr<Packet> relayPacket = Create<Packet> (sizeRelay);
  //std::cout << "create new tag" << std::endl;
  PacketTag reproducePacketTag = PacketTag(packetId, ++hop, nodePower[node]);
  RelayNodeTag relayNodeTag = RelayNodeTag(dest_id,dest_pos.x,dest_pos.y);
  //relayPacket.AddByteTag (tag);
  //std::cout << node->GetId() << " receive packet" << std::endl;
  relayPacket->AddByteTag (reproducePacketTag); 
  relayPacket->AddByteTag (srcTag);
  relayPacket->AddByteTag (relayNodeTag);
  relayPacket->AddByteTag (timeTag);
  relayPacket->AddByteTag (rangeTage);
  Ptr<WaveNetDevice> realayDevice = DynamicCast<WaveNetDevice> (dev);
  //std::cout << dest_id << " prepare to relay " << last_id << std::endl;

  EventId relayEvent = Simulator::Schedule (MilliSeconds (waitTime), SendMultihopPacket, realayDevice, relayPacket);
  nodeRelayEvent[node][packetId] = relayEvent;
  // if (packetId == 6)
  // {
  //   std::cout << dest_id << " Receive a MultiHop Packet" << std::endl;
  // }

  
  //std::cout << dest_id << " Receive a MultiHop Packet" << std::endl;
  return true;
}
bool
ReceiveWsmpPacket (Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  //////////////////////////////std::cout << "receive" << std::endl;
  StatsTag tag;
  //std::cout << "receive wsmp" << std::endl;
  //double delay;
  bool result;
  //double packetSize;
  //uint32_t src_id;
  //uint32_t dest_id;
  //std::cout << "aa" << std::endl;
  result = pkt->FindFirstMatchingByteTag (tag);
  //std::cout << "bb" << std::endl;
  //packetSize = pkt.GetSize();
  throughput += sizeSafety;
  if (!result)
  {
    NS_FATAL_ERROR ("the packet here shall have a stats tag");
  }
  Time now = Now ();
  Time sendTime = tag.GetSendTime ();
  //uint32_t packetId = tag.GetPacketId (); 
  //delay = (double) (now - sendTime).GetMicroSeconds () / 1000;
  //std::cout << delay << std::endl;
  //delay = std::ceil(delay);
  //std::cout << delay << std::endl;
  
  Vector src_pos = Vector (tag.GetX (), tag.GetY (),0);
  Ptr<Node> node = dev->GetNode();
  Ptr<MobilityModel> model = node->GetObject<MobilityModel> ();
  Vector dest_pos = model->GetPosition ();
   // std::cout << src_pos << " 2" << std::endl;

  double distance = CalculateDistance (dest_pos, src_pos);
  uint32_t src_id = tag.GetNodeId();
  //uint32_t dest_id = node->GetId();
  //std::cout << dest_id << ":" << delay << std::endl;
  // if (distance < 20)
  // {
  //   std::cout << src_id << " " << dest_id << std::endl;
  // }
  Ptr<Node> src_node = nodes.Get(src_id);

  // if(dest_id == nodesNumber/2)
  // {
  //   if(distance > 500)
  //   {
  //     std::cout << src_node->GetId() << ": " << (int)nodePower[src_node] << " " <<  nodeDensity[src_node] << std::endl; 
  //   }
  // }
  // dest_id = node->GetId();
  // receiveStats[dest_id][src_id]++;
  receiveStat[node][src_node]++;
  // if (node->GetId() == 121)
  // {
  //   std::cout << " receive a wsmp message from " << src_id << std::endl;
  // }
  if ((distance > distanceMax[node])&&(receiveStat[node][src_node] > 3))
  {
    distanceMax[node] = distance;
  }
  receiveDistance[node][src_node] = distance;

 

   // local_density = CalculateDensity ();
   // dev.SetLocalDensity(local_density);
  return true;
}
void ProduceMultihopPacket (Ptr<WaveNetDevice> sender)
{
  Time now = Now ();
  Ptr<Packet> packet = Create<Packet> (sizeRelay);
  Ptr<Node> src = sender->GetNode();
  Ptr<MobilityModel> model_src = src->GetObject<MobilityModel> ();                     
  Vector src_pos = model_src->GetPosition ();
  uint32_t src_id = src->GetId (); 
  //std::cout << "cc" << std::endl;
  relayPacketID++;
  PacketTag packetTag = PacketTag(relayPacketID, 0, nodePower[src]);
  
  SrcNodeTag srcTag = SrcNodeTag(src_id, src_pos.x, src_pos.y);
  RelayNodeTag relayTag;
  TimeTag timeTag = TimeTag(now, 100000);
  RangeTag rangeTage = RangeTag(1,0,2500);
  packet->AddByteTag (packetTag); 
  packet->AddByteTag (srcTag);
  packet->AddByteTag (relayTag);
  packet->AddByteTag (timeTag);
  packet->AddByteTag (rangeTage);
  SendMultihopPacket (sender, packet);

}
void 
RetransmissionMultihopPacket (Ptr<WaveNetDevice> sender,  Ptr<Packet> packet)
{
  //std::cout << "retransmission" << std::endl;
  Ptr<Packet> retransmissionPacket = Create<Packet> (sizeRelay);
  Ptr<Node> src = sender->GetNode();
  PacketTag packetTag;
  SrcNodeTag srcTag;
  RelayNodeTag relayTag;
  TimeTag timeTag;
  RangeTag rangeTage;
  packet->FindFirstMatchingByteTag (packetTag);
  packet->FindFirstMatchingByteTag (srcTag);
  packet->FindFirstMatchingByteTag (relayTag);
  packet->FindFirstMatchingByteTag (timeTag);
  packet->FindFirstMatchingByteTag (rangeTage);
  retransmissionPacket->AddByteTag (packetTag);
  retransmissionPacket->AddByteTag (srcTag);
  retransmissionPacket->AddByteTag (relayTag);
  retransmissionPacket->AddByteTag (timeTag);
  retransmissionPacket->AddByteTag (rangeTage);
  uint32_t packetId = packetTag.GetPacketId();
  txPower = nodePower[src];
  const Address dest = Mac48Address::GetBroadcast ();
  WifiMode wave_mode = WifiModeFactory::CreateWifiMode ("OfdmRate6MbpsBW10MHz",
                                     WIFI_MOD_CLASS_OFDM,
                                     true,
                                     10000000, 6000000,
                                     WIFI_CODE_RATE_1_2,
                                     4);
  TxInfo info = TxInfo (CCH, 7, wave_mode, 0, txPower);
  sender->SendX (retransmissionPacket, dest, WSMP_PROT_NUMBER, info);
  EventId retransmitEvent = Simulator::Schedule (MilliSeconds (100), RetransmissionMultihopPacket, sender, retransmissionPacket);
  // if(packetId == 3)
  // {
  //   std::cout <<  " retransmission by " << src->GetId()  << " " << Now().GetMilliSeconds() << std::endl;
  // }
  //std::cout <<  " retransmission by " << src->GetId() << std::endl;
  retrNum++;
  nodeRetransmissionEvent[src][packetId] = retransmitEvent;
}
void 
SendMultihopPacket (Ptr<WaveNetDevice> sender,  Ptr<Packet> packet)
{
  
  
  Ptr<Node> src = sender->GetNode();
  //uint32_t nodeid = src->GetId();
  //Ptr<MobilityModel> model = src->GetObject<MobilityModel> ();
  //Vector relay_pos = model->GetPosition();
  //std::cout << relay_pos.x << ":" << relay_pos.y << " " << nodeid << " relay the packet!" << std::endl;
  PacketTag packetTag;
  SrcNodeTag srcTag;
  RelayNodeTag relayTag;
  packet->FindFirstMatchingByteTag(srcTag);
  packet->FindFirstMatchingByteTag(relayTag);
  packet->FindFirstMatchingByteTag (packetTag);
  Ptr<Packet> ack = Create<Packet> (sizeAck);
  uint32_t hop = packetTag.GetHop();
  if(hop > 0)
  { 
    //std::cout << "add ack tag" << std::endl;
    ack->AddByteTag (packetTag);
    ack->AddByteTag (srcTag);
    ack->AddByteTag (relayTag);
  }
  uint32_t packetId = packetTag.GetPacketId();
  relayNum[packetId]++;
  // if (packetId == 10)
  // {
  //   std::cout << "relay by " << sender->GetNode()->GetId() << " with " << hop  << std::endl;
  // }
  const Address dest = Mac48Address::GetBroadcast ();
  WifiMode wave_mode = WifiModeFactory::CreateWifiMode ("OfdmRate6MbpsBW10MHz",
                                     WIFI_MOD_CLASS_OFDM,
                                     true,
                                     10000000, 6000000,
                                     WIFI_CODE_RATE_1_2,
                                     4);

  txPower = nodePower[src];
  TxInfo info = TxInfo (CCH, 7, wave_mode, 0, txPower);
  
  //std::cout  << " relay by " << src->GetId() <<" " << Now().GetMilliSeconds() << std::endl;
  sender->SendX (packet, dest, WSMP_PROT_NUMBER, info);
  if(hop > 0)
  {
    TxInfo ackInfo = TxInfo (CCH, 7, wave_mode, 0, 30); 
    sender->SendX (ack, dest, WSMP_PROT_NUMBER, ackInfo);
  }
  //std::cout << nodeid << " send packet" << std::endl;
  EventId retransmitEvent = Simulator::Schedule (MilliSeconds (100), RetransmissionMultihopPacket, sender, packet);
  nodeRetransmissionEvent[src][packetId] = retransmitEvent;
}
void
SendWsmpPackets (Ptr<WaveNetDevice> sender, uint32_t channelNumber)
{
  Time now = Now ();
  Ptr<Packet> packet = Create<Packet> (sizeSafety);
  Ptr<Node> src = sender->GetNode();
  Ptr<MobilityModel> model_src = src->GetObject<MobilityModel> ();                     
  Vector src_pos = model_src->GetPosition (); 
  StatsTag tag = StatsTag(src->GetId(), now, src_pos.x, src_pos.y);
  /////////////////////////std::cout << channelNumber << std::endl;
  packet->AddByteTag (tag);
  if((src_pos.x <= 4000) && (src_pos.x >= 1000))
  {
   
     safetyPacketID++;
  }
  const Address dest = Mac48Address::GetBroadcast ();
  WifiMode wave_mode = WifiModeFactory::CreateWifiMode ("OfdmRate6MbpsBW10MHz",
                                     WIFI_MOD_CLASS_OFDM,
                                     true,
                                     10000000, 6000000,
                                     WIFI_CODE_RATE_1_2,
                                     4);

  txPower = nodePower[src];
  TxInfo info = TxInfo (channelNumber, 7, wave_mode, 0, txPower);  
  //std::cout << "nodeID: " << sender->GetNode()->GetId() << " time: "<< now.GetMicroSeconds()  << std::endl;
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
  rngWaitTime = CreateObject<UniformRandomVariable> ();
  rngWaitTime->SetStream (7);
  safetyPacketID = 0;
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
    Ptr<Node> node = sender->GetNode();
    uint32_t nodeId = node->GetId();
    Ptr<MobilityModel> model = node->GetObject<MobilityModel> ();
    //Vector pos = model->GetPosition ();
    //if ((pos.x > 2300)&&(pos.x <= 2500))
    if((nodeId == 60) || (nodeId == 9999))
    {
       for (int i = 1; i <= 30; ++i)
       {
          int k = i*500;
          Simulator::Schedule (MilliSeconds (k), ProduceMultihopPacket, sender);
       }
       // Simulator::Schedule (MilliSeconds (1000), ProduceMultihopPacket, sender);
       // Simulator::Schedule (MilliSeconds (1500), ProduceMultihopPacket, sender);
       // Simulator::Schedule (MilliSeconds (2000), ProduceMultihopPacket, sender);
       // Simulator::Schedule (MilliSeconds (2500), ProduceMultihopPacket, sender);
       // Simulator::Schedule (MilliSeconds (3000), ProduceMultihopPacket, sender);
       // Simulator::Schedule (MilliSeconds (3500), ProduceMultihopPacket, sender);
       // Simulator::Schedule (MilliSeconds (4000), ProduceMultihopPacket, sender);
       // Simulator::Schedule (MilliSeconds (4500), ProduceMultihopPacket, sender);
       // Simulator::Schedule (MilliSeconds (5000), ProduceMultihopPacket, sender);
       // Simulator::Schedule (MilliSeconds (5500), ProduceMultihopPacket, sender);
       // Simulator::Schedule (MilliSeconds (6000), ProduceMultihopPacket, sender);
       // Simulator::Schedule (MilliSeconds (6500), ProduceMultihopPacket, sender);
    }
    // if ((nodeId >= 110) && (nodeId <= 200) && (nodeId%10 == 0)) 
    // {
    //    for (uint32_t time = 0; time != simulationTime-5; ++time)
    //   {   
        
    //       //Simulator::Schedule (Seconds (time), CalculateTxPower, sender, CCH);   
    //     Simulator::Schedule (Seconds (rngSafety->GetValue (time, time + 1)), ProduceMultihopPacket, sender);
        
    //   }
    // }
    
    for (uint32_t time = 0; time != simulationTime; ++time)
    {   
       // Simulator::Schedule (Seconds (time), CalculateTxPower);
      
      for (uint32_t sends = 0; sends != frequencySafety; ++sends)
      {
        //Simulator::Schedule (Seconds (time), CalculateTxPower, sender, CCH);   
        Simulator::Schedule (Seconds (rngSafety->GetValue (time, time + 1)), SendWsmpPackets, sender, CCH);
      }
      
    }
  }
  for (uint32_t time = 1; time <= simulationTime; ++time)
  {
    
      Simulator::Schedule (Seconds (time), CalculateTxPower);
  }

}


void
Stats (void)
{
  //std::map<uint32_t,uint32_t>::iterator i;
  uint32_t lossPacket = 0;
  uint32_t delay = 0;
  uint32_t receive = 0;
  //uint32_t relay = 0;
  NS_LOG_UNCOND("Packet Number: " << relayPacketID );
  // for(uint32_t i = 1; i <= relayPacketID; ++i)
  // {
  //   std::cout << i << " " << receiveNum[i] << " " << relayNum[i] << std::endl;
  // }
  for(uint32_t i = 1; i <= relayPacketID; ++i)
  {
    if(receiveNum[i] == 0)
    {
      lossPacket++;
      continue;
    }
    receive += receiveNum[i];
    delay += totalDelay[i];
    //relayNum += relayNum[i];
    meanDelay[i] = totalDelay[i] / receiveNum[i];
    NS_LOG_UNCOND( "Packet Id:" << i << " Mean Delay:" << meanDelay[i] << "ms Relay Times:" << relayNum[i] 
                    << " Valid Relay Hops: "<< validRelayNum[i]);
  }
  //uint32_t meanDelay = delay / receive;
  double receiveRate = (double)(relayPacketID - lossPacket) / (double)relayPacketID;
  NS_LOG_UNCOND("Success Rate: " << receiveRate << " retransmission time: " << retrNum);

  //NS_LOG_UNCOND("Total Relay num: " << receiveRate);
  //NS_LOG_UNCOND("Mean Delay: " << meanDelay);
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
  NS_LOG_UNCOND ("nodes number: " << nodesNumber  << ", simulation time: " << simulationTime << "s ");
  Configuration ();
  Simulator::Stop (Seconds (simulationTime));
  Simulator::Run ();
  Simulator::Destroy ();
  
  Stats ();
  GetTime();
  //outfile.close();
  return 0;
}

int 
main()
{
  //LogComponentEnable ("power-control", LOG_LEVEL_DEBUG);
  GetTime();
  for (int i = 1; i <= 5; ++i)
  {
    density = 100 * i;
    powerControl = true;
  
  //configPower = 15;
    receiversThres = 100;
    Run();
  }

}
 