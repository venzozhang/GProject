/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 * Copyright (c) 2013 Dalian University of Technology
 *
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 * Author: Junling Bu <linlinjavaer@gmail.com>
 *
 */
/**
 * This example shows basic construction of an 802.11p node.  Two nodes
 * are constructed with 802.11p devices, and by default, one node sends a single
 * packet to another node (the number of packets and interval between
 * them can be configured by command-line arguments).  The example shows
 * typical usage of the helper classes for this mode of WiFi (where "OCB" refers
 * to "Outside the Context of a BSS")."
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
#include "ns3/string.h"
#include "ns3/double.h"
#include "ns3/object-map.h"
#include "ns3/regular-wifi-mac.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/wave-net-device.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wifi-helper.h"

NS_LOG_COMPONENT_DEFINE ("TransmissionRange");

using namespace ns3;
// how to evaluate the transmission range of wifi device (or wave device)
  // my idea is that
  // first change the position between sender and receiver, then transmit 1 packets, and record;
  // second change the random run number and run simulation again until 10000 times.
  // finally stats in which range that the packet is received with the 95% probability .
  // this range here will be treated as Transmission Range

NodeContainer nodes;
NetDeviceContainer devices;
uint32_t TestReceives;
uint32_t TestRange;

uint32_t GetTransmissionRange(void);
void CreateTestNodes(void);
bool TestReceive(Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender);
void SendTestPackets(void);
void SetChannel(void);

uint32_t
GetTransmissionRange()
{
  NS_LOG_UNCOND ("calculating transmission range...");
  bool stop = false;
  TestRange = 0;
  while (!stop)
  {
     TestRange += 10.0;
     TestReceives = 0;

    for (uint32_t r = 0; r != 1000; r++)
    {
      CreateTestNodes();
      RngSeedManager::SetSeed (1+r);
      RngSeedManager::SetRun (17+r);
      Simulator::Schedule (Seconds (1), &SendTestPackets);
      Simulator::Stop (Seconds (2));
      Simulator::Run ();
      Simulator::Destroy ();
      
    }
    std::cout << TestRange << " " << TestReceives << std::endl;
    // if (TestReceives / 1000.0 < 0.95)
    // {
    //     stop = true;
    //     continue;
    // }
    if (TestReceives == 0)
    {
      stop = true;
      continue;
    }
    else
    {
        NS_LOG_DEBUG ( TestRange << " " << TestReceives);
    }
    }
    NS_LOG_UNCOND ( "transmission range: " << TestRange);
    return TestRange;
}

/************************************************************************
* Create two nodes to test the transmission range. Which fuction name 
* includes 'Test' is to get the value of transmission range.
*************************************************************************/
void CreateTestNodes()
{
  nodes = NodeContainer ();
  nodes.Create (2);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (TestRange, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);

  SetChannel();

  for (uint32_t i = 0; i != devices.GetN (); ++i)
    {
      devices.Get (i)->SetReceiveCallback (MakeCallback (&TestReceive));
    }
}

/************************************************************************
* Receive a test packet when get transmission range.
*************************************************************************/
bool 
TestReceive(Ptr<NetDevice> dev, Ptr<const Packet> pkt, uint16_t mode, const Address &sender)
{
  TestReceives++;
  return true;
}

/************************************************************************
* Sending a test packet when get transmission range.
*************************************************************************/
void 
SendTestPackets()
{
  Address dest = Mac48Address::GetBroadcast ();
  Ptr<Packet> packet = Create<Packet> (200);
  WifiMode wave_mode = WifiModeFactory::CreateWifiMode ("OfdmRate6MbpsBW10MHz",
                                     WIFI_MOD_CLASS_OFDM,
                                     true,
                                     10000000, 6000000,
                                     WIFI_CODE_RATE_1_2,
                                     4);  
  //wifi_mode = WifiMode();
  //std::cout << wifi_mode.GetCodeRate() << wifi_mode.  
  //uint8_t txPower = sender->CalculateTxPower();
  uint8_t txPower = 0;
  TxInfo info = TxInfo (CCH, 7, wave_mode, 0, txPower);  
  Ptr<WaveNetDevice> sender = DynamicCast<WaveNetDevice> (devices.Get(0));
  sender->SendX (packet, dest, 0x88DC, info);
}

/************************************************************************
* Configure channel paremeters
*************************************************************************/
void 
SetChannel(void)
{
  YansWifiChannelHelper waveChannel;// = YansWifiChannelHelper::Default ();
  waveChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  //waveChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
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


int main()
{
  uint32_t transmissionRange = GetTransmissionRange();
  std::cout << transmissionRange << std::endl;
}