/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011-2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Author: Manuel Requena <manuel.requena@cttc.es>
 */

#include "ns3/log.h"
#include "ns3/simulator.h"

#include "ns3/lte-pdcp.h"
#include "ns3/lte-pdcp-header.h"
#include "ns3/lte-pdcp-sap.h"
#include "ns3/lte-pdcp-tag.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LtePdcp");

class LtePdcpSpecificLteRlcSapUser : public LteRlcSapUser
{
public:
  LtePdcpSpecificLteRlcSapUser (LtePdcp* pdcp);

  // Interface provided to lower RLC entity (implemented from LteRlcSapUser)
  virtual void ReceivePdcpPdu (Ptr<Packet> p);  //+++++++++++++++++before was (Ptr<Packet> p)*

private:
  LtePdcpSpecificLteRlcSapUser ();
  LtePdcp* m_pdcp;
};

LtePdcpSpecificLteRlcSapUser::LtePdcpSpecificLteRlcSapUser (LtePdcp* pdcp)
  : m_pdcp (pdcp)
{
}

LtePdcpSpecificLteRlcSapUser::LtePdcpSpecificLteRlcSapUser ()
{
}

void
LtePdcpSpecificLteRlcSapUser::ReceivePdcpPdu (Ptr<Packet> p)  //+++++++++++++++before was (Ptr<Packet> p)*
{
  m_pdcp->DoReceivePdu (p);  //++++++++++++++++++before was (p)*
  // NS_LOG_UNCOND("LtePdcpSpecificLteRlcSapUser::ReceivePdcpPdu ");
}

///////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED (LtePdcp);

LtePdcp::LtePdcp ()
  : m_pdcpSapUser (0),
    m_rlcSapProvider (0),
    m_rnti (0),
    m_lcid (0),
    m_targetRnti (0),  //+++++++++++*
    m_D2DFlag (0),  //+++++++++++*
    m_txSequenceNumber (0),
    m_rxSequenceNumber (0)
{
  NS_LOG_FUNCTION (this);
  m_pdcpSapProvider = new LtePdcpSpecificLtePdcpSapProvider<LtePdcp> (this);
  m_rlcSapUser = new LtePdcpSpecificLteRlcSapUser (this);
}

LtePdcp::~LtePdcp ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
LtePdcp::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LtePdcp")
    .SetParent<Object> ()
    .AddTraceSource ("TxPDU",
                     "PDU transmission notified to the RLC.",
                     MakeTraceSourceAccessor (&LtePdcp::m_txPdu),
                     "ns3::LtePdcp::PduTxTracedCallback")
    .AddTraceSource ("RxPDU",
                     "PDU received.",
                     MakeTraceSourceAccessor (&LtePdcp::m_rxPdu),
                     "ns3::LtePdcp::PduRxTracedCallback")
    ;
  return tid;
}

void
LtePdcp::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  delete (m_pdcpSapProvider);
  delete (m_rlcSapUser);
}


void
LtePdcp::SetRnti (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << (uint32_t) rnti);
  m_rnti = rnti;
}

void
LtePdcp::SetLcId (uint8_t lcId)
{
  NS_LOG_FUNCTION (this << (uint32_t) lcId);
  m_lcid = lcId;
}

void
LtePdcp::SetLtePdcpSapUser (LtePdcpSapUser * s)
{
  NS_LOG_FUNCTION (this << s);
  m_pdcpSapUser = s;
}

LtePdcpSapProvider*
LtePdcp::GetLtePdcpSapProvider ()
{
  NS_LOG_FUNCTION (this);
  return m_pdcpSapProvider;
}

void
LtePdcp::SetLteRlcSapProvider (LteRlcSapProvider * s)
{
  NS_LOG_FUNCTION (this << s);
  m_rlcSapProvider = s;
}

LteRlcSapUser*
LtePdcp::GetLteRlcSapUser ()
{
  NS_LOG_FUNCTION (this);
  return m_rlcSapUser;
}

LtePdcp::Status 
LtePdcp::GetStatus ()
{
  Status s;
  s.txSn = m_txSequenceNumber;
  s.rxSn = m_rxSequenceNumber;
  return s;
}

void 
LtePdcp::SetStatus (Status s)
{
  m_txSequenceNumber = s.txSn;
  m_rxSequenceNumber = s.rxSn;
}

////////////////////////////////////////
void
LtePdcp::SetTargetRnti(uint16_t targetRnti)
  {
    m_targetRnti = targetRnti;
  }

void
LtePdcp::SetD2DMode(bool flag)
{
  m_D2DFlag = flag;
}

void
LtePdcp::DoTransmitPdcpSdu (Ptr<Packet> p)  //++++++++++*before was (Ptr<Packet> p)*
{
  // Ptr<Packet> p = paramstx.pdcpSdu;  //+++++++++++*
  // m_rnti = paramstx.rnti;  //++++++++++++*
  // m_lcid = paramstx.lcid;  //+++++++++++*
  // m_targetRnti = paramstx.targetRnti;  //++++++++++*
  // m_D2DFlag = paramstx.D2DFlag;  //++++++++++++*
  NS_LOG_FUNCTION (this << m_rnti << (uint32_t) m_lcid << p->GetSize ());
  //std::cout << "111" << std::endl;
  LtePdcpHeader pdcpHeader;
  pdcpHeader.SetSequenceNumber (m_txSequenceNumber);
  m_txSequenceNumber++;
  if (m_txSequenceNumber > m_maxPdcpSn)
    {
      m_txSequenceNumber = 0;
    }

  pdcpHeader.SetDcBit (LtePdcpHeader::DATA_PDU);
  //std::cout << "LtePdcp::DoTransmitPdcpSdu flag:" << m_D2DFlag <<std::endl;
  NS_LOG_LOGIC ("PDCP header: " << pdcpHeader);
  p->AddHeader (pdcpHeader);
  //std::cout << "LtePdcp::DoTransmitPdcpSdu: add header successful" << std::endl;
  // Sender timestamp
  PdcpTag pdcpTag (Simulator::Now ());
  //std::cout << "bbb" << std::endl;
  p->AddByteTag (pdcpTag);
  //std::cout << "ccc" << std::endl;
  m_txPdu (m_rnti, m_lcid, p->GetSize ());
  //std::cout << "bbb" << std::endl;

  LteRlcSapProvider::TransmitPdcpPduParameters params;
  params.rnti = m_rnti;
  params.lcid = m_lcid;
  params.pdcpPdu = p;
  params.targetRnti = m_targetRnti; //++++++++++*
  params.D2DFlag = m_D2DFlag;  //++++++++++*

  //NS_LOG_UNCOND("LtePdcp::DoTransmitPdcpSdu:   "  << m_D2DFlag);
  if(m_D2DFlag)
  {
     NS_LOG_UNCOND("LtePdcp::Send a  D2D Packet  -----  src rnti is: " << m_rnti << "  dest rnti is: " << m_targetRnti);
  }
  m_rlcSapProvider->TransmitPdcpPdu (params);
}

void
LtePdcp::DoReceivePdu (Ptr<Packet> p)  //++++++++++*before was (Ptr<Packet> p)*
{
  // Ptr<Packet> p = paramsrx.pdcpSdu;  //+++++++++++*
  // m_rnti = paramsrx.rnti;  //++++++++++++*
  // m_lcid = paramsrx.lcid;  //+++++++++++*
  // m_targetRnti = paramsrx.targetRnti;  //++++++++++*
  // m_flag = paramsrx.flag;  //++++++++++++*  
  NS_LOG_FUNCTION (this << m_rnti << (uint32_t) m_lcid << p->GetSize ());
  //std::cout << "LtePdcp::DoReceivePdu" << std::endl;
  // Receiver timestamp
  PdcpTag pdcpTag;
  Time delay;
  if (p->FindFirstMatchingByteTag (pdcpTag))
    {
      delay = Simulator::Now() - pdcpTag.GetSenderTimestamp ();
    }

  m_rxPdu(m_rnti, m_lcid, p->GetSize (), delay.GetNanoSeconds ());


  LtePdcpHeader pdcpHeader;
  p->RemoveHeader (pdcpHeader);
  NS_LOG_LOGIC ("PDCP header: " << pdcpHeader);

  m_rxSequenceNumber = pdcpHeader.GetSequenceNumber () + 1;
  if (m_rxSequenceNumber > m_maxPdcpSn)
    {
      m_rxSequenceNumber = 0;
    }

  LtePdcpSapUser::ReceivePdcpSduParameters params;
  params.pdcpSdu = p;
  params.rnti = m_rnti;
  params.lcid = m_lcid;
  params.targetRnti = m_targetRnti; //++++++++++*
  params.D2DFlag = m_D2DFlag;  //++++++++++*
  //std::cout << "LtePdcp::DoReceivePdu complete" << std::endl;
  m_pdcpSapUser->ReceivePdcpSdu (params);
}


} // namespace ns3
