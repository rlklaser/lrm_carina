/*
 *  Copyright (C) 2012, Laboratorio de Robotica Movel - ICMC/USP
 *  Rafael Luiz Klaser <rlklaser@gmail.com>
 *  http://lrm.icmc.usp.br
 *
 *  Apoio FAPESP: 2012/04555-4
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file wifi_sim.cpp
 * @brief 
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 23, 2012
 *
 */
#if 0
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

using namespace ns3;

static void
CourseChange (std::string context, Ptr<const MobilityModel> position)
{
  Vector pos = position->GetPosition ();
  std::cout << Simulator::Now () << ", pos=" << position << ", x=" << pos.x << ", y=" << pos.y
            << ", z=" << pos.z << std::endl;
}

int main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);

  printf("init\n");

  NodeContainer c;
  c.Create (10000);

  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                                 "X", StringValue ("100.0"),
                                 "Y", StringValue ("100.0"),
                                 "Rho", StringValue ("ns3::UniformRandomVariable[Min=0|Max=30]"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c);

  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                   MakeCallback (&CourseChange));

  printf("bb\n");
  Simulator::Stop (Seconds (1000.0));

  printf("run\n");
  Simulator::Run ();

  Simulator::Destroy ();
  return 0;
}
#endif

#if 0
#include <iostream>

#include "ns3/core-module.h"
//#include "ns3/helper-module.h"
#include "ns3/node.h"
#include "ns3/simulator.h"

using namespace ns3;

static void
GenerateTraffic (Ptr<Socket> socket, uint32_t size)
{
  std::cout << "at=" << Simulator::Now ().GetSeconds () << "s, tx bytes=" << size << std::endl;
  socket->Send (Create<Packet> (size));
  if (size > 0)
    {
      Simulator::Schedule (Seconds (0.5), &GenerateTraffic, socket, size - 50);
    }
  else
    {
      socket->Close ();
    }
}

static void
SocketPrinter (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  while (packet = socket->Recv ())
    {
      std::cout << "at=" << Simulator::Now ().GetSeconds () << "s, rx bytes=" << packet->GetSize () << std::endl;
    }
}

static void
PrintTraffic (Ptr<Socket> socket)
{
  socket->SetRecvCallback (MakeCallback (&SocketPrinter));
}

void
RunSimulation (void)
{
  NodeContainer c;
  c.Create (1);

  InternetStackHelper internet;
  internet.Install (c);


  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (c.Get (0), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  sink->Bind (local);

  Ptr<Socket> source = Socket::CreateSocket (c.Get (0), tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address::GetLoopback (), 80);
  source->Connect (remote);

  GenerateTraffic (source, 500);
  PrintTraffic (sink);


  Simulator::Run ();

  Simulator::Destroy ();
}

int main (int argc, char *argv[])
{
  RunSimulation ();

  return 0;
}
#endif

#if false
/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 The Boeing Company
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
 */

//
// This script configures two nodes on an 802.11b physical layer, with
// 802.11b NICs in infrastructure mode, and by default, the station sends
// one packet of 1000 (application) bytes to the access point.  The
// physical layer is configured
// to receive at a fixed RSS (regardless of the distance and transmit
// power); therefore, changing position of the nodes has no effect.
//
// There are a number of command-line options available to control
// the default behavior.  The list of available command-line options
// can be listed with the following command:
// ./waf --run "wifi-simple-infra --help"
//
// For instance, for this configuration, the physical layer will
// stop successfully receiving packets when rss drops below -97 dBm.
// To see this effect, try running:
//
// ./waf --run "wifi-simple-infra --rss=-97 --numPackets=20"
// ./waf --run "wifi-simple-infra --rss=-98 --numPackets=20"
// ./waf --run "wifi-simple-infra --rss=-99 --numPackets=20"
//
// Note that all ns-3 attributes (not just the ones exposed in the below
// script) can be changed at command line; see the documentation.
//
// This script can also be helpful to put the Wifi layer into verbose
// logging mode; this command will turn on all wifi logging:
//
// ./waf --run "wifi-simple-infra --verbose=1"
//
// When you are done, you will notice two pcap trace files in your directory.
// If you have tcpdump installed, you can try this:
//
// tcpdump -r wifi-simple-infra-0-0.pcap -nn -tt
//

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

NS_LOG_COMPONENT_DEFINE ("WifiSimpleInfra");

using namespace ns3;

void ReceivePacket (Ptr<Socket> socket)
{
  NS_LOG_UNCOND ("Received one packet!");
  printf("ha\n");
}

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount-1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}


int main (int argc, char *argv[])
{
  std::string phyMode ("DsssRate1Mbps");
  double rss = -80;  // -dBm
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 10;
  double interval = 1.0; // seconds
  bool verbose = true;

  CommandLine cmd;

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("rss", "received signal strength", rss);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);

  cmd.Parse (argc, argv);
  // Convert to time object
  Time interPacketInterval = Seconds (interval);

  // disable fragmentation for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  // turn off RTS/CTS for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
                      StringValue (phyMode));

  NodeContainer c;
  c.Create (2);

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents ();  // Turn on all Wifi logging
    }
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  // This is one parameter that matters when using FixedRssLossModel
  // set it to zero; otherwise, gain will be added
  wifiPhy.Set ("RxGain", DoubleValue (0) );
  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  // The below FixedRssLossModel will cause the rss to be fixed regardless
  // of the distance between the two stations, and the transmit power
  wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",DoubleValue (rss));
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add a non-QoS upper mac, and disable rate control
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));

  // Setup the rest of the upper mac
  Ssid ssid = Ssid ("wifi-default");
  // setup sta.
  wifiMac.SetType ("ns3::StaWifiMac",
                   "Ssid", SsidValue (ssid),
                   "ActiveProbing", BooleanValue (false));
  NetDeviceContainer staDevice = wifi.Install (wifiPhy, wifiMac, c.Get (0));
  NetDeviceContainer devices = staDevice;
  // setup ap.
  wifiMac.SetType ("ns3::ApWifiMac",
                   "Ssid", SsidValue (ssid));
  NetDeviceContainer apDevice = wifi.Install (wifiPhy, wifiMac, c.Get (1));
  devices.Add (apDevice);

  // Note that with FixedRssLossModel, the positions below are not
  // used for received signal strength.
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (5.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c);

  InternetStackHelper internet;
  internet.Install (c);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (0), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  Ptr<Socket> source = Socket::CreateSocket (c.Get (1), tid);
  InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
  source->SetAllowBroadcast (true);
  source->Connect (remote);

  // Tracing
  wifiPhy.EnablePcap ("wifi-simple-infra", devices);

  // Output what we are doing
  NS_LOG_UNCOND ("Testing " << numPackets  << " packets sent with receiver rss " << rss );

  Simulator::ScheduleWithContext (source->GetNode ()->GetId (),
                                  Seconds (1.0), &GenerateTraffic,
                                  source, packetSize, numPackets, interPacketInterval);

  Simulator::Stop (Seconds (30.0));
  Simulator::Run ();
  printf("boing\n");
  Simulator::Destroy ();

  return 0;
}
#endif

#if false
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
 */
/*
#include "ns3/core-module.h"
#include "ns3/simulator-module.h"
#include "ns3/node-module.h"
#include "ns3/helper-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
*/

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/csma-module.h"
#include "ns3/udp-echo-client.h"
#include "ns3/udp-echo-server.h"
// Default Network Topology
//
//   Wifi 10.1.3.0
//                 AP
//  *    *    *    *
//  |    |    |    |    10.1.1.0
// n5   n6   n7   n0 -------------- n1   n2   n3   n4
//                   point-to-point  |    |    |    |
//                                   ================
//                                     LAN 10.1.2.0

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("ThirdScriptExample");

int
main (int argc, char *argv[])
{
  bool verbose = true;
  uint32_t nCsma = 3;
  uint32_t nWifi = 3;

  CommandLine cmd;
  cmd.AddValue ("nCsma", "Number of \"extra\" CSMA nodes/devices", nCsma);
  cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);
  cmd.AddValue ("verbose", "Tell echo applications to log if true", verbose);

  cmd.Parse (argc,argv);

  if (verbose)
    {
      LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
      LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }

  NodeContainer p2pNodes;
  p2pNodes.Create (2);

  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  pointToPoint.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NetDeviceContainer p2pDevices;
  p2pDevices = pointToPoint.Install (p2pNodes);

  NodeContainer csmaNodes;
  csmaNodes.Add (p2pNodes.Get (1));
  csmaNodes.Create (nCsma);

  CsmaHelper csma;
  csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
  csma.SetChannelAttribute ("Delay", TimeValue (NanoSeconds (6560)));

  NetDeviceContainer csmaDevices;
  csmaDevices = csma.Install (csmaNodes);

  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (nWifi);
  NodeContainer wifiApNode = p2pNodes.Get (0);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());

  WifiHelper wifi = WifiHelper::Default ();
  wifi.SetRemoteStationManager ("ns3::AarfWifiManager");

  NqosWifiMacHelper mac = NqosWifiMacHelper::Default ();

  Ssid ssid = Ssid ("ns-3-ssid");
  mac.SetType ("ns3::NqstaWifiMac",
    "Ssid", SsidValue (ssid),
    "ActiveProbing", BooleanValue (false));

  NetDeviceContainer staDevices;
  staDevices = wifi.Install (phy, mac, wifiStaNodes);

  mac.SetType ("ns3::NqapWifiMac",
    "Ssid", SsidValue (ssid),
    "BeaconGeneration", BooleanValue (true),
    "BeaconInterval", TimeValue (Seconds (2.5)));

  NetDeviceContainer apDevices;
  apDevices = wifi.Install (phy, mac, wifiApNode);

  MobilityHelper mobility;

  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
    "MinX", DoubleValue (0.0),
    "MinY", DoubleValue (0.0),
    "DeltaX", DoubleValue (5.0),
    "DeltaY", DoubleValue (10.0),
    "GridWidth", UintegerValue (3),
    "LayoutType", StringValue ("RowFirst"));

  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
    "Bounds", RectangleValue (Rectangle (-50, 50, -50, 50)));
  mobility.Install (wifiStaNodes);

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNode);

  InternetStackHelper stack;
  stack.Install (csmaNodes);
  stack.Install (wifiApNode);
  stack.Install (wifiStaNodes);

  Ipv4AddressHelper address;

  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer p2pInterfaces;
  p2pInterfaces = address.Assign (p2pDevices);

  address.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer csmaInterfaces;
  csmaInterfaces = address.Assign (csmaDevices);

  address.SetBase ("10.1.3.0", "255.255.255.0");
  address.Assign (staDevices);
  address.Assign (apDevices);

  UdpEchoServerHelper echoServer (9);

  ApplicationContainer serverApps = echoServer.Install (csmaNodes.Get (nCsma));
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (10.0));

  UdpEchoClientHelper echoClient (csmaInterfaces.GetAddress (nCsma), 9);
  echoClient.SetAttribute ("MaxPackets", UintegerValue (1));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (1.)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (1024));

  ApplicationContainer clientApps =
    echoClient.Install (wifiStaNodes.Get (nWifi - 1));
  clientApps.Start (Seconds (2.0));
  clientApps.Stop (Seconds (10.0));

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  Simulator::Stop (Seconds (10.0));

  pointToPoint.EnablePcapAll ("third");
  phy.EnablePcap ("third", apDevices.Get (0));
  csma.EnablePcap ("third", csmaDevices.Get (0), true);

  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
#endif

#if 0
#include <ns3/core-module.h>
//#include <ns3/helper-module.h>
#include <ns3/mobility-module.h>
//#include "ns3/simulator-module.h"

using namespace ns3;

static void
CourseChange (std::string foo, Ptr<const MobilityModel> mobility)
{
  Vector pos = mobility->GetPosition ();
  Vector vel = mobility->GetVelocity ();
  std::cout << Simulator::Now () << ", model=" << mobility << ", POS: x=" << pos.x << ", y=" << pos.y
            << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
            << ", z=" << vel.z << std::endl;
}

int main (int argc, char *argv[])
{
	bool verbose = true;

  Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Mode", StringValue ("Time"));
  Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Time", StringValue ("2s"));
  Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Speed", StringValue ("Constant:1.0"));
  Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Bounds", StringValue ("0|200|0|200"));

  CommandLine cmd;

  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);

  cmd.Parse (argc, argv);

  NodeContainer c;
  c.Create (100);

  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                                 "X", StringValue ("100.0"),
                                 "Y", StringValue ("100.0"),
                                 "Rho", StringValue ("Uniform:0:30"));
  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                             "Mode", StringValue ("Time"),
                             "Time", StringValue ("2s"),
                             "Speed", StringValue ("Constant:1.0"),
                             "Bounds", StringValue ("0|200|0|200"));
  mobility.InstallAll ();
  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                   MakeCallback (&CourseChange));

  Simulator::Stop (Seconds (100.0));

  Simulator::Run ();

  return 0;
}
#endif

#if 0
/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006,2007 INRIA
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
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

using namespace ns3;

static void
CourseChange (std::string foo, Ptr<const MobilityModel> mobility)
{
  Vector pos = mobility->GetPosition ();
  Vector vel = mobility->GetVelocity ();
  std::cout << Simulator::Now () << ", model=" << mobility << ", POS: x=" << pos.x << ", y=" << pos.y
            << ", z=" << pos.z << "; VEL:" << vel.x << ", y=" << vel.y
            << ", z=" << vel.z << std::endl;
}

int main (int argc, char *argv[])
{
  Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Mode", StringValue ("Time"));
  Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Time", StringValue ("2s"));
  Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  Config::SetDefault ("ns3::RandomWalk2dMobilityModel::Bounds", StringValue ("0|200|0|200"));

  CommandLine cmd;
  cmd.Parse (argc, argv);

  NodeContainer c;
  c.Create (100);

  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                                 "X", StringValue ("100.0"),
                                 "Y", StringValue ("100.0"),
                                 "Rho", StringValue ("ns3::UniformRandomVariable[Min=0|Max=30]"));
  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                             "Mode", StringValue ("Time"),
                             "Time", StringValue ("2s"),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"),
                             "Bounds", StringValue ("0|200|0|200"));
  mobility.InstallAll ();
  Config::Connect ("/NodeList/*/$ns3::MobilityModel/CourseChange",
                   MakeCallback (&CourseChange));

  Simulator::Stop (Seconds (100.0));

  Simulator::Run ();

  Simulator::Destroy ();
  return 0;
}

#endif

#if okk
/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 CTTC
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
 * Author: Nicola Baldo <nbaldo@cttc.es>
 */



#include <iostream>

#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/spectrum-model-ism2400MHz-res1MHz.h>
#include <ns3/spectrum-model-300kHz-300GHz-log.h>
#include <ns3/wifi-spectrum-value-helper.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/waveform-generator.h>
#include <ns3/spectrum-analyzer.h>
#include <ns3/log.h>
#include <string>
#include <iomanip>
#include <ns3/friis-spectrum-propagation-loss.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/mobility-module.h>
#include <ns3/spectrum-helper.h>
#include <ns3/applications-module.h>
#include <ns3/adhoc-aloha-noack-ideal-phy-helper.h>


NS_LOG_COMPONENT_DEFINE ("TestAdhocOfdmAloha");

using namespace ns3;

static bool g_verbose = true;
static uint64_t g_rxBytes;

void
PhyRxEndOkTrace (std::string context, Ptr<const Packet> p)
{
  if (g_verbose)
    {
      std::cout << context << " PHY RX END OK p:" << p << std::endl;
    }
  g_rxBytes += p->GetSize ();
}



/**
 * Store the last pathloss value for each TX-RX pair. This is an
 * example of how the PathlossTrace (provided by some SpectrumChannel
 * implementations) work.
 *
 */
class GlobalPathlossDatabase
{
public:

  /**
   * update the pathloss value
   *
   * \param context
   * \param txPhy the transmitting PHY
   * \param rxPhy the receiving PHY
   * \param lossDb the loss in dB
   */
  void UpdatePathloss (std::string context, Ptr<SpectrumPhy> txPhy, Ptr<SpectrumPhy> rxPhy, double lossDb);

  /**
   * print the stored pathloss values to standard output
   *
   */
  void Print ();

private:
  std::map<uint32_t, std::map<uint32_t, double> > m_pathlossMap;
};

void
GlobalPathlossDatabase::UpdatePathloss (std::string context,
                                        Ptr<SpectrumPhy> txPhy,
                                        Ptr<SpectrumPhy> rxPhy,
                                        double lossDb)
{
  uint32_t txNodeId = txPhy->GetMobility ()->GetObject<Node> ()->GetId ();
  uint32_t rxNodeId = rxPhy->GetMobility ()->GetObject<Node> ()->GetId ();
  m_pathlossMap[txNodeId][rxNodeId] = lossDb;
}

void
GlobalPathlossDatabase::Print ()
{
  for (std::map<uint32_t, std::map<uint32_t, double> >::const_iterator txit = m_pathlossMap.begin ();
       txit != m_pathlossMap.end ();
       ++txit)
    {
      for (std::map<uint32_t, double>::const_iterator rxit = txit->second.begin ();
           rxit != txit->second.end ();
           ++rxit)
        {
          std::cout << txit->first << " --> " << rxit->first << " : " << rxit->second << " dB" << std::endl;
        }
    }
}



int main (int argc, char** argv)
{
  CommandLine cmd;
  double lossDb = 150;
  double txPowerW = 0.1;
  uint64_t phyRate = 500000;
  uint32_t pktSize = 1000;
  double simDuration = 30;
  std::string channelType ("ns3::SingleModelSpectrumChannel");
  cmd.AddValue ("verbose", "Print trace information if true", g_verbose);
  cmd.AddValue ("lossDb", "link loss in dB", lossDb);
  cmd.AddValue ("txPowerW", "txPower in Watts", txPowerW);
  cmd.AddValue ("phyRate", "PHY rate in bps", phyRate);
  cmd.AddValue ("pktSize", "packet size in bytes", pktSize);
  cmd.AddValue ("simDuration", "duration of the simulation in seconds", simDuration);
  cmd.AddValue ("channelType", "which SpectrumChannel implementation to be used", channelType);
  cmd.Parse (argc, argv);

  NodeContainer c;
  c.Create (2);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (5.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");


  mobility.Install (c);


  SpectrumChannelHelper channelHelper;
  channelHelper.SetChannel (channelType);
  channelHelper.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  Ptr<MatrixPropagationLossModel> propLoss = CreateObject<MatrixPropagationLossModel> ();
  propLoss->SetLoss (c.Get (0)->GetObject<MobilityModel> (), c.Get (1)->GetObject<MobilityModel> (), lossDb, true);
  channelHelper.AddPropagationLoss (propLoss);
  Ptr<SpectrumChannel> channel = channelHelper.Create ();


  WifiSpectrumValue5MhzFactory sf;

  uint32_t channelNumber = 1;
  Ptr<SpectrumValue> txPsd =  sf.CreateTxPowerSpectralDensity (txPowerW, channelNumber);

  // for the noise, we use the Power Spectral Density of thermal noise
  // at room temperature. The value of the PSD will be constant over the band of interest.
  const double k = 1.381e-23; //Boltzmann's constant
  const double T = 290; // temperature in Kelvin
  double noisePsdValue = k * T; // watts per hertz
  Ptr<SpectrumValue> noisePsd = sf.CreateConstant (noisePsdValue);

  AdhocAlohaNoackIdealPhyHelper deviceHelper;
  deviceHelper.SetChannel (channel);
  deviceHelper.SetTxPowerSpectralDensity (txPsd);
  deviceHelper.SetNoisePowerSpectralDensity (noisePsd);
  deviceHelper.SetPhyAttribute ("Rate", DataRateValue (DataRate (phyRate)));
  NetDeviceContainer devices = deviceHelper.Install (c);

  PacketSocketHelper packetSocket;
  packetSocket.Install (c);

  PacketSocketAddress socket;
  socket.SetSingleDevice (devices.Get (0)->GetIfIndex ());
  socket.SetPhysicalAddress (devices.Get (1)->GetAddress ());
  socket.SetProtocol (1);

  OnOffHelper onoff ("ns3::PacketSocketFactory", Address (socket));
  onoff.SetConstantRate (DataRate (2*phyRate));
  onoff.SetAttribute ("PacketSize", UintegerValue (pktSize));

  ApplicationContainer apps = onoff.Install (c.Get (0));
  apps.Start (Seconds (0.0));
  apps.Stop (Seconds (simDuration));

  Config::Connect ("/NodeList/*/DeviceList/*/Phy/RxEndOk", MakeCallback (&PhyRxEndOkTrace));

  GlobalPathlossDatabase globalPathlossDatabase;
  Config::Connect ("/ChannelList/*/PropagationLoss",
                   MakeCallback (&GlobalPathlossDatabase::UpdatePathloss, &globalPathlossDatabase));

  g_rxBytes = 0;
  //Simulator::Stop (Seconds (simDuration + 0.000001));
  Simulator::Run ();

  if (g_verbose)
    {
      globalPathlossDatabase.Print ();

      double throughputBps = (g_rxBytes * 8.0) / simDuration;
      std::cout << "throughput:       " << throughputBps << std::endl;
      std::cout << "throughput:       " << std::setw (20) << std::fixed << throughputBps << " bps" << std::endl;
      std::cout << "phy rate  :       "   << std::setw (20) << std::fixed << phyRate*1.0 << " bps" << std::endl;
      double rxPowerW = txPowerW / (std::pow (10.0, lossDb/10.0));
      double capacity = 20e6*log2 (1.0 + (rxPowerW/20.0e6)/noisePsdValue);
      std::cout << "shannon capacity: "   << std::setw (20) << std::fixed << capacity <<  " bps" << std::endl;

    }



  Simulator::Destroy ();



  //GtkConfigStore config;
 // config.ConfigureDefaults ();
  //config.ConfigureAttributes ();

  return 0;
}
#endif

#if zzz
/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation;
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/
// Default Network Topology
//
// Wifi 10.1.1.0
// AP
// * *
// | |
// n1 n0
//
// Node #0 is the AP, #1 is a base station
// #1 sends UDP echo mesg to the AP; AP sends a UDP response back to the node
// Communication is possible only when the station is within a certain distance from the AP
#include "ns3/core-module.h"
//#include "ns3/simulator-module.h"
//#include "ns3/node-module.h"
//#include "ns3/helper-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ipv4-global-routing-helper.h"
using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("Wifi-2-nodes-fixed");
void PrintLocations(NodeContainer nodes, std::string header)
{
	std::cout << header << std::endl;
	for (NodeContainer::Iterator iNode = nodes.Begin(); iNode != nodes.End(); ++iNode)
	{
		Ptr<Node> object = *iNode;
		Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
//NS_ASSERT (position != 0);
		Vector pos = position->GetPosition();
		std::cout << "(" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
	}
	std::cout << std::endl;
}
void PrintAddresses(Ipv4InterfaceContainer container, std::string header)
{
	std::cout << header << std::endl;
	uint32_t nNodes = container.GetN();
	for (uint32_t i = 0; i < nNodes; ++i)
	{
		std::cout << container.GetAddress(i, 0) << std::endl;
	}
	std::cout << std::endl;
}

int main(int argc, char *argv[])
{
	bool verbose = true;
	uint32_t nWifi = 2;
	/** Change this parameter and verify the output */
	double xDistance = 10.0;

	CommandLine cmd;
	cmd.AddValue("xDistance", "Distance between two nodes along x-axis", xDistance);

	cmd.Parse(argc, argv);
	if (verbose)
	{
		LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
		LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
	}
// 1. Create the nodes and hold them in a container
	NodeContainer wifiStaNodes, wifiApNode;

	wifiStaNodes.Create(nWifi);
	wifiApNode = wifiStaNodes.Get(0);
// 2. Create channel for communication
	YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
	YansWifiPhyHelper phy = YansWifiPhyHelper::Default();
	phy.SetChannel(channel.Create());
	WifiHelper wifi = WifiHelper::Default();
	wifi.SetRemoteStationManager("ns3::AarfWifiManager");

	NqosWifiMacHelper mac = NqosWifiMacHelper::Default();
// 3a. Set up MAC for base stations
	Ssid ssid = Ssid("ns-3-ssid");
	mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
	NetDeviceContainer staDevices;
	staDevices = wifi.Install(phy, mac, wifiStaNodes.Get(1));
// 3b. Set up MAC for AP
	mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid), "BeaconGeneration", BooleanValue(true), "BeaconInterval", TimeValue(Seconds(5)));
	NetDeviceContainer apDevice;
	apDevice = wifi.Install(phy, mac, wifiApNode);
// 4. Set mobility of the nodes
	MobilityHelper mobility;
// All space coordinates in meter
	mobility.SetPositionAllocator("ns3::GridPositionAllocator", "MinX", DoubleValue(0.0), "MinY", DoubleValue(0.0), "DeltaX", DoubleValue(xDistance), "DeltaY", DoubleValue(10.0), "GridWidth", UintegerValue(3), "LayoutType", StringValue("RowFirst"));

	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(wifiStaNodes);
// 5.Add Internet layers stack
	InternetStackHelper stack;
	stack.Install(wifiStaNodes);
// 6. Assign IP address to each device
	Ipv4AddressHelper address;
	Ipv4InterfaceContainer wifiInterfaces, wifiApInterface;
	address.SetBase("10.1.1.0", "255.255.255.0");
	wifiApInterface = address.Assign(apDevice);
	wifiInterfaces = address.Assign(staDevices);
// 7a. Create and setup applications (traffic sink)
	UdpEchoServerHelper echoServer(9); // Port # 9
	ApplicationContainer serverApps = echoServer.Install(wifiApNode);
	serverApps.Start(Seconds(1.0));
	serverApps.Stop(Seconds(4.0));
// 7b. Create and setup applications (traffic source)
	UdpEchoClientHelper echoClient(wifiApInterface.GetAddress(0), 9);
	echoClient.SetAttribute("MaxPackets", UintegerValue(1));
	echoClient.SetAttribute("Interval", TimeValue(Seconds(1.)));
	echoClient.SetAttribute("PacketSize", UintegerValue(1024));
	ApplicationContainer clientApps = echoClient.Install(wifiStaNodes.Get(1));
	clientApps.Start(Seconds(2.0));
	clientApps.Stop(Seconds(3.0));
	Ipv4GlobalRoutingHelper::PopulateRoutingTables();
	Simulator::Stop(Seconds(4.0));
// 8. Enable tracing (optional)
//phy.EnablePcapAll ("wifi-2-nodes-fixed", true);

	PrintAddresses(wifiInterfaces, "IP addresses of base stations");
	PrintAddresses(wifiApInterface, "IP address of AP");
	PrintLocations(wifiStaNodes, "Location of all nodes");

	Simulator::Run();
	Simulator::Destroy();

	return 0;
}
#endif

#if o
#include "ns3/core-module.h"
//#include "ns3/simulator-module.h"
//#include "ns3/node-module.h"
//#include "ns3/helper-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"

using namespace ns3;

int main(int argc, char *argv[]) {
	NodeContainer wifiNodes;
	wifiNodes.Create(3);

	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
	wifiPhy.SetChannel(wifiChannel.Create());
	WifiHelper wifi = WifiHelper::Default();
	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default();
	wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager");
	wifiMac.SetType("ns3::AdhocWifiMac");
	NetDeviceContainer wifiDevices = wifi.Install(wifiPhy, wifiMac, wifiNodes);

	MobilityHelper mobility;
	mobility.SetPositionAllocator("ns3::GridPositionAllocator", "MinX", DoubleValue(0.0), "MinY", DoubleValue(0.0), "DeltaX", DoubleValue(5.0), "DeltaY", DoubleValue(10.0), "GridWidth", UintegerValue(3), "LayoutType", StringValue("RowFirst"));
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(wifiNodes);

	InternetStackHelper internet;
	internet.Install(wifiNodes);

	Ipv4AddressHelper address;
	address.SetBase("10.1.1.0", "255.255.255.0");
	Ipv4InterfaceContainer wifiInterfaces;
	wifiInterfaces = address.Assign(wifiDevices);

	UdpEchoServerHelper echoServer(9);
	ApplicationContainer serverApps = echoServer.Install(wifiNodes.Get(0));
	serverApps.Start(Seconds(1.0));
	serverApps.Stop(Seconds(10.0));

	UdpEchoClientHelper echoClient(wifiInterfaces.GetAddress(0), 9);
	echoClient.SetAttribute("MaxPackets", UintegerValue(2));
	echoClient.SetAttribute("Interval", TimeValue(Seconds(1.)));
	echoClient.SetAttribute("PacketSize", UintegerValue(1024));
	ApplicationContainer clientApps = echoClient.Install(wifiNodes.Get(2));
	clientApps.Start(Seconds(2.0));
	clientApps.Stop(Seconds(10.0));

	Ipv4GlobalRoutingHelper::PopulateRoutingTables();

	Simulator::Stop(Seconds(5.05));
	Simulator::Run();
	Simulator::Destroy();
	return 0;
}
#endif

#if TESTES_LOG_OK
/*
using namespace ns3;
int main(int argc, char *argv[]) {
	//BuildingsMobilityModel
	//HybridBuildingsPropagationLossModel
}
*/

#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/mobility-module.h>
#include <ns3/lte-module.h>
#include <ns3/config-store.h>
#include <ns3/buildings-helper.h>
#include <ns3/hybrid-buildings-propagation-loss-model.h>

#include <iomanip>
#include <string>
#include <vector>

using namespace ns3;
using std::vector;

int
main (int argc, char *argv[])
{
  double hEnb = 30.0;
  double hUe = 1.0;
  bool enbIndoor = true;
  bool ueIndoor = true;
  bool verbose = true;
  CommandLine cmd;

  cmd.AddValue("hEnb", "Height of the eNB", hEnb);
  cmd.AddValue("hUe", "Height of UE", hUe);
  cmd.AddValue("enbIndoor", "Boolean for eNB Indoor/Outdoor selection", enbIndoor);
  cmd.AddValue("ueIndoor", "Boolean for UE Indoor/Outdoor selection", ueIndoor);
  cmd.AddValue ("verbose", "Print trace information if true", verbose);

  cmd.Parse(argc, argv);

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults ();

  // parse again so you can override default values from the command line
  cmd.Parse (argc, argv);

  std::ofstream outFile;
  outFile.open ("buildings-pathloss-profiler.out");

  std::ofstream outFile2;
  outFile2.open ("log-pathloss-profiler.out");

  if (!outFile.is_open ())
  {
    NS_FATAL_ERROR ("Can't open output file");
  }

  Ptr<BuildingsMobilityModel> mmEnb = CreateObject<BuildingsMobilityModel> ();
  mmEnb->SetPosition (Vector (0.0, 0.0, hEnb));

  if(enbIndoor)
  {
	//Ptr<Building>building1 = Create<Building>(-2, 2, -2, 2, 0.0, 20.0);
	Ptr<Building> building1 = CreateObject<Building> ();
	building1->SetBoundaries(Box (-2, 2, -2, 2, 0, 20));
	building1->SetBuildingType(Building::Residential);
	building1->SetExtWallsType(Building::ConcreteWithWindows);
	building1->SetNFloors(2);
	building1->SetNRoomsX(2);
	building1->SetNRoomsY(2);

	//mmEnb->SetIndoor(building1, 1, 1, 1);
  }

  BuildingsHelper::MakeConsistent(mmEnb);

  Ptr<HybridBuildingsPropagationLossModel> propagationLossModel = CreateObject<HybridBuildingsPropagationLossModel>();
  // cancel shadowing effect
  propagationLossModel->SetAttribute("ShadowSigmaOutdoor", DoubleValue (0.0));
  propagationLossModel->SetAttribute("ShadowSigmaIndoor", DoubleValue (0.0));
  propagationLossModel->SetAttribute("ShadowSigmaExtWalls", DoubleValue (0.0));

  //propagationLossModel->SetAttribute ("Los2NlosThr", DoubleValue (1550.0));

  //for (uint8_t i = 0; i < 23; i++)
  for (uint32_t i = 1; i < 2300; i++)
  {
	  Ptr<BuildingsMobilityModel> mmUe = CreateObject<BuildingsMobilityModel> ();
	  mmUe->SetPosition (Vector (i, 0.0, hUe));

	  BuildingsHelper::MakeConsistent (mmUe);

	  Ptr<PropagationLossModel> propLM = CreateObject<LogDistancePropagationLossModel>();
	  //mmPp->SetPosition (Vector (i, 0.0, hUe));

	  double loss = propagationLossModel->GetLoss (mmEnb, mmUe);
	  outFile << i << "\t"
			  << loss
			  << std::endl;

	  double dbm = propLM->CalcRxPower(100/1000.0, mmEnb, mmUe);
	  outFile2 << i << "\t"
	  			  << dbm
	  			  << std::endl;
  }

  Simulator::Destroy ();
}
#endif

#if MASSA
/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 Network Security Lab, University of Washington, Seattle.
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
 * Author: Sidharth Nabar <snabar@uw.edu>, He Wu <mdzz@u.washington.edu>
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/energy-module.h"
#include "ns3/internet-module.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

NS_LOG_COMPONENT_DEFINE ("EnergyExample");

using namespace ns3;

std::string
PrintReceivedPacket (Address& from)
{
  InetSocketAddress iaddr = InetSocketAddress::ConvertFrom (from);

  std::ostringstream oss;
  oss << "--\nReceived one packet! Socket: " << iaddr.GetIpv4 ()
      << " port: " << iaddr.GetPort ()
      << " at time = " << Simulator::Now ().GetSeconds ()
      << "\n--";

  return oss.str ();
}

/**
 * \param socket Pointer to socket.
 *
 * Packet receiving sink.
 */
void
ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address from;
  while ((packet = socket->RecvFrom (from)))
    {
      if (packet->GetSize () > 0)
        {
          NS_LOG_UNCOND (PrintReceivedPacket (from));
        }
    }
}

/**
 * \param socket Pointer to socket.
 * \param pktSize Packet size.
 * \param n Pointer to node.
 * \param pktCount Number of packets to generate.
 * \param pktInterval Packet sending interval.
 *
 * Traffic generator.
 */
static void
GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, Ptr<Node> n,
                 uint32_t pktCount, Time pktInterval)
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic, socket, pktSize, n,
                           pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

/// Trace function for remaining energy at node.
void
RemainingEnergy (double oldValue, double remainingEnergy)
{
  //NS_LOG_UNCOND()
  std::cout << Simulator::Now ().GetSeconds ()
                 << "s Current remaining energy = " << remainingEnergy << "J" << std::endl;
}

/// Trace function for total energy consumption at node.
void
TotalEnergy (double oldValue, double totalEnergy)
{
  //NS_LOG_UNCOND()
  //std::cout << Simulator::Now ().GetSeconds ()
   //              << "s Total energy consumed by radio = " << totalEnergy << "J" << std::endl;
}

int
main (int argc, char *argv[])
{

  //LogComponentEnable ("EnergySource", LOG_LEVEL_DEBUG);
  //LogComponentEnable ("BasicEnergySource", LOG_LEVEL_DEBUG);
  ////LogComponentEnable ("DeviceEnergyModel", LOG_LEVEL_DEBUG);
  //LogComponentEnable ("WifiRadioEnergyModel", LOG_LEVEL_DEBUG);


  std::string phyMode ("DsssRate1Mbps");
  double Prss = -80;            // dBm
  uint32_t PpacketSize = 200;   // bytes
  bool verbose = true;

  // simulation parameters
  uint32_t numPackets = 1;  // number of packets to send
  double interval = 3;          // seconds
  double startTime = 0.0;       // seconds
  double distanceToRx = 0.26;  // meters
  /*
   * This is a magic number used to set the transmit power, based on other
   * configuration.
   */
  double offset = 81;

  CommandLine cmd;
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("Prss", "Intended primary RSS (dBm)", Prss);
  cmd.AddValue ("PpacketSize", "size of application packet sent", PpacketSize);
  cmd.AddValue ("numPackets", "Total number of packets to send", numPackets);
  cmd.AddValue ("startTime", "Simulation start time", startTime);
  cmd.AddValue ("distanceToRx", "X-Axis distance between nodes", distanceToRx);
  cmd.AddValue ("verbose", "Turn on all device log components", verbose);
  cmd.Parse (argc, argv);

  // Convert to time object
  Time interPacketInterval = Seconds (interval);

  // disable fragmentation for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold",
                      StringValue ("2200"));
  // turn off RTS/CTS for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold",
                      StringValue ("2200"));
  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
                      StringValue (phyMode));

  NodeContainer c;
  c.Create (2);     // create 2 nodes
  NodeContainer networkNodes;
  networkNodes.Add (c.Get (0));
  networkNodes.Add (c.Get (1));

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents ();
    }
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

  /** Wifi PHY **/
  /***************************************************************************/
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  wifiPhy.Set ("RxGain", DoubleValue (-10));
  wifiPhy.Set ("TxGain", DoubleValue (offset + Prss));
  wifiPhy.Set ("CcaMode1Threshold", DoubleValue (0.0));
  /***************************************************************************/

  /** wifi channel **/
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  // create wifi channel
  Ptr<YansWifiChannel> wifiChannelPtr = wifiChannel.Create ();
  wifiPhy.SetChannel (wifiChannelPtr);

  /** MAC layer **/
  // Add a non-QoS upper MAC, and disable rate control
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",
                                StringValue (phyMode), "ControlMode",
                                StringValue (phyMode));
  // Set it to ad-hoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");

  /** install PHY + MAC **/
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, networkNodes);

  /** mobility **/
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (distanceToRx, distanceToRx, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c);

#if energy
  /** Energy Model **/
  /***************************************************************************/
  /* energy source */
  BasicEnergySourceHelper basicSourceHelper;
  // configure energy source
  basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (0.1));
  // install source
  EnergySourceContainer sources = basicSourceHelper.Install (c);
  /* device energy model */
  WifiRadioEnergyModelHelper radioEnergyHelper;
  // configure radio energy model
  radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
  // install device model
  DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (devices, sources);
#endif

  /** Internet stack **/
  InternetStackHelper internet;
  internet.Install (networkNodes);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.62.7.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (networkNodes.Get (1), tid);  // node 1, receiver
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  Ptr<Socket> source = Socket::CreateSocket (networkNodes.Get (0), tid);    // node 0, sender
  InetSocketAddress remote = InetSocketAddress (Ipv4Address::GetBroadcast (), 80);
  source->SetAllowBroadcast (true);
  source->Connect (remote);

#if energy
  /** connect trace sources **/
  /***************************************************************************/
  // all sources are connected to node 1
  // energy source
  Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources.Get (1));
  basicSourcePtr->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergy));
  // device energy model
  Ptr<DeviceEnergyModel> basicRadioModelPtr =
    basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
  NS_ASSERT (basicRadioModelPtr != NULL);
  basicRadioModelPtr->TraceConnectWithoutContext ("TotalEnergyConsumption", MakeCallback (&TotalEnergy));
  /***************************************************************************/
#endif

  /** simulation setup **/
  // start traffic
  Simulator::Schedule (Seconds (startTime), &GenerateTraffic, source, PpacketSize,
                       networkNodes.Get (0), numPackets, interPacketInterval);

  Simulator::Stop (Seconds (10.0));
  Simulator::Run ();

  printf("new\n");

  Simulator::Stop (Seconds (1.0));
  Simulator::Run ();

  Simulator::Destroy ();

  return 0;
}
#endif

#if funfa
/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008 Timo Bingmann
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
 * Author: Timo Bingmann <timo.bingmann@student.kit.edu>
 */

#include "ns3/propagation-loss-model.h"
#include "ns3/jakes-propagation-loss-model.h"
#include "ns3/constant-position-mobility-model.h"

#include "ns3/config.h"
#include "ns3/string.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/pointer.h"
#include "ns3/gnuplot.h"
#include "ns3/simulator.h"

#include <map>

using namespace ns3;

/// Round a double number to the given precision. e.g. dround(0.234, 0.1) = 0.2
/// and dround(0.257, 0.1) = 0.3
static double dround (double number, double precision)
{
  number /= precision;
  if (number >= 0)
    {
      number = floor (number + 0.5);
    }
  else
    {
      number = ceil (number - 0.5);
    }
  number *= precision;
  return number;
}

static Gnuplot
TestDeterministic (Ptr<PropagationLossModel> model)
{
  Ptr<ConstantPositionMobilityModel> a = CreateObject<ConstantPositionMobilityModel> ();
  Ptr<ConstantPositionMobilityModel> b = CreateObject<ConstantPositionMobilityModel> ();

  Gnuplot plot;

  plot.AppendExtra ("set xlabel 'Distance'");
  plot.AppendExtra ("set ylabel 'rxPower (dBm)'");
  plot.AppendExtra ("set key top right");

  double txPowerDbm = +20; // dBm

  Gnuplot2dDataset dataset;

  dataset.SetStyle (Gnuplot2dDataset::LINES);

  {
    a->SetPosition (Vector (0.0, 0.0, 0.0));

    for (double distance = 0.0; distance < 2500.0; distance += 10.0)
      {
        b->SetPosition (Vector (distance, 0.0, 0.0));

        // CalcRxPower() returns dBm.
        double rxPowerDbm = model->CalcRxPower (txPowerDbm, a, b);

        dataset.Add (distance, rxPowerDbm);

        Simulator::Stop (Seconds (1.0));
        Simulator::Run ();
      }
  }

  std::ostringstream os;
  os << "txPower " << txPowerDbm << "dBm";
  dataset.SetTitle (os.str ());

  plot.AddDataset (dataset);

  plot.AddDataset ( Gnuplot2dFunction ("-94 dBm CSThreshold", "-94.0") );

  return plot;
}

static Gnuplot
TestProbabilistic (Ptr<PropagationLossModel> model, unsigned int samples = 100000)
{
  Ptr<ConstantPositionMobilityModel> a = CreateObject<ConstantPositionMobilityModel> ();
  Ptr<ConstantPositionMobilityModel> b = CreateObject<ConstantPositionMobilityModel> ();

  Gnuplot plot;

  plot.AppendExtra ("set xlabel 'Distance'");
  plot.AppendExtra ("set ylabel 'rxPower (dBm)'");
  plot.AppendExtra ("set zlabel 'Probability' offset 0,+10");
  plot.AppendExtra ("set view 50, 120, 1.0, 1.0");
  plot.AppendExtra ("set key top right");

  plot.AppendExtra ("set ticslevel 0");
  plot.AppendExtra ("set xtics offset -0.5,0");
  plot.AppendExtra ("set ytics offset 0,-0.5");
  plot.AppendExtra ("set xrange [100:]");

  double txPowerDbm = +20; // dBm

  Gnuplot3dDataset dataset;

  dataset.SetStyle ("with linespoints");
  dataset.SetExtra ("pointtype 3 pointsize 0.5");

  typedef std::map<double, unsigned int> rxPowerMapType;

  // Take given number of samples from CalcRxPower() and show probability
  // density for discrete distances.
  {
    a->SetPosition (Vector (0.0, 0.0, 0.0));

    for (double distance = 100.0; distance < 2500.0; distance += 100.0)
      {
        b->SetPosition (Vector (distance, 0.0, 0.0));

        rxPowerMapType rxPowerMap;

        for (unsigned int samp = 0; samp < samples; ++samp)
          {
            // CalcRxPower() returns dBm.
            double rxPowerDbm = model->CalcRxPower (txPowerDbm, a, b);
            rxPowerDbm = dround (rxPowerDbm, 1.0);

            rxPowerMap[ rxPowerDbm ]++;

            Simulator::Stop (Seconds (0.01));
            Simulator::Run ();
          }

        for (rxPowerMapType::const_iterator i = rxPowerMap.begin ();
             i != rxPowerMap.end (); ++i)
          {
            dataset.Add (distance, i->first, (double)i->second / (double)samples);
          }
        dataset.AddEmptyLine ();
      }
  }

  std::ostringstream os;
  os << "txPower " << txPowerDbm << "dBm";
  dataset.SetTitle (os.str ());

  plot.AddDataset (dataset);

  return plot;
}

static Gnuplot
TestDeterministicByTime (Ptr<PropagationLossModel> model,
                         Time timeStep = Seconds (0.001),
                         Time timeTotal = Seconds (1.0),
                         double distance = 100.0)
{
  Ptr<ConstantPositionMobilityModel> a = CreateObject<ConstantPositionMobilityModel> ();
  Ptr<ConstantPositionMobilityModel> b = CreateObject<ConstantPositionMobilityModel> ();

  Gnuplot plot;

  plot.AppendExtra ("set xlabel 'Time (s)'");
  plot.AppendExtra ("set ylabel 'rxPower (dBm)'");
  plot.AppendExtra ("set key center right");

  double txPowerDbm = +20; // dBm

  Gnuplot2dDataset dataset;

  dataset.SetStyle (Gnuplot2dDataset::LINES);

  {
    a->SetPosition (Vector (0.0, 0.0, 0.0));
    b->SetPosition (Vector (distance, 0.0, 0.0));

    Time start = Simulator::Now ();
    while( Simulator::Now () < start + timeTotal )
      {
        // CalcRxPower() returns dBm.
        double rxPowerDbm = model->CalcRxPower (txPowerDbm, a, b);

        Time elapsed = Simulator::Now () - start;
        dataset.Add (elapsed.GetSeconds (), rxPowerDbm);

        Simulator::Stop (timeStep);
        Simulator::Run ();
      }
  }

  std::ostringstream os;
  os << "txPower " << txPowerDbm << "dBm";
  dataset.SetTitle (os.str ());

  plot.AddDataset (dataset);

  plot.AddDataset ( Gnuplot2dFunction ("-94 dBm CSThreshold", "-94.0") );

  return plot;
}

int main (int argc, char *argv[])
{
  GnuplotCollection gnuplots ("main-propagation-loss.pdf");

  {
    Ptr<FriisPropagationLossModel> friis = CreateObject<FriisPropagationLossModel> ();

    Gnuplot plot = TestDeterministic (friis);
    plot.SetTitle ("ns3::FriisPropagationLossModel (Default Parameters)");
    gnuplots.AddPlot (plot);
  }

  {
    Ptr<LogDistancePropagationLossModel> log = CreateObject<LogDistancePropagationLossModel> ();
    log->SetAttribute ("Exponent", DoubleValue (2.5));

    Gnuplot plot = TestDeterministic (log);
    plot.SetTitle ("ns3::LogDistancePropagationLossModel (Exponent = 2.5)");
    gnuplots.AddPlot (plot);
  }

  {
    Ptr<RandomPropagationLossModel> random = CreateObject<RandomPropagationLossModel> ();
    Ptr<ExponentialRandomVariable> expVar = CreateObjectWithAttributes<ExponentialRandomVariable> ("Mean", DoubleValue (50.0));
    random->SetAttribute ("Variable", PointerValue (expVar));

    Gnuplot plot = TestDeterministic (random);
    plot.SetTitle ("ns3::RandomPropagationLossModel with Exponential Distribution");
    gnuplots.AddPlot (plot);
  }

  {
    Ptr<JakesPropagationLossModel> jakes = CreateObject<JakesPropagationLossModel> ();

    // doppler frequency shift for 5.15 GHz at 100 km/h
    Config::SetDefault ("ns3::JakesProcess::DopplerFrequencyHz", DoubleValue (477.9));

    Gnuplot plot = TestDeterministicByTime (jakes, Seconds (0.001), Seconds (1.0));
    plot.SetTitle ("ns3::JakesPropagationLossModel (with 477.9 Hz shift and 1 millisec resolution)");
    gnuplots.AddPlot (plot);
  }

  {
    Ptr<JakesPropagationLossModel> jakes = CreateObject<JakesPropagationLossModel> ();

    // doppler frequency shift for 5.15 GHz at 100 km/h
    Config::SetDefault ("ns3::JakesProcess::DopplerFrequencyHz", DoubleValue (477.9));

    Gnuplot plot = TestDeterministicByTime (jakes, Seconds (0.0001), Seconds (0.1));
    plot.SetTitle ("ns3::JakesPropagationLossModel (with 477.9 Hz shift and 0.1 millisec resolution)");
    gnuplots.AddPlot (plot);
  }

  {
    Ptr<ThreeLogDistancePropagationLossModel> log3 = CreateObject<ThreeLogDistancePropagationLossModel> ();

    Gnuplot plot = TestDeterministic (log3);
    plot.SetTitle ("ns3::ThreeLogDistancePropagationLossModel (Defaults)");
    gnuplots.AddPlot (plot);
  }

  {
    Ptr<ThreeLogDistancePropagationLossModel> log3 = CreateObject<ThreeLogDistancePropagationLossModel> ();
    // more prominent example values:
    log3->SetAttribute ("Exponent0", DoubleValue (1.0));
    log3->SetAttribute ("Exponent1", DoubleValue (3.0));
    log3->SetAttribute ("Exponent2", DoubleValue (10.0));

    Gnuplot plot = TestDeterministic (log3);
    plot.SetTitle ("ns3::ThreeLogDistancePropagationLossModel (Exponents 1.0, 3.0 and 10.0)");
    gnuplots.AddPlot (plot);
  }

  {
    Ptr<NakagamiPropagationLossModel> nak = CreateObject<NakagamiPropagationLossModel> ();

    Gnuplot plot = TestProbabilistic (nak);
    plot.SetTitle ("ns3::NakagamiPropagationLossModel (Default Parameters)");
    gnuplots.AddPlot (plot);
  }

  {
    Ptr<ThreeLogDistancePropagationLossModel> log3 = CreateObject<ThreeLogDistancePropagationLossModel> ();

    Ptr<NakagamiPropagationLossModel> nak = CreateObject<NakagamiPropagationLossModel> ();
    log3->SetNext (nak);

    Gnuplot plot = TestProbabilistic (log3);
    plot.SetTitle ("ns3::ThreeLogDistancePropagationLossModel and ns3::NakagamiPropagationLossModel (Default Parameters)");
    gnuplots.AddPlot (plot);
  }

  gnuplots.GenerateOutput (std::cout);

  // produce clean valgrind
  Simulator::Destroy ();
  return 0;
}
#endif

/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 CTTC
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
 * Author: Nicola Baldo <nbaldo@cttc.es>
 */



#include <iostream>

#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/spectrum-model-ism2400MHz-res1MHz.h>
#include <ns3/spectrum-model-300kHz-300GHz-log.h>
#include <ns3/wifi-spectrum-value-helper.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/waveform-generator.h>
#include <ns3/spectrum-analyzer.h>
#include <ns3/log.h>
#include <string>
#include <iomanip>
#include <ns3/friis-spectrum-propagation-loss.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/mobility-module.h>
#include <ns3/spectrum-helper.h>
#include <ns3/applications-module.h>
#include <ns3/adhoc-aloha-noack-ideal-phy-helper.h>


NS_LOG_COMPONENT_DEFINE ("TestAdhocOfdmAloha");

using namespace ns3;

static bool g_verbose = true;
static uint64_t g_rxBytes;

void
PhyRxEndOkTrace (std::string context, Ptr<const Packet> p)
{
  if (g_verbose)
    {
      std::cout << context << " PHY RX END OK p:" << p << std::endl;
    }
  g_rxBytes += p->GetSize ();
}



/**
 * Store the last pathloss value for each TX-RX pair. This is an
 * example of how the PathlossTrace (provided by some SpectrumChannel
 * implementations) work.
 *
 */
class GlobalPathlossDatabase
{
public:

  /**
   * update the pathloss value
   *
   * \param context
   * \param txPhy the transmitting PHY
   * \param rxPhy the receiving PHY
   * \param lossDb the loss in dB
   */
  void UpdatePathloss (std::string context, Ptr<SpectrumPhy> txPhy, Ptr<SpectrumPhy> rxPhy, double lossDb);

  /**
   * print the stored pathloss values to standard output
   *
   */
  void Print ();

private:
  std::map<uint32_t, std::map<uint32_t, double> > m_pathlossMap;
};

void
GlobalPathlossDatabase::UpdatePathloss (std::string context,
                                        Ptr<SpectrumPhy> txPhy,
                                        Ptr<SpectrumPhy> rxPhy,
                                        double lossDb)
{
  uint32_t txNodeId = txPhy->GetMobility ()->GetObject<Node> ()->GetId ();
  uint32_t rxNodeId = rxPhy->GetMobility ()->GetObject<Node> ()->GetId ();
  m_pathlossMap[txNodeId][rxNodeId] = lossDb;
}

void
GlobalPathlossDatabase::Print ()
{
  for (std::map<uint32_t, std::map<uint32_t, double> >::const_iterator txit = m_pathlossMap.begin ();
       txit != m_pathlossMap.end ();
       ++txit)
    {
      for (std::map<uint32_t, double>::const_iterator rxit = txit->second.begin ();
           rxit != txit->second.end ();
           ++rxit)
        {
          std::cout << txit->first << " --> " << rxit->first << " : " << rxit->second << " dB" << std::endl;
        }
    }
}



int main (int argc, char** argv)
{
  CommandLine cmd;
  double lossDb = 150;
  double txPowerW = 0.1;
  uint64_t phyRate = 500000;
  uint32_t pktSize = 1000;
  double simDuration = 0.5;
  std::string channelType ("ns3::SingleModelSpectrumChannel");
  cmd.AddValue ("verbose", "Print trace information if true", g_verbose);
  cmd.AddValue ("lossDb", "link loss in dB", lossDb);
  cmd.AddValue ("txPowerW", "txPower in Watts", txPowerW);
  cmd.AddValue ("phyRate", "PHY rate in bps", phyRate);
  cmd.AddValue ("pktSize", "packet size in bytes", pktSize);
  cmd.AddValue ("simDuration", "duration of the simulation in seconds", simDuration);
  cmd.AddValue ("channelType", "which SpectrumChannel implementation to be used", channelType);
  cmd.Parse (argc, argv);

  NodeContainer c;
  c.Create (2);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (5.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");


  mobility.Install (c);


  SpectrumChannelHelper channelHelper;
  channelHelper.SetChannel (channelType);
  channelHelper.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  Ptr<MatrixPropagationLossModel> propLoss = CreateObject<MatrixPropagationLossModel> ();
  propLoss->SetLoss (c.Get (0)->GetObject<MobilityModel> (), c.Get (1)->GetObject<MobilityModel> (), lossDb, true);
  channelHelper.AddPropagationLoss (propLoss);
  Ptr<SpectrumChannel> channel = channelHelper.Create ();


  WifiSpectrumValue5MhzFactory sf;

  uint32_t channelNumber = 1;
  Ptr<SpectrumValue> txPsd =  sf.CreateTxPowerSpectralDensity (txPowerW, channelNumber);

  // for the noise, we use the Power Spectral Density of thermal noise
  // at room temperature. The value of the PSD will be constant over the band of interest.
  const double k = 1.381e-23; //Boltzmann's constant
  const double T = 290; // temperature in Kelvin
  double noisePsdValue = k * T; // watts per hertz
  Ptr<SpectrumValue> noisePsd = sf.CreateConstant (noisePsdValue);

  AdhocAlohaNoackIdealPhyHelper deviceHelper;
  deviceHelper.SetChannel (channel);
  deviceHelper.SetTxPowerSpectralDensity (txPsd);
  deviceHelper.SetNoisePowerSpectralDensity (noisePsd);
  deviceHelper.SetPhyAttribute ("Rate", DataRateValue (DataRate (phyRate)));
  NetDeviceContainer devices = deviceHelper.Install (c);

  PacketSocketHelper packetSocket;
  packetSocket.Install (c);

  PacketSocketAddress socket;
  socket.SetSingleDevice (devices.Get (0)->GetIfIndex ());
  socket.SetPhysicalAddress (devices.Get (1)->GetAddress ());
  socket.SetProtocol (1);

  OnOffHelper onoff ("ns3::PacketSocketFactory", Address (socket));
  onoff.SetConstantRate (DataRate (2*phyRate));
  onoff.SetAttribute ("PacketSize", UintegerValue (pktSize));

  ApplicationContainer apps = onoff.Install (c.Get (0));
  apps.Start (Seconds (0.0));
  apps.Stop (Seconds (simDuration));

  Config::Connect ("/NodeList/*/DeviceList/*/Phy/RxEndOk", MakeCallback (&PhyRxEndOkTrace));

  GlobalPathlossDatabase globalPathlossDatabase;
  Config::Connect ("/ChannelList/*/PropagationLoss",
                   MakeCallback (&GlobalPathlossDatabase::UpdatePathloss, &globalPathlossDatabase));

  g_rxBytes = 0;
  Simulator::Stop (Seconds (simDuration + 0.000001));
  Simulator::Run ();

  if (g_verbose)
    {
      globalPathlossDatabase.Print ();

      double throughputBps = (g_rxBytes * 8.0) / simDuration;
      std::cout << "throughput:       " << throughputBps << std::endl;
      std::cout << "throughput:       " << std::setw (20) << std::fixed << throughputBps << " bps" << std::endl;
      std::cout << "phy rate  :       "   << std::setw (20) << std::fixed << phyRate*1.0 << " bps" << std::endl;
      double rxPowerW = txPowerW / (std::pow (10.0, lossDb/10.0));
      double capacity = 20e6*log2 (1.0 + (rxPowerW/20.0e6)/noisePsdValue);
      std::cout << "shannon capacity: "   << std::setw (20) << std::fixed << capacity <<  " bps" << std::endl;

    }



  Simulator::Destroy ();
  return 0;
}

