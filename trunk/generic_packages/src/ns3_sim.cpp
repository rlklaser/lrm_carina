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
 * @file ns3_sim.cpp
 * @brief
 * @author Rafael Luiz Klaser <rlklaser@gmail.com>
 * @date Nov 22, 2012
 *
 */

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
#include "ns3/simulator.h"
#include "ns3/node.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Wifi-2-nodes-fixed");

void PrintLocations(NodeContainer nodes, std::string header)
{
	std::cout << header << std::endl;
	for (NodeContainer::Iterator iNode = nodes.Begin(); iNode != nodes.End(); ++iNode)
	{
		Ptr<Node> object = *iNode;
		Ptr<MobilityModel> position = object->GetObject<MobilityModel>();
		NS_ASSERT (position != 0);
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
