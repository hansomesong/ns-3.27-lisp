/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 University of Liege
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
 * Author: Lionel Agbodjan <lionel.agbodjan@gmail.com>
 */


/*
 * RUN Command:
 * NS_LOG="LispOverIpv4Impl:LispProtocol:SimpleMapTables:LispHelper=level_all|prefix_node:Ipv4L3Protocol=level_debug|prefix_node|prefix_level" ./waf --run test-abilene
 *
 * RUN DEBUG Command:
 * NS_LOG="LispOverIpv4Impl:SimpleMapTables:LispHelper=level_all|prefix_node:Ipv4L3Protocol=level_debug|prefix_node|prefix_level" ./waf --run test-abilene --command-template="gdb --args %s <args>"
 *
 * Network topology : Topology of the abilene network
 *  R0 (0) (4, 3)
 *  | \
 *  |  \
 *  |   \    MS              MR
 *  |    \                               R8 (8)------------R9 (9)
 *  |     \                               \                /
 *  |    R3 (3)---------------R4 (4) -----R7 (7)        R10 (10)
 *  |   /                     |            |            /
 *  |  /                      |           R6 (6)-------
 *  | /                       |           /
 *  R1 (1)                    |         /
 *    \                       |       /
 *     \                      |     /
 *      R2 (2)----------------R5 (5)
 *
 * R0 - (4, 3)
 * R1 - (4, 8)
 * R2 - (6, 10)
 * R3 - (7, 6)
 * R4 - (10, 6)
 * R5 - (10, 11)
 * R6 - (14, 9)
 * R7 - (13, 5)
 * R8 - (12, 4)
 * R9 - (18, 5)
 * R10 - (17, 7)
 */

#include <iostream>
#include <fstream>
#include <string>
#include <cassert>

#include "ns3/netanim-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/map-resolver-helper.h"
#include "ns3/map-server-helper.h"
#include "ns3/lisp-etr-itr-app-helper.h"

//#include "ns3/core-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TestAbilene");


int
main (int argc, char *argv[])
{

  bool verbose = true;

  std::string animFile = "lisp-abilene-animation.xml" ;  // Name of file for animation output

  LogComponentEnable ("TestAbilene", LOG_LEVEL_ALL);
  // Set up some default values for the simulation.  Use the
  Config::SetDefault ("ns3::OnOffApplication::PacketSize", UintegerValue (210));
  Config::SetDefault ("ns3::OnOffApplication::DataRate", StringValue ("448kb/s"));

  // Command line arguments
  CommandLine cmd;
  bool enableFlowMonitor = false;
  cmd.AddValue ("EnableMonitor", "Enable Flow Monitor", enableFlowMonitor);
  cmd.AddValue ("verbose", "Tell application to log if true", verbose);
  cmd.AddValue ("animFile",  "File Name for Animation Output", animFile);
  cmd.Parse (argc,argv);

  //
  NS_LOG_INFO ("Create nodes.");
  NodeContainer routers;
  routers.Create (11);



  NodeContainer r0r3 = NodeContainer (routers.Get (0), routers.Get (3));
  NodeContainer r0r1 = NodeContainer (routers.Get (0), routers.Get (1));
  NodeContainer r1r3 = NodeContainer (routers.Get (1), routers.Get (3));
  NodeContainer r1r2 = NodeContainer (routers.Get (1), routers.Get (2));
  NodeContainer r3r4 = NodeContainer (routers.Get (3), routers.Get (4));
  NodeContainer r2r5 = NodeContainer (routers.Get (2), routers.Get (5));
  NodeContainer r4r5 = NodeContainer (routers.Get (4), routers.Get (5));
  NodeContainer r4r7 = NodeContainer (routers.Get (4), routers.Get (7));
  NodeContainer r5r6 = NodeContainer (routers.Get (5), routers.Get (6));
  NodeContainer r6r7 = NodeContainer (routers.Get (6), routers.Get (7));
  NodeContainer r7r8 = NodeContainer (routers.Get (7), routers.Get (8));
  NodeContainer r8r9 = NodeContainer (routers.Get(8), routers.Get (9));
  NodeContainer r6r10 = NodeContainer (routers.Get(6), routers.Get (10));
  NodeContainer r9r10 = NodeContainer (routers.Get (9), routers.Get (10));

  // make them all routers
  InternetStackHelper internet;
  internet.Install(routers);

  // terminal nodes
  NodeContainer endHosts;
  endHosts.Create (4);
  NodeContainer t0r0 = NodeContainer (endHosts.Get (0), routers.Get (0));
  NodeContainer t0r1 = NodeContainer (endHosts.Get (0), routers.Get (0));
  NodeContainer t1r0 = NodeContainer (endHosts.Get (1), routers.Get (0));
  NodeContainer t1r1 = NodeContainer (endHosts.Get (1), routers.Get (1));

  NodeContainer r9t2 = NodeContainer (routers.Get (9), endHosts.Get (2));
  NodeContainer r10t2 = NodeContainer (routers.Get (10), endHosts.Get (2));
  NodeContainer r9t3 = NodeContainer (routers.Get (9), endHosts.Get (3));
  NodeContainer r10t3 = NodeContainer (routers.Get (10), endHosts.Get (3));

  // install
  internet.Install (endHosts);

  NodeContainer mrms;
  mrms.Create (2);
  NodeContainer mrr3 = NodeContainer (mrms.Get(0), routers.Get (3));
  NodeContainer msr4 = NodeContainer (mrms.Get(1), routers.Get (4));

  // install servers
  internet.Install (mrms);

  // create channels without any IP addressing info
  NS_LOG_INFO ("Create channels");
  PointToPointHelper p2p;
  p2p.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("2ms"));

  // routers
  NetDeviceContainer d0d3 = p2p.Install (r0r3);
  NetDeviceContainer d0d1 = p2p.Install (r0r1);
  NetDeviceContainer d1d3 = p2p.Install (r1r3);
  NetDeviceContainer d1d2 = p2p.Install (r1r2);
  NetDeviceContainer d3d4 = p2p.Install (r3r4);
  NetDeviceContainer d2d5 = p2p.Install (r2r5);
  NetDeviceContainer d4d5 = p2p.Install (r4r5);
  NetDeviceContainer d4d7 = p2p.Install (r4r7);
  NetDeviceContainer d5d6 = p2p.Install (r5r6);
  NetDeviceContainer d6d7 = p2p.Install (r6r7);
  NetDeviceContainer d7d8 = p2p.Install (r7r8);
  NetDeviceContainer d8d9 = p2p.Install (r8r9);
  NetDeviceContainer d6d10 = p2p.Install (r6r10);
  NetDeviceContainer d9d10 = p2p.Install (r9r10);

  // servers and End hosts
  NetDeviceContainer dt0d0 = p2p.Install (t0r0);
  NetDeviceContainer dt0d1 = p2p.Install (t0r1);
  NetDeviceContainer dt1d0 = p2p.Install (t1r0);
  NetDeviceContainer dt1d1 = p2p.Install (t1r1);

  NetDeviceContainer d9dt2 = p2p.Install (r9t2);
  NetDeviceContainer d10dt2 = p2p.Install (r10t2);
  NetDeviceContainer d9dt3 = p2p.Install (r9t3);
  NetDeviceContainer d10dt3 = p2p.Install (r10t3);

  NetDeviceContainer dmrd3 = p2p.Install (mrr3);
  NetDeviceContainer dmsd4 = p2p.Install (msr4);
  NetDeviceContainer dmrdms = p2p.Install (mrms);
  // We add IP addresses
  NS_LOG_INFO ("Assign IP Addresses.");
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i0i3 = ipv4.Assign (d0d3);

  ipv4.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer i0i1 = ipv4.Assign (d0d1);

  ipv4.SetBase ("10.1.3.0", "255.255.255.0");
  Ipv4InterfaceContainer i1i3 = ipv4.Assign (d1d3);

  ipv4.SetBase ("10.1.4.0", "255.255.255.0");
  Ipv4InterfaceContainer i1i2 = ipv4.Assign (d1d2);

  ipv4.SetBase ("10.1.5.0", "255.255.255.0");
  Ipv4InterfaceContainer i3i4 = ipv4.Assign (d3d4);

  ipv4.SetBase ("10.1.6.0", "255.255.255.0");
  Ipv4InterfaceContainer i2i5 = ipv4.Assign (d2d5);

  ipv4.SetBase ("10.1.7.0", "255.255.255.0");
  Ipv4InterfaceContainer i4i5 = ipv4.Assign (d4d5);

  ipv4.SetBase ("10.1.8.0", "255.255.255.0");
  Ipv4InterfaceContainer i4i7 = ipv4.Assign (d4d7);

  ipv4.SetBase ("10.1.9.0", "255.255.255.0");
  Ipv4InterfaceContainer i5i6 = ipv4.Assign (d5d6);

  ipv4.SetBase ("10.1.10.0", "255.255.255.0");
  Ipv4InterfaceContainer i6i7 = ipv4.Assign (d6d7);

  ipv4.SetBase ("10.1.11.0", "255.255.255.0");
  Ipv4InterfaceContainer i7i8 = ipv4.Assign (d7d8);

  ipv4.SetBase ("10.1.12.0", "255.255.255.0");
  Ipv4InterfaceContainer i8i9 = ipv4.Assign (d8d9);

  ipv4.SetBase ("10.1.13.0", "255.255.255.0");
  Ipv4InterfaceContainer i6i10 = ipv4.Assign (d6d10);

  ipv4.SetBase ("10.1.14.0", "255.255.255.0");
  Ipv4InterfaceContainer i9i10 = ipv4.Assign (d9d10);

  // connection with endhosts
  // endhost 0 with r0
  ipv4.SetBase ("10.1.15.0", "255.255.255.0");
  Ipv4InterfaceContainer it0i0 = ipv4.Assign (dt0d0);

  // endhost 0 with r1
  ipv4.SetBase ("10.1.16.0", "255.255.255.0");
  Ipv4InterfaceContainer it0i1 = ipv4.Assign (dt0d1);

  // endhost 1 with r0
  ipv4.SetBase ("10.1.17.0", "255.255.255.0");
  Ipv4InterfaceContainer it1i0 = ipv4.Assign (dt1d0);

  // endhost 1 with r1
  ipv4.SetBase ("10.1.18.0", "255.255.255.0");
  Ipv4InterfaceContainer it1i1 = ipv4.Assign (dt1d1);

  // r9 with endhost2
  ipv4.SetBase ("10.1.19.0", "255.255.255.0");
  Ipv4InterfaceContainer i9it2 = ipv4.Assign (d9dt2);

  // r10 with endhost2
    ipv4.SetBase ("10.1.20.0", "255.255.255.0");
    Ipv4InterfaceContainer i10it2 = ipv4.Assign (d10dt2);

  // r9 with endhost 3
  ipv4.SetBase ("10.1.21.0", "255.255.255.0");
  Ipv4InterfaceContainer i9it3 = ipv4.Assign (d9dt3);

  // r10 with endhost 3
  ipv4.SetBase ("10.1.22.0", "255.255.255.0");
  Ipv4InterfaceContainer i10it3 = ipv4.Assign (d10dt3);

  // Connecting servers
  // mr and r3
  ipv4.SetBase ("10.1.23.0", "255.255.255.0");
  Ipv4InterfaceContainer imri3 = ipv4.Assign (dmrd3);

  // ms and r4
  ipv4.SetBase ("10.1.24.0", "255.255.255.0");
  Ipv4InterfaceContainer imsi4 = ipv4.Assign (dmsd4);

  // ms and mr
  ipv4.SetBase ("10.1.25.0", "255.255.255.0");
  Ipv4InterfaceContainer imrims = ipv4.Assign (dmrdms);
  // Create router node
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  // finally install lisp
  NS_LOG_INFO ("Install Lisp");
  NodeContainer lispRouters = NodeContainer (routers.Get (0), routers.Get (1), routers.Get (9), routers.Get (10));
  LispHelper lispHelper;
  lispHelper.BuildRlocsSet ("./abilene_rlocs.txt");
  lispHelper.Install(lispRouters);
  lispHelper.BuildMapTables2("./abilene_rloc_config_xml.txt");
  lispHelper.InstallMapTables(lispRouters);


  NS_LOG_WARN ("Example: Lisp is successfully aggregated");

  // Initializing Map Resolver Ddt
  MapResolverDdtHelper mapResDdtHelp;
  mapResDdtHelp.SetMapServerAddress (static_cast<Address> (Ipv4Address ("10.1.25.2")));
  ApplicationContainer mrDdtApps = mapResDdtHelp.Install (mrms.Get (0));
  mrDdtApps.Start (Seconds (1.0));
  mrDdtApps.Stop (Seconds (5.0));

  LispEtrItrAppHelper lispAppHelper;
  Ptr<Locator> mrLocator = Create<Locator> (imri3.GetAddress (0));
  lispAppHelper.AddMapResolverRlocs (mrLocator);
  lispAppHelper.AddMapServerAddress (static_cast<Address> (Ipv4Address ("10.1.24.1")));
  ApplicationContainer mapResClientApps = lispAppHelper.Install (lispRouters);
  mapResClientApps.Start (Seconds (1.0));
  mapResClientApps.Stop (Seconds (5.0));

  // initializing Map Server ddt helper
  MapServerDdtHelper mapServerDdtHelp;
  ApplicationContainer mapServDdtApps = mapServerDdtHelp.Install (mrms.Get (1));
  mapServDdtApps.Start (Seconds (1.0));
  mapServDdtApps.Stop (Seconds (5.0));

  // Create the OnOff applications to send udp datagrams of size
  // 210 bytes at a rate 448 kb/s
  NS_LOG_INFO ("Create Applications.");
  uint16_t port = 9;   // Discard port (RFC 863)
  OnOffHelper onoff ("ns3::UdpSocketFactory",
                     Address (InetSocketAddress (i10it3.GetAddress (1), port))); // to endhost 3
  onoff.SetConstantRate (DataRate ("448kb/s"));
  ApplicationContainer apps = onoff.Install (endHosts.Get (0)); // from endhost 0
  apps.Start (Seconds (1.0));
  apps.Stop (Seconds (5.0));

  // Create a packet sink to receive these packets
  PacketSinkHelper sink ("ns3::UdpSocketFactory",
                         Address (InetSocketAddress (Ipv4Address::GetAny (), port)));
  apps = sink.Install (endHosts.Get (3));
  apps.Start (Seconds (1.0));
  apps.Stop (Seconds (4.0));

  AsciiTraceHelper ascii;
  p2p.EnableAsciiAll (ascii.CreateFileStream ("lisp-routing.tr"));
  p2p.EnablePcapAll ("lisp-routing-final");

  // Flow Monitor
  FlowMonitorHelper flowmonHelper;
  if (enableFlowMonitor)
    {
      flowmonHelper.InstallAll ();
    }

  // Create the animation object and configure for specified output
  AnimationInterface anim (animFile);
  // node positions
  anim.SetConstantPosition(routers.Get (0), 4, 3);

  anim.SetConstantPosition(routers.Get (1), 4, 8);
  anim.UpdateNodeColor(routers.Get (1), 0, 153, 0);
  anim.SetConstantPosition(routers.Get (2), 6, 10);
  anim.SetConstantPosition(routers.Get (3), 7, 6);
  anim.SetConstantPosition(routers.Get (4), 10, 6);
  anim.SetConstantPosition(routers.Get (5), 10, 11);
  anim.SetConstantPosition(routers.Get (6), 14, 9);
  anim.SetConstantPosition(routers.Get (7), 13, 5);
  anim.SetConstantPosition(routers.Get (8), 12, 4);
  anim.SetConstantPosition(routers.Get (9), 18, 5);
  anim.UpdateNodeColor(routers.Get (9), 0, 153, 0);
  anim.SetConstantPosition(routers.Get (10), 17, 7);
  anim.UpdateNodeColor(routers.Get (10), 0, 153, 0);
  //terminal nodes
  anim.SetConstantPosition(endHosts.Get (0), 2, 3);
  anim.UpdateNodeColor(endHosts.Get (0), 0, 0, 204);
  anim.SetConstantPosition(endHosts.Get (1), 2, 8);
  anim.UpdateNodeColor(endHosts.Get (1), 0, 0, 204);

  anim.SetConstantPosition(endHosts.Get (2), 19, 3);
  anim.UpdateNodeColor(endHosts.Get (2), 0, 0, 204);
  anim.SetConstantPosition(endHosts.Get (3), 20, 5);
  anim.UpdateNodeColor(endHosts.Get (3), 0, 0, 204);

  // servers
  anim.SetConstantPosition(mrms.Get (0), 7, 4);
  anim.UpdateNodeColor(mrms.Get (0), 255, 128, 0);
  anim.SetConstantPosition(mrms.Get (1), 10, 4);
  anim.UpdateNodeColor(mrms.Get (1), 204, 204, 0);
  anim.UpdateNodeColor(routers.Get (0), 0, 153, 0);

  anim.EnablePacketMetadata (true); // Optional
  anim.EnableIpv4L3ProtocolCounters (Seconds (0), Seconds (5)); // Optional

  NS_LOG_INFO ("Run Simulation.");
  Simulator::Stop (Seconds (5));
  Simulator::Run ();
  std::cout << "Animation Trace file created:" << animFile.c_str ()<< std::endl;
  NS_LOG_INFO ("Done.");

  if (enableFlowMonitor)
    {
      flowmonHelper.SerializeToXmlFile ("lisp-routing.flowmon", false, false);
    }

  Simulator::Destroy ();
  return 0;
}
