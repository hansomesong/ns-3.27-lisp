#include <fstream>
#include <iostream>
#include <algorithm>
#include <vector>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/tinyxml2-module.h"
#include "ns3/rng-seed-manager.h"


using namespace ns3;
using namespace tinyxml2;

NS_LOG_COMPONENT_DEFINE("RouteExperimentTuto");

//------------------------------------------------------
class TimestampTag : public Tag {
public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;

  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);

  // these are our accessors to our tag structure
  void SetTimestamp (Time time);
  Time GetTimestamp (void) const;

  void Print (std::ostream &os) const;

private:
  Time m_timestamp;

  // end class TimestampTag
};

//----------------------------------------------------------------------
//-- TimestampTag
//------------------------------------------------------
TypeId
TimestampTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("TimestampTag")
    .SetParent<Tag> ()
    .AddConstructor<TimestampTag> ()
    .AddAttribute ("Timestamp",
                   "Some momentous point in time!",
                   EmptyAttributeValue (),
                   MakeTimeAccessor (&TimestampTag::GetTimestamp),
                   MakeTimeChecker ())
  ;
  return tid;
}
TypeId
TimestampTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

uint32_t
TimestampTag::GetSerializedSize (void) const
{
  return 8;
}
void
TimestampTag::Serialize (TagBuffer i) const
{
  int64_t t = m_timestamp.GetNanoSeconds ();
  i.Write ((const uint8_t *)&t, 8);
}
void
TimestampTag::Deserialize (TagBuffer i)
{
  int64_t t;
  i.Read ((uint8_t *)&t, 8);
  m_timestamp = NanoSeconds (t);
}

void
TimestampTag::SetTimestamp (Time time)
{
  m_timestamp = time;
}
Time
TimestampTag::GetTimestamp (void) const
{
  return m_timestamp;
}

void
TimestampTag::Print (std::ostream &os) const
{
  os << "t=" << m_timestamp;
}


class RoutingExperiment
{
public:
  RoutingExperiment ();
  void Run ();
  void CommandSetup (int argc, char **argv);

  void CalculateThroughput ();
  void AddThroughputToXml (int nodeId, double throughput);
  void DevRxTrace (std::string context, Ptr<const Packet> p, const Address &address);
  void DevTxTrace (std::string context, Ptr<const Packet> p);
  void AddPacketToXml (int nodeId, int64_t delay);
  void AddPacketCountToXml (int nodeId, int rxPackets, int txPackets);

private:
  std::string m_protocolName;
  uint32_t m_protocol;
  bool m_tcp;
  std::string m_filename;
  uint32_t m_nDrones;
  double m_totalSimulationTime;
  uint32_t m_nDataStreams;
  uint32_t m_startTime;
  double m_appOnTime;
  double m_appOffTime;
  double m_transmissionPower;
  double m_applicationDataRate;
  bool m_tracePackets;
  bool m_traceRouting;
  double m_minDroneSpeed;
  double m_maxDroneSpeed;
  int m_run;
  bool m_measureThroughput;
  int m_packetSize;

  std::vector<std::vector<int64_t> > receivedTimestamps;
  std::vector<int> rxPackets;
  std::vector<int> txPackets;

  Ptr<PacketSink> sink;
  uint64_t lastTotalRx;

  XMLDocument xml_metrics;
  XMLNode* xml_root;
};

RoutingExperiment::RoutingExperiment ()
  : m_protocol (2), // AODV
    m_tcp (false),
    m_filename ("my-adhoc"),
    m_nDrones (30),
    m_totalSimulationTime (100.0),
    m_nDataStreams (1),
    m_startTime (50.0),
    m_appOnTime (0.5),
    m_appOffTime (0),
    m_transmissionPower (20.0),
    m_applicationDataRate (50.0),
    m_tracePackets (false),
    m_traceRouting (false),
    m_minDroneSpeed (4.0),
    m_maxDroneSpeed (5.0),
    m_run (2),
    m_measureThroughput (false),
    m_packetSize(512)
{
  receivedTimestamps.resize(m_nDrones);
  rxPackets.resize(m_nDrones);
  txPackets.resize(m_nDrones);
  lastTotalRx = 0;
}

void
RoutingExperiment::CommandSetup (int argc, char **argv)
{
  CommandLine cmd;
  cmd.AddValue ("protocol", "1=OLSR;2=AODV;3=DSDV;4=DSR", m_protocol);
  cmd.AddValue ("tcp", "Enable TCP, default is UDP", m_tcp);
  cmd.AddValue ("filename", "Choose a filename to save the results", m_filename);
  cmd.AddValue ("drones", "Choose number of drones", m_nDrones);
  cmd.AddValue ("duration", "Choose duration of simulation", m_totalSimulationTime);
  cmd.AddValue ("datastreams", "Set number of communicating drones", m_nDataStreams);
  cmd.AddValue ("starttime", "Set when drones should start sending data", m_startTime);
  cmd.AddValue ("appontime", "Set how long application should burst", m_appOnTime);
  cmd.AddValue ("appofftime", "Set how long application should wait between bursts", m_appOffTime);
  cmd.AddValue ("transmissionpower", "Set the power of Wifi antenna", m_transmissionPower);
  cmd.AddValue ("applicationdatarate", "Set datarate at which the application generates data in kb/s", m_applicationDataRate);
  cmd.AddValue ("tracepackets", "Enable packet tracing", m_tracePackets);
  cmd.AddValue ("tracerouting", "Enable route tracing", m_traceRouting);
  cmd.AddValue ("mindronespeed", "Minimal speed of the drones", m_minDroneSpeed);
  cmd.AddValue ("maxdronespeed", "Maximal speed of the drones", m_maxDroneSpeed);
  cmd.AddValue ("run", "Select run number", m_run);
  cmd.AddValue ("throughput", "Measure throughput", m_measureThroughput);
  cmd.AddValue ("packetsize", "Set size of packets", m_packetSize);
  cmd.Parse (argc, argv);
}

void
RoutingExperiment::CalculateThroughput ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */
  double cur = (sink->GetTotalRx() - lastTotalRx) * (double) 8/1e6;     /* Convert Application RX Packets to MBits. */
  AddThroughputToXml(1, cur);
  lastTotalRx = sink->GetTotalRx ();

  Simulator::Schedule (Seconds (1.0), &RoutingExperiment::CalculateThroughput, this);
}

void
RoutingExperiment::DevRxTrace (std::string context, Ptr<const Packet> p, const Address &address)
{
  // Get the node id from context
  /*std::regex nodeId_regex ("\\/NodeList\\/(\\d)\\/ApplicationList\\/.+");
  std::smatch nodeId_match;
  int nodeId = -1;
  if(std::regex_match(context, nodeId_match, nodeId_regex)) {
    nodeId = std::stoi(nodeId_match[1].str ());
  }*/
  int nodeId = context.at (10) - '0';

  TimestampTag timestamp;
  bool found = p->FindFirstMatchingByteTag (timestamp);
  Time tx = timestamp.GetTimestamp ();
  if (found) {
    // Check if we have already received this packet (same timestamp)
    bool alreadyReceived = (std::find(receivedTimestamps[nodeId].begin (), receivedTimestamps[nodeId].end (), tx.GetNanoSeconds ()) != receivedTimestamps[nodeId].end ());
    if (!alreadyReceived) {
      rxPackets[nodeId] = rxPackets[nodeId] + 1;
      receivedTimestamps[nodeId].push_back (tx.GetNanoSeconds ());

      int64_t delay = (Simulator::Now () - tx).GetNanoSeconds ();
      AddPacketToXml(nodeId, delay);
    }
  }
}

void
RoutingExperiment::DevTxTrace (std::string context, Ptr<const Packet> p)
{
  // Get the node id from context
  /*std::regex nodeId_regex ("\\/NodeList\\/(\\d)\\/ApplicationList\\/.+");
  std::smatch nodeId_match;
  int nodeId = -1;
  if(std::regex_match(context, nodeId_match, nodeId_regex)) {
    nodeId = std::stoi(nodeId_match[1].str ());
  }*/
  int nodeId = context.at (10) - '0';

  txPackets[nodeId] = txPackets[nodeId] + 1;
}

void
RoutingExperiment::AddThroughputToXml (int nodeId, double throughput)
{
  // Search the right node
  XMLElement* xml_node = xml_root->FirstChildElement ("Node");
  int temp_nodeId;
  if (xml_node != NULL) {
    xml_node->QueryIntAttribute ("id", &temp_nodeId);
  }

  while (xml_node != NULL && temp_nodeId != nodeId) {
    xml_node = xml_node->NextSiblingElement ();
    if (xml_node != NULL) {
      xml_node->QueryIntAttribute ("id", &temp_nodeId);
    }
  }

  // Create node in xml if it does not exist
  if (xml_node == NULL) {
    xml_node = xml_metrics.NewElement ("Node");
    xml_node->SetAttribute ("id", nodeId);
    xml_root->InsertEndChild (xml_node);
  }

  XMLElement* xml_throughput = xml_metrics.NewElement ("Throughput");
  xml_throughput->SetAttribute ("throughput", throughput);
  xml_throughput->SetAttribute ("time", (Simulator::Now ()).GetSeconds ());
  xml_node->InsertEndChild (xml_throughput);
}

void
RoutingExperiment::AddPacketToXml (int nodeId, int64_t delaylong)
{
  // Search the right node
  XMLElement* xml_node = xml_root->FirstChildElement ("Node");
  int temp_nodeId;
  if (xml_node != NULL) {
    xml_node->QueryIntAttribute ("id", &temp_nodeId);
  }

  while (xml_node != NULL && temp_nodeId != nodeId) {
    xml_node = xml_node->NextSiblingElement ();
    if (xml_node != NULL) {
      xml_node->QueryIntAttribute ("id", &temp_nodeId);
    }
  }

  // Create node in xml if it does not exist
  if (xml_node == NULL) {
    xml_node = xml_metrics.NewElement ("Node");
    xml_node->SetAttribute ("id", nodeId);
    xml_root->InsertEndChild (xml_node);
  }

  XMLElement* xml_packet = xml_metrics.NewElement ("Packet");
  xml_packet->SetAttribute ("delay", delaylong);
  xml_node->InsertEndChild (xml_packet);
}

void
RoutingExperiment::AddPacketCountToXml (int nodeId, int rxPackets_arg, int txPackets_arg)
{
  // Search the right node
  XMLElement* xml_node = xml_root->FirstChildElement ("Node");
  int temp_nodeId;
  if (xml_node != NULL) {
    xml_node->QueryIntAttribute ("id", &temp_nodeId);
  }

  while (xml_node != NULL && temp_nodeId != nodeId) {
    xml_node = xml_node->NextSiblingElement ();
    if (xml_node != NULL) {
      xml_node->QueryIntAttribute ("id", &temp_nodeId);
    }
  }

  // Create node in xml if it does not exist
  if (xml_node == NULL) {
    xml_node = xml_metrics.NewElement ("Node");
    xml_node->SetAttribute ("id", nodeId);
    xml_root->InsertEndChild (xml_node);
  }

  xml_node->SetAttribute ("rxPackets", rxPackets_arg);
  xml_node->SetAttribute ("txPackets", txPackets_arg);
}

int
main (int argc, char *argv[])
{
  LogComponentEnable ("RouteExperimentTuto", LOG_LEVEL_ALL);
  RoutingExperiment experiment;
  experiment.CommandSetup (argc,argv);
  experiment.Run ();
}

void
RoutingExperiment::Run ()
{
  // Select a non overlapping substream of the rng
  SeedManager::SetRun (m_run);

  NodeContainer adhocNodes;
  adhocNodes.Create (m_nDrones);

  /****************************************************************************
    WIFI SETUP
  ****************************************************************************/

  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211n_2_4GHZ);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue ("HtMcs1"),
                                "ControlMode", StringValue ("HtMcs1"),
                                "NonUnicastMode", StringValue ("HtMcs1"));

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());

  if (m_transmissionPower > 20) {
    m_transmissionPower = 20;
  }
  wifiPhy.Set ("TxPowerStart",DoubleValue (m_transmissionPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (m_transmissionPower));

  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");

  NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, adhocNodes);

  /****************************************************************************
    MOBILITY MODEL
  ****************************************************************************/

  int nodePause = 3; // in s

  MobilityHelper mobilityAdhoc;

  ObjectFactory pos;
  pos.SetTypeId ("ns3::GridPositionAllocator");
  pos.Set("MinX", DoubleValue (-500.0));
  pos.Set("MinY", DoubleValue (-500.0));
  pos.Set("DeltaX", DoubleValue (20.0));
  pos.Set("DeltaY", DoubleValue (20.0));
  pos.Set("GridWidth", UintegerValue (50));
  pos.Set("LayoutType", StringValue ("UniformRandom"));

  Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();

  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min=" << m_minDroneSpeed << "|Max=" << m_maxDroneSpeed << "]";
  std::stringstream ssPause;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";
  mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc));

  mobilityAdhoc.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                      "rho", DoubleValue(500.0),
                                      "X", DoubleValue(0.0),
                                      "Y", DoubleValue(0.0));
  mobilityAdhoc.Install (adhocNodes);

  /****************************************************************************
    ROUTING PROTOCOL
  ****************************************************************************/

  AodvHelper aodv;
  OlsrHelper olsr;
  DsdvHelper dsdv;
  DsrHelper dsr;
  DsrMainHelper dsrMain;
  Ipv4ListRoutingHelper list;
  InternetStackHelper internet;

  switch (m_protocol)
    {
    case 1:
      list.Add (olsr, 100);
      m_protocolName = "OLSR";
      break;
    case 2:
      //aodv.Set("EnableHello", BooleanValue (false));
      list.Add (aodv, 100);
      m_protocolName = "AODV";
      break;
    case 3:
      list.Add (dsdv, 100);
      m_protocolName = "DSDV";
      break;
    case 4:
      m_protocolName = "DSR";
      break;
    default:
      NS_FATAL_ERROR ("No such protocol:" << m_protocol);
    }

  if (m_protocol < 4)
    {
      internet.SetRoutingHelper (list);
      internet.Install (adhocNodes);
    }
  else if (m_protocol == 4)
    {
      internet.Install (adhocNodes);
      dsrMain.Install (dsr, adhocNodes);
    }

  NS_LOG_INFO ("assigning ip address");

  Ipv4AddressHelper addressAdhoc;
  addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer adhocInterfaces;
  adhocInterfaces = addressAdhoc.Assign (adhocDevices);

  /****************************************************************************
    APPLICATION
  ****************************************************************************/

  uint16_t port = 9;

  std::string TransportLayer;
  if (m_tcp) {
    TransportLayer = "ns3::TcpSocketFactory";
    m_packetSize = 1448;
  } else {
    TransportLayer = "ns3::UdpSocketFactory";
    m_packetSize = 1472;
  }

  int dataStreamLimit = std::min(m_nDrones, 2 * m_nDataStreams);
  Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();

  for (int i = 0; i < dataStreamLimit; i += 2) {
    OnOffHelper onoff (TransportLayer, Address (InetSocketAddress (adhocInterfaces.GetAddress (i + 1), port)));

    std::stringstream dataRate;
    dataRate << m_applicationDataRate << "kb/s";
    onoff.SetConstantRate (DataRate (dataRate.str ()));

    std::stringstream onTime;
    std::stringstream offTime;
    onTime << "ns3::ConstantRandomVariable[Constant=" << m_appOnTime << "]";
    offTime << "ns3::ConstantRandomVariable[Constant=" << m_appOffTime << "]";
    onoff.SetAttribute ("OnTime", StringValue (onTime.str ()));
    onoff.SetAttribute ("OffTime", StringValue (offTime.str ()));
    //onoff.SetAttribute ("MaxBytes", UintegerValue (512));
    onoff.SetAttribute ("PacketSize", UintegerValue (m_packetSize));

    // Random variable so that all nodes do not start sending data at exactly the same time
    ApplicationContainer apps = onoff.Install (adhocNodes.Get (i));
    apps.Start (Seconds (var->GetValue (m_startTime, m_startTime + 1)));
    apps.Stop (Seconds (m_totalSimulationTime));

    // Install the corresponding sink and start it in the first second of simulation
    PacketSinkHelper sinkHelper (TransportLayer, Address (InetSocketAddress (adhocInterfaces.GetAddress (i + 1), port)));
    ApplicationContainer appSink = sinkHelper.Install (adhocNodes.Get (i + 1));
    appSink.Start (Seconds (1.0));
    appSink.Stop (Seconds (m_totalSimulationTime));

    if (m_measureThroughput && i == 0) {
      Simulator::Schedule (Seconds (m_startTime + 1), &RoutingExperiment::CalculateThroughput, this);
      sink = StaticCast<PacketSink> (appSink.Get (0));
    }

    std::stringstream nodeSink;
    std::stringstream nodeApp;
    nodeSink << i + 1;
    nodeApp << i;
    // Make callback to measure delay
    Config::Connect ("/NodeList/" + nodeSink.str () + "/ApplicationList/0/$ns3::PacketSink/Rx", MakeCallback (&RoutingExperiment::DevRxTrace, this));
    Config::Connect ("/NodeList/" + nodeApp.str () + "/ApplicationList/0/$ns3::OnOffApplication/Tx", MakeCallback (&RoutingExperiment::DevTxTrace, this));
  }



  /****************************************************************************
    SIMULATION AND MONITORING
  ****************************************************************************/

  xml_root = xml_metrics.NewElement("Root");
  xml_metrics.InsertFirstChild(xml_root);

  // Packet drop reasons https://www.nsnam.org/doxygen/group__flow-monitor.html#ga035ac54633cca2eec144e14f0dcf4179

  Ptr<FlowMonitor> flowmon;
  FlowMonitorHelper flowmonHelper;
  flowmon = flowmonHelper.InstallAll ();

  //AnimationInterface anim (m_filename + "_animation.xml");
  //if (!m_tracePackets) {
  //  anim.SkipPacketTracing();
  //}
  //anim.EnablePacketMetadata (true);
  //if (m_traceRouting) {
  //  anim.EnableIpv4RouteTracking (m_filename + "_routing.xml", Seconds(0), Seconds(5), Seconds(0.25));
  //}

  NS_LOG_INFO ("Run Simulation.");

  Simulator::Stop (Seconds (m_totalSimulationTime));
  Simulator::Run ();

  flowmon->SerializeToXmlFile (m_filename + "_flows.xml", true, true);

  Simulator::Destroy ();

  for (int i = 0; i < dataStreamLimit; i = i + 2) {
    AddPacketCountToXml (i + 1, rxPackets[i + 1], txPackets[i]);
  }

  xml_metrics.SaveFile ((m_filename + "_metrics.xml").c_str ());
}
