/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
 /*
  * Copyright (c) 2009 MIRKO BANCHI
  * Copyright (c) 2015 University of Washington
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation;
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTAILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program; if not, write to the Free Software
  * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
  *
  * Authors: Mirko Banchi <mk.banchi@gmail.com>
  *          Sebastien Deronne <sebastien.deronne@gmail.com>
  *          Tom Henderson <tomhend@u.washington.edu>
  *
  * Adapted from wifi-ht-network.cc example
  */
 
 #include <iomanip>
 #include "ns3/command-line.h"
 #include "ns3/config.h"
 #include "ns3/uinteger.h"
 #include "ns3/boolean.h"
 #include "ns3/double.h"
 #include "ns3/string.h"
 #include "ns3/log.h"
 #include "ns3/yans-wifi-helper.h"
 #include "ns3/spectrum-wifi-helper.h"
 #include "ns3/ssid.h"
 #include "ns3/mobility-helper.h"
 #include "ns3/mobility-module.h"
 #include "ns3/internet-stack-helper.h"
 #include "ns3/ipv4-address-helper.h"
 #include "ns3/udp-client-server-helper.h"
 #include "ns3/packet-sink-helper.h"
 #include "ns3/on-off-helper.h"
 #include "ns3/ipv4-global-routing-helper.h"
 #include "ns3/packet-sink.h"
 #include "ns3/yans-wifi-channel.h"
 #include "ns3/multi-model-spectrum-channel.h"
 #include "ns3/propagation-loss-model.h"
 #include "ns3/propagation-environment.h"
 #include "ns3/enum.h"
 #include "ns3/pointer.h"
 #include <math.h>
 #include <iostream>
 //added for flow monitor
 #include "ns3/flow-monitor.h"
 #include "ns3/flow-monitor-helper.h"
 #include "ns3/netanim-module.h"
 
 // This is a simple example of an IEEE 802.11n Wi-Fi network.
 //
 // The main use case is to enable and test SpectrumWifiPhy vs YansWifiPhy
 // for packet error ratio
 //
 // Network topology:
 //
 //  Wi-Fi 192.168.1.0
 //
 //   STA                  AP
 //    * <-- distance -->  *
 //    |                   |
 //    n1                  n2
 //
 // Users may vary the following command-line arguments in addition to the
 // attributes, global values, and default values typically available:
 //
 //    --simulationTime:  Simulation time in seconds [10]
 //    --distance:        meters separation between nodes [50]
 //    --index:           restrict index to single value between 0 and 31 [256]
 //    --wifiType:        select ns3::SpectrumWifiPhy or ns3::YansWifiPhy [ns3::SpectrumWifiPhy]
 //    --errorModelType:  select ns3::NistErrorRateModel or ns3::YansErrorRateModel [ns3::NistErrorRateModel]
 //    --enablePcap:      enable pcap output [false]
 //
 // By default, the program will step through 32 index values, corresponding
 // to the following MCS, channel width, and guard interval combinations:
 //   index 0-7:    MCS 0-7, long guard interval, 20 MHz channel
 //   index 8-15:   MCS 0-7, short guard interval, 20 MHz channel
 //   index 16-23:  MCS 0-7, long guard interval, 40 MHz channel
 //   index 24-31:  MCS 0-7, short guard interval, 40 MHz channel
 // and send UDP for 10 seconds using each MCS, using the SpectrumWifiPhy and the
 // NistErrorRateModel, at a distance of 50 meters.  The program outputs
 // results such as:
 //
 // wifiType: ns3::SpectrumWifiPhy distance: 50m; time: 10; TxPower: 1 dBm (1.3 mW)
 // index   MCS  Rate (Mb/s) Tput (Mb/s) Received Signal (dBm) Noise (dBm) SNR (dB)
 //     0     0      6.50        5.77    7414      -79.71      -93.97       14.25
 //     1     1     13.00       11.58   14892      -79.71      -93.97       14.25
 //     2     2     19.50       17.39   22358      -79.71      -93.97       14.25
 //     3     3     26.00       22.96   29521      -79.71      -93.97       14.25
 //   ...
 //
 
 using namespace ns3;
 
 // Global variables for use in callbacks.
 double g_signalDbmAvg;
 double g_noiseDbmAvg;
 uint32_t g_samples;

 void printOutput(double throughput, uint64_t totalPacketsThrough, ApplicationContainer serverApp, uint32_t payloadSize, bool flag)
 {
   //UDP
   if(flag == 0)
   {  
       double timeNow = Simulator::Now ().GetSeconds ();
       std::cout << "start time"<< timeNow << std::endl;
       double totalPacketsThroughNew = DynamicCast<UdpServer> (serverApp.Get (0))->GetReceived ();
       Simulator::Schedule (Seconds (2.0), &printOutput, throughput, totalPacketsThroughNew, serverApp, payloadSize, 1);
   }
   if(flag == 1)
   {   
       double totalPacketsThroughNew;
       double timeNow1 = Simulator::Now ().GetSeconds ();
       std::cout << "stop time" <<timeNow1 << std::endl;
       totalPacketsThroughNew = DynamicCast<UdpServer> (serverApp.Get (0))->GetReceived () - totalPacketsThrough; 
       throughput = totalPacketsThroughNew * payloadSize * 8 / (2.0 * 1000000.0); //Mbit/s
       double timeNow = Simulator::Now ().GetSeconds ();
       std::cout << timeNow << std::endl;
       std::cout << "throughput is" << throughput << std::endl;
   }    
 }

 void changeDirection(Ptr<Node> ptr, Vector directionNew) 
 {
    ptr -> GetObject<ConstantVelocityMobilityModel>() ->SetVelocity (directionNew);
 }

 void nodePrinterInfo()
 {
	 NodeContainer const & n = NodeContainer::GetGlobal ();
	 for (NodeContainer::Iterator i = n.Begin (); i != n.End (); ++i)
	 {
	 Ptr<Node> node = *i;
	 Ptr<MobilityModel> mob = node->GetObject<MobilityModel> ();
	 uint32_t  nodeID = node->GetId();
	 if (! mob) continue;
	 Vector pos = mob->GetPosition ();
	 Vector vel = mob->GetVelocity ();
	 std::cout << "Node " << nodeID << " is at (" << pos.x << ", " <<  pos.y << ", " << pos.z << ")\n";
	 std::cout << "Node speed" << nodeID << " is (" << vel.x << ", " <<  vel.y << ", " << vel.z << ")\n";
	 std::cout << "With IP address: " << node->GetObject<Ipv4> ()->GetAddress(1, 0).GetLocal() << "\n";
	 }
 }
 
 int outputAlign(ns3::Vector3D pos)
 {
   int count1 = 0;
   if (pos.x < 0)
     {
       count1 += 1;
       if(pos.x <= -10)
       {
         count1 += 1;
       }
     }
     else if(pos.x >= 10)
     {
      count1 += 1;
     }
     if (pos.y < 0)
     {
       count1 += 1;
       if(pos.y <= -10)
       {
         count1 += 1;
       }
     }
     else if(pos.y >= 10)
     {
      count1 += 1;
     }
     if (pos.z < 0)
     {
       count1 += 1;
       if(pos.z <= -10)
       {
         count1 += 1;
       }
     }
     else if(pos.z >= 10)
     {
      count1 += 1;
     }
    return count1;
 }

 void showPosition (Ptr<Node> node, double deltaTime)
 {
   uint32_t nodeId = node->GetId ();
   Ptr<MobilityModel> mobModel = node->GetObject<MobilityModel> ();
   Vector3D pos = mobModel->GetPosition ();
   Vector3D speed = mobModel->GetVelocity ();
   std::cout << std::fixed;
   std::cout << std::setprecision(2);
   int count2 = 0;
   if (Simulator::Now ().GetSeconds () < 10)
   {
     count2 = outputAlign(pos);
     std::cout << std::setw (4) << Simulator::Now ().GetSeconds () <<
     std::setw (11) << nodeId <<
     std::setw (11) << "(" << std::setfill(' ') << pos.x << ", " << std::setfill(' ') << pos.y << ", " << std::setfill(' ') << pos.z << ")" <<
     std::setw (11-count2) << "(" << speed.x << ", " << speed.y << ", " << speed.z << ")" <<
     std::endl;
     Simulator::Schedule (Seconds (deltaTime), &showPosition, node, deltaTime);
   }
   else
   {
     count2 = outputAlign(pos);
     std::cout << std::setw (4) << Simulator::Now ().GetSeconds () <<
     std::setw (10) << nodeId <<
     std::setw (11) << "(" << std::setfill(' ') << pos.x << ", " << std::setfill(' ') << pos.y << ", " << std::setfill(' ') << pos.z << ")" <<
     std::setw (11-count2) << "(" << speed.x << ", " << speed.y << ", " << speed.z << ")" <<
     std::endl;
     Simulator::Schedule (Seconds (deltaTime), &showPosition, node, deltaTime);
   }
   //std::cout << std::setw (4) << Simulator::Now ().GetSeconds () <<
   //std::setw (11) << nodeId <<
   //std::setw (11) << "(" << std::setfill(' ') << pos.x << ", " << std::setfill(' ') << pos.y << ", " << std::setfill(' ') << pos.z << ")" <<
   //std::setw (11) << "(" << speed.x << ", " << speed.y << ", " << speed.z
   //         << ")" <<
   //std::endl;

   //Simulator::Schedule (Seconds (deltaTime), &showPosition, node, deltaTime);
 }

 void MonitorSniffRx (Ptr<const Packet> packet,
                      uint16_t channelFreqMhz,
                      WifiTxVector txVector,
                      MpduInfo aMpdu,
                      SignalNoiseDbm signalNoise)
 
 {
   g_samples++;
   g_signalDbmAvg += ((signalNoise.signal - g_signalDbmAvg) / g_samples);
   g_noiseDbmAvg += ((signalNoise.noise - g_noiseDbmAvg) / g_samples);
 }
 
 NS_LOG_COMPONENT_DEFINE ("WifiSpectrumPerExample");
 
 int main (int argc, char *argv[])
 {
   double simulationTime = 30; //seconds
   uint16_t index = 256;
   std::string wifiType = "ns3::YansWifiPhy";
   std::string errorModelType = "ns3::NistErrorRateModel";
   bool enablePcap = false;
   std::string propagationLossType = "AtG"; //Friis, Oku, Naka
   bool bandWidth20 = true; //true -> BW = 20MHz, false -> BW = 40MHz 
   //shitty new parameters 
   int channelWidth = 20; 
   bool guardinterval = true;
   double frequency = 2.4;
   double txPower = 1;  // dBm (100 mW)
   uint32_t rtsThreshold = 65535;
 
   CommandLine cmd;
   cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
   cmd.AddValue ("index", "restrict index to single value between 0 and 31", index);
   cmd.AddValue ("wifiType", "select ns3::SpectrumWifiPhy or ns3::YansWifiPhy", wifiType);
   cmd.AddValue ("errorModelType", "select ns3::NistErrorRateModel or ns3::YansErrorRateModel", errorModelType);
   cmd.AddValue ("enablePcap", "enable pcap output", enablePcap);
   cmd.AddValue ("propagationLossType", "choose propagation loss model", propagationLossType);
   cmd.AddValue ("bandWidth20", "set the bandwidth as 20MHz or 40 MHz ", bandWidth20);
   cmd.Parse (argc,argv);
 

   //print topology
   std::cout << "* -------------------------------- *" << std::endl;
   std::cout << "* PHASE 1: Table - Constant Moving * " << std::endl;
   std::cout << "* -------------------------------- *" << std::endl;
   std::cout << "\n Network topology:" << std::endl;
   std::cout << "\n  Wi-Fi 192.168.1.0" << std::endl;
   std::cout << "\n  Drone                  AP" << std::endl;
   std::cout << "    *  <-- distance -->  *" << std::endl;
   std::cout << "    |                    |" << std::endl;
   std::cout << "    n1                   n0" << std::endl;

   /* Cout -> Initial Parameters Configuration */
   std::cout << "_______________________________________________________________________________________________________________________" << std::endl;
   std::cout << "\n[wifiType: " << wifiType << "]  [Initial Drone position: (25,0,0) m]  [Frequency: " << frequency << " GHz]  [TxPower: " << txPower << " dBm (" << std::setprecision (2) << std::fixed << std::pow (10.0,txPower / 10.0) << " mW)]" << std::endl;
   std::cout << "\n[Maximum Channel width: " << std::setprecision (0) << std::fixed << channelWidth << " MHz]  [Number of spatial streams: 1]  [MobilityModel: ns3::ConstantPositionMobilityModel]" << std::endl;
   std::cout << "\n[propagationLossType: ns3::AirToGroundPropagationLossModel]  [errorModelType: ns3::YansErrorRateModel]  [SGI: " << std::boolalpha << guardinterval << "]" << std::endl;
   std::cout << "_______________________________________________________________________________________________________________________" << std::endl;
   std::cout << std::setw (4) << "\nTime" <<
     std::setw (14) << "Node ID" <<
     std::setw (15) << "Position" <<
     std::setw (25) << "Speed" <<
     std::endl;

   /* MCS and distance iterating */
<<<<<<< HEAD
=======
   for (uint16_t i = startIndex; i <= stopIndex; i++)
    {
>>>>>>> 560d6d08efb7a9f696e92580974ccc43d7b62c0c
      std::cout << "-----------------------------------------------------------------------------" << std::endl;

       uint32_t payloadSize;
       payloadSize = 972; // 1000 bytes IPv4

       /* Node setting */

       NodeContainer wifiStaNode;
       wifiStaNode.Create (1);
       NodeContainer wifiApNode;
       wifiApNode.Create (1);

       /* PHY setting */

       YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
       SpectrumWifiPhyHelper spectrumPhy = SpectrumWifiPhyHelper::Default ();
       if (wifiType == "ns3::YansWifiPhy")
         {
           YansWifiChannelHelper channel;
           if (propagationLossType == "Friis")
           {
            channel.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (5.180e9));
           }
          else if (propagationLossType == "AtG")
          {
            channel.AddPropagationLoss ("ns3::AirToGroundPropagationLossModel", "Frequency", DoubleValue (5.180e9), "LoS",BooleanValue(true));
          }
          else{
            std::cout << "Wrong propagation loss model!" << std::endl;
            return 0;
          }
          // multi losses? 
           channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
           //channel.SetPropagationDelay ("ns3::RandomPropagationDelayModel");
           phy.SetChannel (channel.Create ());
           phy.Set ("ShortGuardEnabled", BooleanValue (guardinterval));
           phy.Set ("TxPowerStart", DoubleValue (1)); // dBm (1.26 mW)
           phy.Set ("TxPowerEnd", DoubleValue (1));
           phy.Set ("Frequency", UintegerValue (5180));
           //antenna
           uint8_t nStreams = 1;
           phy.Set ("Antennas", UintegerValue (nStreams));
           phy.Set ("MaxSupportedTxSpatialStreams", UintegerValue (nStreams));
           phy.Set ("MaxSupportedRxSpatialStreams", UintegerValue (nStreams));
         }
       else if (wifiType == "ns3::SpectrumWifiPhy")
         {
           Ptr<MultiModelSpectrumChannel> spectrumChannel
             = CreateObject<MultiModelSpectrumChannel> ();
           Ptr<FriisPropagationLossModel> lossModel
             = CreateObject<FriisPropagationLossModel> ();
           lossModel->SetFrequency (5.180e9);
           spectrumChannel->AddPropagationLossModel (lossModel);
 
           Ptr<ConstantSpeedPropagationDelayModel> delayModel
             = CreateObject<ConstantSpeedPropagationDelayModel> ();
           spectrumChannel->SetPropagationDelayModel (delayModel);
 
           spectrumPhy.SetChannel (spectrumChannel);
           spectrumPhy.SetErrorRateModel (errorModelType);
           spectrumPhy.Set ("Frequency", UintegerValue (5180));
           spectrumPhy.Set ("TxPowerStart", DoubleValue (1)); // dBm  (1.26 mW)
           spectrumPhy.Set ("TxPowerEnd", DoubleValue (1));
         }
       else
         {
           NS_FATAL_ERROR ("Unsupported WiFi type " << wifiType);
         }
 
 
       WifiHelper wifi;
       wifi.SetStandard (WIFI_PHY_STANDARD_80211n_5GHZ);
       WifiMacHelper mac;
 
       Ssid ssid = Ssid ("ns380211n");
    
       
       /* MCS data rate setting */ 

       //wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", DataRate, "ControlMode", StringValue ("OfdmRate24Mbps"));
       wifi.SetRemoteStationManager ("ns3::MinstrelHtWifiManager", "RtsCtsThreshold", UintegerValue (rtsThreshold));
       NetDeviceContainer staDevice;
       NetDeviceContainer apDevice;
 
       if (wifiType == "ns3::YansWifiPhy")
         {
           mac.SetType ("ns3::StaWifiMac",
                        "Ssid", SsidValue (ssid));
           staDevice = wifi.Install (phy, mac, wifiStaNode);
           mac.SetType ("ns3::ApWifiMac",
                        "Ssid", SsidValue (ssid));
           apDevice = wifi.Install (phy, mac, wifiApNode);
 
         }
       else if (wifiType == "ns3::SpectrumWifiPhy")
         {
           mac.SetType ("ns3::StaWifiMac",
                        "Ssid", SsidValue (ssid));
           staDevice = wifi.Install (spectrumPhy, mac, wifiStaNode);
           mac.SetType ("ns3::ApWifiMac",
                        "Ssid", SsidValue (ssid));
           apDevice = wifi.Install (spectrumPhy, mac, wifiApNode);
         }
 

       /* mobility setting */
       Ptr<Node> droneptr = wifiStaNode.Get(0);

       MobilityHelper mobility;
       Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

	     positionAlloc->Add (Vector (0.0, 0.0, 0.0)); // AP - position

	     mobility.SetPositionAllocator (positionAlloc);
	     mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	     mobility.Install (wifiApNode);

	     mobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator", "X",StringValue("ns3::UniformRandomVariable[Min=25|Max=25]"),
	                                  "Y",StringValue("ns3::UniformRandomVariable[Min=0|Max=0]"),
	                                  "Z",StringValue("ns3::UniformRandomVariable[Min=0|Max=0]")
	                                 );
       mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
       mobility.Install (wifiStaNode);
       droneptr -> GetObject<ConstantVelocityMobilityModel>() ->SetVelocity (Vector (-4.88,11.79,2.5));

       /* Change directions */
       Simulator::Schedule (Seconds (1.5), &changeDirection, droneptr, Vector(-11.79,4.88,2.5)); //5s later change the direction of drone
       Simulator::Schedule (Seconds (3.0), &changeDirection, droneptr, Vector(0,0,0));
       //Simulator::Schedule (Seconds (9.0), &changeDirection, droneptr, Vector(-11.79,-4.88,2.5));
       /*
       Simulator::Schedule (Seconds (4.5), &changeDirection, droneptr, Vector(-4.88,-11.79,2.5));
       Simulator::Schedule (Seconds (6.0), &changeDirection, droneptr, Vector(4.88,-11.79,2.5));
       Simulator::Schedule (Seconds (7.5), &changeDirection, droneptr, Vector(11.79,-4.88,2.5));
       Simulator::Schedule (Seconds (9.0), &changeDirection, droneptr, Vector(11.79,4.88,2.5));
       Simulator::Schedule (Seconds (10.5), &changeDirection, droneptr, Vector(4.88,11.79,2.5));
       Simulator::Schedule (Seconds (12.0), &changeDirection, droneptr, Vector(-4.88,11.79,2.5));
       Simulator::Schedule (Seconds (13.5), &changeDirection, droneptr, Vector(-11.79,4.88,2.5));
       Simulator::Schedule (Seconds (15.0), &changeDirection, droneptr, Vector(-11.79,-4.88,2.5));
       Simulator::Schedule (Seconds (16.5), &changeDirection, droneptr, Vector(-4.88,-11.79,2.5));
       Simulator::Schedule (Seconds (18.0), &changeDirection, droneptr, Vector(4.88,-11.79,2.5));
       Simulator::Schedule (Seconds (19.5), &changeDirection, droneptr, Vector(11.79,-4.88,2.5));
       Simulator::Schedule (Seconds (21.0), &changeDirection, droneptr, Vector(11.79,4.88,2.5));
       Simulator::Schedule (Seconds (22.5), &changeDirection, droneptr, Vector(4.88,11.79,2.5));
       Simulator::Schedule (Seconds (24.0), &changeDirection, droneptr, Vector(-4.88,11.791,2.5));
       Simulator::Schedule (Seconds (25.5), &changeDirection, droneptr, Vector(-11.79,4.88,2.5));
       Simulator::Schedule (Seconds (27.0), &changeDirection, droneptr, Vector(-11.79,-4.88,2.5));
       Simulator::Schedule (Seconds (28.5), &changeDirection, droneptr, Vector(-4.88,-11.7,2.5));
       */
       
       /* Print position */
       Simulator::Schedule (Seconds (0.0), &showPosition, droneptr, 1.5); //print drone position every 0.1s

       /* Internet stack */

       InternetStackHelper stack;
       stack.Install (wifiApNode);
       stack.Install (wifiStaNode);
 
       Ipv4AddressHelper address;
       address.SetBase ("192.168.1.0", "255.255.255.0");
       Ipv4InterfaceContainer staNodeInterface;
       Ipv4InterfaceContainer apNodeInterface;
 
       staNodeInterface = address.Assign (staDevice);
       apNodeInterface = address.Assign (apDevice);

       /* Setting applications */

       ApplicationContainer serverApp;
       //UDP flow
       uint16_t port = 9;
       UdpServerHelper server (port);
       serverApp = server.Install (wifiStaNode.Get (0));
       serverApp.Start (Seconds (0.0));
       serverApp.Stop (Seconds (simulationTime + 1));
 
       UdpClientHelper client (staNodeInterface.GetAddress (0), port);
       client.SetAttribute ("MaxPackets", UintegerValue (4294967295u));
       client.SetAttribute ("Interval", TimeValue (Time ("0.0001"))); //packets/s
       client.SetAttribute ("PacketSize", UintegerValue (payloadSize));
       ApplicationContainer clientApp = client.Install (wifiApNode.Get (0));
       clientApp.Start (Seconds (1.0));
       clientApp.Stop (Seconds (simulationTime + 1));
         

 
       Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/Phy/MonitorSnifferRx", MakeCallback (&MonitorSniffRx));
 
       if (enablePcap)
         {
           std::stringstream ss;
           ss << "wifi-spectrum-per-example-";
           phy.EnablePcap (ss.str (), apDevice);
         }
       g_signalDbmAvg = 0;
       g_noiseDbmAvg = 0;
       g_samples = 0;
 
       Simulator::Stop (Seconds (simulationTime + 1));

       //Flow Monitor
       Ptr<FlowMonitor> flowMonitor;
       FlowMonitorHelper flowHelper;
       flowMonitor = flowHelper.InstallAll();

       //AnimationInterface anim ("mid1,xml");
      
 
       double throughput = 0;
       uint64_t totalPacketsThrough = 0;
       //Simulator::Schedule (Seconds (2.9), &printOutput, throughput, totalPacketsThrough, serverApp, payloadSize);
       Simulator::Schedule (Seconds (5), &printOutput, throughput, totalPacketsThrough, serverApp, payloadSize, 0);

       Simulator::Run ();


       /* Drone Node positions: */
	     Ptr<Node> const droneNode = wifiStaNode.Get (0);
       Ptr<MobilityModel> mob = droneNode->GetObject<MobilityModel> ();
       Vector pos = mob->GetPosition ();

       /* Compute the maximum rate based on Shannon Equation */
       double rate_Shannon;  // C = W*log2(1 + SNR)S
       double snr_dB = g_signalDbmAvg - g_noiseDbmAvg;
       double snr = pow(10.0, snr_dB/10.0);
       if (bandWidth20) // W = 20 MHz 
       {
    	   rate_Shannon = 20*log2(1 + snr);
	     }
       else // W = 40 MHz 
       {
    	   rate_Shannon = 40*log2(1 + snr);
       }
     

      
       
       /* Printing the Parameters Outputs */
       //std::cout << "\n[wifiType: " << wifiType << "]   [Constant moving]" << "   [time: " << simulationTime << "s]    [TxPower: 1 dBm (1.3 mW)]" << std::endl;
       std::cout << std::setw (5) << "index" <<
       std::setw (17) << "Tput (Mb/s)" <<
       std::setw (12) << "Received " <<
       std::setw (14) << "Signal (dBm)" <<
       std::setw (14) << "Noise (dBm)" <<
       std::setw (12) << "SNR (dB)" <<
       std::setw (18) << " ShannonR (Mb/s) |" <<
       std::setw (23) << " position [x,y,z] (m) |" <<
       std::endl;

       std::cout << std::setw (4) <<
         std::setw (7) << (1) <<
         std::setprecision (2) << std::fixed <<
         std::setw (14) << throughput <<
         std::setw (14) << totalPacketsThrough;

       if (totalPacketsThrough > 0)
         {
           std::cout << std::setw (14) << g_signalDbmAvg <<
             std::setw (15) << g_noiseDbmAvg <<
             std::setw (12) << (g_signalDbmAvg - g_noiseDbmAvg) <<
			 std::setw (14) << rate_Shannon;
         }
       else
         {
           std::cout << std::setw (13) << "N/A" <<
             std::setw (14) << "N/A" <<
             std::setw (14) << "N/A" <<
             std::setw (14) << "N/A";
         }
	     std::cout << std::setw (9) << "(" << pos.x << ", " <<  pos.y << ", " << pos.z << ")" <<
       std::endl;
       

       std::stringstream ssflow;
       ssflow << "mainoutput-MCS-" <<".xml";
       flowMonitor->SerializeToXmlFile(ssflow.str(), true, true);

       Simulator::Destroy ();
      
    
   return 0;
 }