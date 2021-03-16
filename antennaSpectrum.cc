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
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
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
  *
  * Edited by: Gerard Giramé Rizzo <gerardgr@kth.se>
  * Date: Oct 16, 2020
  *
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
 #include "ns3/spectrum-wifi-phy.h"
 #include "ns3/spectrum-wifi-helper.h"
 #include "ns3/ssid.h"
 #include "ns3/mobility-helper.h"
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
 #include "ns3/air-to-ground-propagation-loss-model.h"
 #include "ns3/antenna-model.h"
 #include "ns3/isotropic-antenna-model.h"
 #include "ns3/core-module.h"
 #include "ns3/point-to-point-module.h"
 //added for flow monitor
 #include "ns3/flow-monitor.h"
 #include "ns3/flow-monitor-helper.h"
 #include "ns3/network-module.h"
 #include "ns3/applications-module.h"
 #include "ns3/wifi-module.h"
 #include "ns3/mobility-module.h"
 #include "ns3/csma-module.h"
 #include "ns3/internet-module.h"
 #include <math.h>
 #include <vector>

 // This is a simple example of an IEEE 802.11n Wi-Fi network.
 //
 // The main use case is to enable and test SpectrumWifiPhy vs YansWifiPhy
 // for packet error ratio
 //
 // Network topology:
 //
 //  Wi-Fi 192.168.1.0
 //
 //  Drone                  AP
 //    *  <-- distance -->  *
 //    |                    |
 //    n1                   n0
 //
 // Users may vary the following command-line arguments in addition to the
 // attributes, global values, and default values typically available:
 //
 //    --simulationTime:  Simulation time in seconds [10]
 //    --udp:             UDP if set to 1, TCP otherwise [true]
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

/*	_______________________________________________________________________________________________________________________
	[wifiType: ns3::YansWifiPhy]  [Initial Drone position: (12.5,0,0) m]  [Frequency: 5 GHz]  [TxPower: 20 dBm (100.00 mW)]
	[Maximum Channel width: 40 MHz]  [Number of spatial streams: X]  [MobilityModel: ns3::ConstantPositionMobilityModel]
	[propagationLossType: ns3::AirToGroundPropagationLossModel]  [errorModelType: ns3::YansErrorRateModel]  [SGI: false]
	 ______________________________________________________________________________________________________________________
	Iteration | Thput (Mb/s) | Rx Packets | Signal (dBm) | Noise (dBm) | SNR (dB) | position [x,y,z] (m) |
	---------------------------------------------------------------------------------------------------------
	0          77.75         99986        -46.11         -90.96       44.85      (12.50, 0.00, 0.00)
	1          77.72         99946        -46.23         -90.96       44.73      (8.85, 8.85, 1.88)
	2          77.75         99984        -46.52         -90.96       44.44      (0.00, 12.50, 3.75)
	3          77.69         99907        -47.00         -90.96       43.96      (-8.85, 8.85, 5.62)
	4          77.20         99282        -47.58         -90.96       43.38      (-12.50, 0.00, 7.50)
													...
*/

 using namespace ns3;

 // Global variables for use in callbacks.
 double g_signalDbmAvg;
 double g_noiseDbmAvg;
 uint32_t g_samples;

 void MonitorSniffRx (Ptr<const Packet> packet, uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise)
 {
   g_samples++;
   g_signalDbmAvg += ((signalNoise.signal - g_signalDbmAvg) / g_samples);
   g_noiseDbmAvg += ((signalNoise.noise - g_noiseDbmAvg) / g_samples);
 }

 NS_LOG_COMPONENT_DEFINE ("Table - Iterating distance");

 int main (int argc, char *argv[])
 {

/*
 * ------------------------------------
 * (0) - Setting Initial Configuration
 * ------------------------------------
 */

   bool udp = true;
   double simulationTime = 10; //seconds
   std::string wifiType = "ns3::SpectrumWifiPhy";                                // or ns3::SpectrumWifiPhy
   std::string propagationLossType = "ns3::AirToGroundPropagationLossModel"; // or ns3::FriisPropagationLossModel
   std::string errorModelType = "ns3::NistErrorRateModel";                   // or ns3::NistErrorRateModel
   double frequency = 2.4; //either 2.4 or 5.0 GHz
   double txPower = 1;  // dBm
   bool enablePcap = false;
   bool isLoS = true;   //AtG propagation model: LoS or NLoS
   const uint32_t tcpPacketSize = 1448;

   uint32_t rtsThreshold = 65535;

   int channelWidth = 20;           // 20 MHz, 40 MHz
   bool guardinterval = true;      // short guard interval (sgi)


   CommandLine cmd;
   cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
   cmd.AddValue ("udp", "UDP if set to 1, TCP otherwise", udp);
   cmd.AddValue ("wifiType", "select ns3::SpectrumWifiPhy or ns3::YansWifiPhy", wifiType);
   cmd.AddValue ("errorModelType", "select ns3::NistErrorRateModel or ns3::YansErrorRateModel", errorModelType);
   cmd.AddValue ("propagationLossType", "select ns3::FriisPropagationLossModel or ns3::AirToGroundPropagationLossModel", propagationLossType);
   cmd.AddValue ("frequency", "set the frequency band either 2.4 or 5 GHz", frequency);
   cmd.AddValue ("channelWidth", "set the maximum channel width either 20 or 40 MHz", channelWidth);
   cmd.AddValue ("guardinterval", "Short Guard Interval Supported", guardinterval);
   cmd.AddValue ("enablePcap", "enable pcap output", enablePcap);
   cmd.Parse (argc,argv);

   std::vector<Vector> dronePos_XYZ;
   std::vector<double> x_coor = {5.00, 3.54, 0.00, -3.54, -5.00, -3.54, 0.00, 3.54, 5.00, 3.54, 0.00, -3.54, -5.00, -3.54, 0.00, 3.54, 5.00, 3.54, 0.00, -3.54};
   std::vector<double> y_coor = {0.00, 3.54, 5.00, 3.54, 0.00, -3.54, -5.00, -3.54, 0.00, 3.54, 5.00, 3.54, 0.00, -3.54, -5.00, -3.54, 0.00, 3.54, 5.00, 3.54};
   std::vector<double> z_coor = {0.00, 0.75, 1.50, 2.25, 3.00, 3.75, 4.50, 5.25, 6.00, 6.75, 7.50, 8.25, 9.00, 9.75, 10.50, 11.25, 12.00, 12.75, 13.50, 14.25};

   for (uint16_t l = 0; l < x_coor.size(); l++)
   {
	   Vector pos = Vector(5*x_coor[l], 5*y_coor[l], 5*z_coor[l]);
	   dronePos_XYZ.push_back(pos);
	   //std::cout << "(" << dronePos_XYZ[l].x << "," << dronePos_XYZ[l].y << "," << dronePos_XYZ[l].z << ") m" << std::endl;
   }

   std::cout << "* ----------------------------------- *" << std::endl;
   std::cout << "* PHASE 1: Table - Iterating distance * " << std::endl;
   std::cout << "* ----------------------------------- *" << std::endl;
   std::cout << "\n Network topology:" << std::endl;
   std::cout << "\n  Wi-Fi 192.168.1.0" << std::endl;
   std::cout << "\n  Drone                  AP" << std::endl;
   std::cout << "    *  <-- distance -->  *" << std::endl;
   std::cout << "    |                    |" << std::endl;
   std::cout << "    n1                   n0" << std::endl;

   /* Cout -> Initial Parameters Configuration */
   std::cout << "_______________________________________________________________________________________________________________________" << std::endl;
   std::cout << "\n[wifiType: " << wifiType << "]  [Initial Drone position: " << "(" << dronePos_XYZ[0].x << "," << dronePos_XYZ[0].y << "," << dronePos_XYZ[0].z << ") m]  [Frequency: " << frequency << " GHz]  [TxPower: " << txPower << " dBm (" << std::setprecision (2) << std::fixed << std::pow (10.0,txPower / 10.0) << " mW)]" << std::endl;
   std::cout << "\n[Maximum Channel width: " << std::setprecision (0) << std::fixed << channelWidth << " MHz]  [Number of spatial streams: 1]  [MobilityModel: ns3::ConstantPositionMobilityModel]" << std::endl;
   std::cout << "\n[propagationLossType: " << propagationLossType << "]  [errorModelType: " << errorModelType << "]  [SGI: " << std::boolalpha << guardinterval << "]" << std::endl;
   std::cout << "_______________________________________________________________________________________________________________________" << std::endl;

   std::cout << std::setw (9) << "\n Iteration |" <<
     //std::setw (6) << "MCS |" <<
     std::setw (13) << " Thput (Mb/s) |" <<
     std::setw (10) << " Rx Packets |" <<
     std::setw (12) << " Signal (dBm) |" <<
     std::setw (12) << " Noise (dBm) |" <<
     std::setw (9) << " SNR (dB) |" <<
	 std::setw (21) << " position [x,y,z] (m) |" <<
     std::endl;

   std::cout << "---------------------------------------------------------------------------------------------------------" << std::endl;
	   for (uint16_t j = 0; j < dronePos_XYZ.size(); j++)
	 	  {

       uint32_t payloadSize;
       if (udp)
         {
           payloadSize = 972; // 1000 bytes IPv4
         }
       else
         {
           payloadSize = 1448; // 1500 bytes IPv6
           Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (payloadSize));
         }

       /*
        * -------------------------
        * (1) - Create two nodes
        * -------------------------
        */

       NodeContainer wifiStaNode;
       wifiStaNode.Create (1);
       NodeContainer wifiApNode;
       wifiApNode.Create (1);

       /*
        * -------------------------
        * (2) - Channel: Phy & MAC
        * -------------------------
        */

       /* Set PHY */
       YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
       SpectrumWifiPhyHelper spectrumPhy = SpectrumWifiPhyHelper::Default ();
       if (wifiType == "ns3::YansWifiPhy")
         {
           YansWifiChannelHelper channel;
           if (propagationLossType == "ns3::FriisPropagationLossModel")
           {
        	   channel.AddPropagationLoss ("ns3::FriisPropagationLossModel",
        	                                          "Frequency", DoubleValue (frequency*1e9));
           } else if(propagationLossType =="ns3::AirToGroundPropagationLossModel")
           {
        	   channel.AddPropagationLoss ("ns3::AirToGroundPropagationLossModel",
        	           	                                          "Frequency", DoubleValue (frequency*1e9), "LoS",BooleanValue(isLoS));
           } else{
               std::cout << "Wrong propagation loss model!" << std::endl;
               NS_FATAL_ERROR ("Unsupported Propagation Loss type " << propagationLossType);
               return 0;
           }
           channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");

           phy.SetChannel (channel.Create ());
           phy.SetErrorRateModel (errorModelType);
           phy.Set ("TxPowerStart", DoubleValue (txPower));
           phy.Set ("TxPowerEnd", DoubleValue (txPower));
           //phy.Set ("Frequency", UintegerValue (5180));
         }
       else if (wifiType == "ns3::SpectrumWifiPhy")
         {
           Ptr<MultiModelSpectrumChannel> spectrumChannel
             = CreateObject<MultiModelSpectrumChannel> ();
           Ptr<AirToGroundPropagationLossModel> lossModel
             = CreateObject<AirToGroundPropagationLossModel> ();
           lossModel->SetFrequency (5.180e9);
           spectrumChannel->AddPropagationLossModel (lossModel);

           Ptr<ConstantSpeedPropagationDelayModel> delayModel
             = CreateObject<ConstantSpeedPropagationDelayModel> ();
           spectrumChannel->SetPropagationDelayModel (delayModel);

           spectrumPhy.SetChannel (spectrumChannel);
           //spectrumPhy.SetChannelNumber (1);
           //spectrumPhy.SetAntenna (IsotropicAntennaModel);
           spectrumPhy.SetErrorRateModel (errorModelType);
           spectrumPhy.Set ("Frequency", UintegerValue (5180));
           spectrumPhy.Set ("Antennas", UintegerValue (1));
           phy.Set ("MaxSupportedTxSpatialStreams", UintegerValue (1));
           phy.Set ("MaxSupportedRxSpatialStreams", UintegerValue (1));
           spectrumPhy.Set ("TxPowerStart", DoubleValue (1)); // dBm  (1.26 mW)
           spectrumPhy.Set ("TxPowerEnd", DoubleValue (1));
         }
       else
         {
           NS_FATAL_ERROR ("Unsupported WiFi type " << wifiType);
         }


       /* Set MAC */
       WifiHelper wifi;
       if (frequency == 5.0)
         {
           wifi.SetStandard (WIFI_PHY_STANDARD_80211n_5GHZ);
         }
       else if (frequency == 2.4)
         {
			wifi.SetStandard (WIFI_PHY_STANDARD_80211n_2_4GHZ);
          }
        else
          {
            std::cout << "Wrong frequency value!" << std::endl;
            NS_FATAL_ERROR ("Unsupported Frequency band: it should be either 2.4 or 5.0 GHz, but is " << frequency);
            return 0;
          }
       WifiMacHelper mac;

       Ssid ssid = Ssid ("ns380211n");

       wifi.SetRemoteStationManager ("ns3::MinstrelHtWifiManager", "RtsCtsThreshold", UintegerValue (rtsThreshold));

       NetDeviceContainer staDevice;
       NetDeviceContainer apDevice;

       /* Install the devices to the nodes */
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

		// Setup Channel width & Guard interval (sgi)
		Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (channelWidth));
		Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported", BooleanValue (guardinterval));


    //set antenna at the drone
       Ptr<IsotropicAntennaModel> droneAntennas = CreateObject<IsotropicAntennaModel>();
       //droneAntennas->Set("Gain", DoubleValue(3));
       Ptr<WifiNetDevice> Dronedevice = staDevice.Get (0)->GetObject<WifiNetDevice>();
       Ptr<SpectrumWifiPhy> droneSpectrum = Dronedevice->GetPhy()->GetObject<SpectrumWifiPhy>();
       droneSpectrum->SetAntenna(droneAntennas);

       //set antenna at the AP
       Ptr<IsotropicAntennaModel> apAntennas = CreateObject<IsotropicAntennaModel>();
       //apAntennas->Set("Gain", DoubleValue(3));
       Ptr<WifiNetDevice> APdevice = apDevice.Get (0)->GetObject<WifiNetDevice>();
       Ptr<SpectrumWifiPhy> APSpectrum = APdevice->GetPhy()->GetObject<SpectrumWifiPhy>();
       APSpectrum->SetAntenna(apAntennas);


	  /*
	   * -------------------------
	   * (3) - Mobility
	   * -------------------------
	   */
	   //Mobility Allocation
       MobilityHelper mobility;
       Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

       positionAlloc->Add (Vector (0.0, 0.0, 0.0)); // AP position
       positionAlloc->Add (Vector (dronePos_XYZ[j].x, dronePos_XYZ[j].y, dronePos_XYZ[j].z)); // Drone position
       mobility.SetPositionAllocator (positionAlloc);

       mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

       mobility.Install (wifiApNode);
       mobility.Install (wifiStaNode);

       /*
        * --------------------------------
        * (4) - Upper Layers configuration
        * --------------------------------
        */

       /* Internet stack*/
       InternetStackHelper stack;
       stack.Install (wifiApNode);
       stack.Install (wifiStaNode);

       Ipv4AddressHelper address;
       address.SetBase ("192.168.1.0", "255.255.255.0");
       Ipv4InterfaceContainer staNodeInterface; // Drone_IPv4: 192.168.1.2
       Ipv4InterfaceContainer apNodeInterface;  // AP_IPv4: 192.168.1.2

       staNodeInterface = address.Assign (staDevice);
       apNodeInterface = address.Assign (apDevice);

       /* Setting transport & application layers */
       ApplicationContainer serverApp;
       if (udp)
         {
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
         }
       else
         {
           //TCP flow
           uint16_t port = 50000;
           Address localAddress (InetSocketAddress (Ipv4Address::GetAny (), port));
           PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", localAddress);
           serverApp = packetSinkHelper.Install (wifiStaNode.Get (0));
           serverApp.Start (Seconds (0.0));
           serverApp.Stop (Seconds (simulationTime + 1));

           OnOffHelper onoff ("ns3::TcpSocketFactory", Ipv4Address::GetAny ());
           onoff.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
           onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
           onoff.SetAttribute ("PacketSize", UintegerValue (payloadSize));
           onoff.SetAttribute ("DataRate", DataRateValue (1000000000)); //bit/s
           AddressValue remoteAddress (InetSocketAddress (staNodeInterface.GetAddress (0), port));
           onoff.SetAttribute ("Remote", remoteAddress);
           ApplicationContainer clientApp = onoff.Install (wifiApNode.Get (0));
           clientApp.Start (Seconds (1.0));
           clientApp.Stop (Seconds (simulationTime + 1));
         }


       /* Monitor Sniffer Rx */
       Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/Phy/MonitorSnifferRx", MakeCallback (&MonitorSniffRx));

       if (enablePcap)
         {
    	   phy.EnablePcap ("wifi-spectrum-per-example-", apDevice.Get(0));
    	   phy.EnablePcap ("wifi-spectrum-per-example-", staDevice.Get(0));
         }
       g_signalDbmAvg = 0;
       g_noiseDbmAvg = 0;
       g_samples = 0;

       Simulator::Stop (Seconds (simulationTime + 1));
       Simulator::Run ();

       /*
        * -------------------------
        * (5) - Simulator Outputs
        * -------------------------
        */

       double throughput = 0;
       uint64_t totalPacketsThrough = 0;
       if (udp)
         {
           //UDP
           totalPacketsThrough = DynamicCast<UdpServer> (serverApp.Get (0))->GetReceived ();
           throughput = totalPacketsThrough * payloadSize * 8 / (simulationTime * 1000000.0); //Mbit/s
         }
       else
         {
           //TCP
           uint64_t totalBytesRx = DynamicCast<PacketSink> (serverApp.Get (0))->GetTotalRx ();
           totalPacketsThrough = totalBytesRx / tcpPacketSize;
           throughput = totalBytesRx * 8 / (simulationTime * 1000000.0); //Mbit/s
         }


     /* Drone Node positions: */
	Ptr<Node> const droneNode = wifiStaNode.Get (0);
    Ptr<MobilityModel> mob = droneNode->GetObject<MobilityModel> ();
    Vector pos = mob->GetPosition ();

    /* Printing the Parameters Outputs */
       std::cout << std::setw (6) << j <<
         std::setprecision (2) << std::fixed <<
         std::setw (15) << throughput <<
         std::setw (14) << totalPacketsThrough;

       if (totalPacketsThrough > 0)
         {
           std::cout << std::setw (14) << g_signalDbmAvg <<
             std::setw (15) << g_noiseDbmAvg <<
             std::setw (12) << (g_signalDbmAvg - g_noiseDbmAvg);
         }
       else
         {
           std::cout << std::setw (13) << "N/A" <<
             std::setw (14) << "N/A" <<
             std::setw (14) << "N/A";
         }
	   std::cout << std::setw (7) << "(" << pos.x << ", " <<  pos.y << ", " << pos.z << ")" <<
       std::endl;

       Simulator::Destroy ();
	 	  }

   return 0;
 }