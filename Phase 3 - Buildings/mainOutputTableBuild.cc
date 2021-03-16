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
  * Date: Nov 19, 2020
  *
  */

 #include <iomanip>
 #include "ns3/command-line.h"
 #include "ns3/config.h"
 #include "ns3/uinteger.h"
 #include "ns3/boolean.h"
 #include "ns3/pointer.h"
 #include "ns3/double.h"
 #include "ns3/string.h"
 #include "ns3/log.h"
 #include "ns3/yans-wifi-helper.h"
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
 #include "ns3/channel-condition-model.h"
 #include "ns3/three-gpp-propagation-loss-model.h"
 #include "ns3/air-to-ground-propagation-loss-model.h"
 #include <ns3/buildings-helper.h>
 #include <ns3/mobility-building-info.h>
 #include "ns3/buildings-channel-condition-model.h"
 #include "ns3/constant-position-mobility-model.h"
 #include <math.h>
 #include <vector>
 #include <fstream>
 #include <string>
 #include <iterator>

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

/*

	CSD Project - main code that collects all the simulation parameters & outputs in a single-big table.
	_____________________________________________________________________________________________________________________________

	[wifiType: ns3::SpectrumWifiPhy]  [Initial Drone position: (50,0,20) m]  [Frequency: 2.4 GHz]  [TxPower: 10 dBm (10.00 mW)]

	[Maximum Channel width: 20 MHz]  [Number of spatial streams: 1]  [MobilityModel: ns3::ConstantPositionMobilityModel]

	[propagationLossType: ns3::ThreeGppUmaPropagationLossModel]  [errorModelType: ns3::YansErrorRateModel]  [SGI: true]
	_____________________________________________________________________________________________________________________________

	 Iteration | Thput (Mb/s) | Rx Packets | Signal (dBm) | Noise (dBm) | SNR (dB) | position [x,y,z] (m) | LoS/NLoS |
	-----------------------------------------------------------------------------------------------------------------------------
		 0          58.54         75284        -67.73         -93.97       26.24     (50.00, 0.00, 20.00)      1
		 1          63.24         81333        -66.32         -93.97       27.64     (0.00, 50.00, 50.00)      0
		 2          61.58         79198        -66.32         -93.97       27.64     (-50.00, 0.00, 50.00)      0
		 3          63.71         81935        -66.32         -93.97       27.64     (0.00, -50.00, 50.00)      0
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

 std::vector<int> createArray( int min, int max, int step )
 {
     std::vector<int> array;
     for( int i = min; i <= max; i += step )
     {
         array.push_back( i );
     }
     return array;
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
   std::string wifiType = "ns3::SpectrumWifiPhy";                            // or ns3::SpectrumWifiPhy, ns3::YansWifiPhy
   std::string propagationLossType = "ns3::ThreeGppUmaPropagationLossModel"; // or ns3::FriisPropagationLossModel, ns3::AirToGroundPropagationLossModel
   std::string errorModelType = "ns3::YansErrorRateModel";                   // or ns3::NistErrorRateModel
   double frequency = 2.4; //either 2.4 or 5.0 GHz
   double txPower = 10;  // dBm
   bool enablePcap = false;
   bool isLoS = true;   //AtG propagation model: LoS or NLoS
   const uint32_t tcpPacketSize = 1448;
   int nStreams = 1;       //number of spatial streams

   uint32_t rtsThreshold = 65535;

   int channelWidth = 20;           // 20 MHz, 40 MHz
   bool guardinterval = true;      // short guard interval (sgi)

   // save results to .txt file
   std::vector<std::string> txt_throughput;
   txt_throughput.push_back(" Throughput:  \n");
   std::vector<std::string> txt_snr;
   txt_snr.push_back("\n SNR :  \n");
   std::vector<std::string> txt_3ddistance;
   txt_3ddistance.push_back("\n 3D distance:  \n");
   std::vector<std::string> txt_droneXYZ;
   txt_droneXYZ.push_back("\n Drone's position (x,y,z):  \n");

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

   // Position of the drone (first building example)
   std::cout << "Drone's Trajectory: " << std::endl;
   std::vector<int> X = createArray(-50, 50, 5);
   std::vector<int> Y = createArray(-30, 30, 5);
   int Z = 25;

   for (uint16_t m = 0; m < X.size(); m++)
   		{
	   for (uint16_t n = 0; n < Y.size(); n++)
   		   {
   			   Vector pos = Vector(X[m], Y[n], Z);
   			   dronePos_XYZ.push_back(pos);
   			   std::cout << "Position (X,Y,Z) = (" << pos.x << ", " <<  pos.y << ", " << pos.z << ")" << std::endl;
   		   }
   		}

   std::cout << "* ------------------------------------ *" << std::endl;
   std::cout << "*   MAIN: Table - Iterating distance   *" << std::endl;
   std::cout << "* ------------------------------------ *" << std::endl;
   std::cout << "\n Network topology:" << std::endl;
   std::cout << "\n  Wi-Fi 192.168.1.0" << std::endl;
   std::cout << "\n  Drone                  AP" << std::endl;
   std::cout << "    *  <-- distance -->  *" << std::endl;
   std::cout << "    |                    |" << std::endl;
   std::cout << "    n1                   n0" << std::endl;

   /* Cout -> Initial Parameters Configuration */
   std::cout << "_____________________________________________________________________________________________________________________________" << std::endl;
   std::cout << "\n[wifiType: " << wifiType << "]  [Initial Drone position: " << "(" << dronePos_XYZ[0].x << "," << dronePos_XYZ[0].y << "," << dronePos_XYZ[0].z << ") m]  [Frequency: " << frequency << " GHz]  [TxPower: " << txPower << " dBm (" << std::setprecision (2) << std::fixed << std::pow (10.0,txPower / 10.0) << " mW)]" << std::endl;
   std::cout << "\n[Maximum Channel width: " << std::setprecision (0) << std::fixed << channelWidth << " MHz]  [Number of spatial streams: " << nStreams << "]  [MobilityModel: ns3::ConstantPositionMobilityModel]" << std::endl;
   std::cout << "\n[propagationLossType: " << propagationLossType << "]  [errorModelType: " << errorModelType << "]  [SGI: " << std::boolalpha << guardinterval << "]" << std::endl;
   std::cout << "_____________________________________________________________________________________________________________________________" << std::endl;

   std::cout << std::setw (9) << "\n Iteration |" <<
     //std::setw (6) << "MCS |" <<
     std::setw (13) << " Thput (Mb/s) |" <<
     std::setw (10) << " Rx Packets |" <<
     std::setw (12) << " Signal (dBm) |" <<
     std::setw (12) << " Noise (dBm) |" <<
     std::setw (9) << " SNR (dB) |" <<
	 std::setw (22) << " position [x,y,z] (m) |" <<
	 std::setw (11) << " LoS/NLoS |" <<
	 //std::setw (11) << " RxPower |" <<
     std::endl;

   std::cout << "-----------------------------------------------------------------------------------------------------------------------------" << std::endl;
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
       YansWifiPhyHelper phy; //= YansWifiPhyHelper::Default ();
       SpectrumWifiPhyHelper spectrumPhy; // = SpectrumWifiPhyHelper::Default ();
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
           } else if (propagationLossType =="ns3::ThreeGppUmaPropagationLossModel")
           {
        	   // Use a deterministic channel condition model (Building)
        	   Ptr<ChannelConditionModel> buildCondModel = CreateObject<BuildingsChannelConditionModel> ();

        	   // Create the propagation loss model
        	   channel.AddPropagationLoss("ns3::ThreeGppUmaPropagationLossModel", "Frequency", DoubleValue (frequency*1e9),
        			   "ShadowingEnabled",BooleanValue(false),
					   "ChannelConditionModel", PointerValue(buildCondModel));

           } else{
               std::cout << "Wrong propagation loss model!" << std::endl;
               NS_FATAL_ERROR ("Unsupported Propagation Loss type " << propagationLossType);
               return 0;
           }
           channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");

           phy.SetChannel (channel.Create ());
           phy.SetErrorRateModel (errorModelType);


           //set MIMO 2T2R with Max Nss = 2
		   phy.Set("Antennas", UintegerValue (nStreams));
		   phy.Set ("MaxSupportedTxSpatialStreams", UintegerValue (nStreams));
		   phy.Set ("MaxSupportedRxSpatialStreams", UintegerValue (nStreams));
           phy.Set ("TxPowerStart", DoubleValue (txPower));
           phy.Set ("TxPowerEnd", DoubleValue (txPower));
           phy.Set ("Frequency", UintegerValue (frequency*1e3));
         }
       else if (wifiType == "ns3::SpectrumWifiPhy")
         {
           Ptr<MultiModelSpectrumChannel> spectrumChannel
             = CreateObject<MultiModelSpectrumChannel> ();
           /*Ptr<FriisPropagationLossModel> lossModel
             = CreateObject<FriisPropagationLossModel> ();
           lossModel->SetFrequency (frequency*1e9);
           */

    	   // Use a deterministic channel condition model (Building)
    	   Ptr<ChannelConditionModel> buildCondModel = CreateObject<BuildingsChannelConditionModel> ();

		   // Create the propagation loss model
		   Ptr<ThreeGppUmaPropagationLossModel> lossModel = CreateObject<ThreeGppUmaPropagationLossModel> ();
		   // Set the Attributes to PropagationLossModel - 3GPP Uma
		   lossModel->SetAttribute ("ShadowingEnabled", BooleanValue (false)); // disable the shadow fading
		   lossModel->SetAttribute ("Frequency", DoubleValue (frequency*1e9));
		   lossModel->SetChannelConditionModel (buildCondModel);

           spectrumChannel->AddPropagationLossModel (lossModel);


           Ptr<ConstantSpeedPropagationDelayModel> delayModel
             = CreateObject<ConstantSpeedPropagationDelayModel> ();
           spectrumChannel->SetPropagationDelayModel (delayModel);

           spectrumPhy.SetChannel (spectrumChannel);
           spectrumPhy.SetErrorRateModel (errorModelType);
           spectrumPhy.Set ("Frequency", UintegerValue (frequency*1e3));
           //spectrumPhy.Set ("Frequency", UintegerValue (5180));
           spectrumPhy.Set ("TxPowerStart", DoubleValue (txPower));
           spectrumPhy.Set ("TxPowerEnd", DoubleValue (txPower));
         }
       else
         {
           NS_FATAL_ERROR ("Unsupported WiFi type " << wifiType);
         }

       /* Set MAC */
       WifiHelper wifi;
       if (frequency == 5.0)
         {
           wifi.SetStandard (WIFI_STANDARD_80211n_5GHZ);
         }
       else if (frequency == 2.4)
         {
			wifi.SetStandard (WIFI_STANDARD_80211n_2_4GHZ);
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

       Ptr<Node> const ApNode = wifiApNode.Get (0);
       Ptr<MobilityModel> a = ApNode->GetObject<MobilityModel> ();
   	   Ptr<Node> const droneNode = wifiStaNode.Get (0);
       Ptr<MobilityModel> b = droneNode->GetObject<MobilityModel> ();

	   /*
		* -------------------------
		* (4) - Setup buildings
		* -------------------------
		*/
		double x_min = 20.0, x_max = 30.0;
		double y_min = -10.0, y_max = 10.0;
		double z_min = 0.0, z_max = 15.0;
		Ptr<Building> building = CreateObject <Building> ();
		building->SetBoundaries (Box (x_min, x_max, y_min, y_max, z_min, z_max));
		Ptr<Building> building2 = CreateObject <Building> ();
		building2->SetBoundaries (Box (-x_max, -x_min, y_min, y_max, z_min, z_max));

		BuildingsHelper::Install (wifiStaNode);
		BuildingsHelper::Install (wifiApNode);

	    Ptr<MobilityBuildingInfo> buildingInfoA = a->GetObject<MobilityBuildingInfo> ();
	    buildingInfoA->MakeConsistent (a);
	    Ptr<MobilityBuildingInfo> buildingInfoB = b->GetObject<MobilityBuildingInfo> ();
	    buildingInfoB->MakeConsistent (b);

       /*
        * --------------------------------
        * (5) - Upper Layers configuration
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
        * (6) - Simulator Outputs
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
    Ptr<MobilityModel> mob = droneNode->GetObject<MobilityModel> ();
    Vector pos = mob->GetPosition ();

    /* Channel condition model between AP and Drone */
	Ptr<ChannelConditionModel> buildCondModel = CreateObject<BuildingsChannelConditionModel> ();
    Ptr<ChannelCondition> cond = buildCondModel->GetChannelCondition (a, b);

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

	   std::cout << std::setw (6) << "(" << pos.x << ", " <<  pos.y << ", " << pos.z << ")" <<
					std::setw (7) << cond->GetLosCondition () << std::endl;

       Simulator::Destroy ();

        // write output to .txt file
      std::ostringstream txt_throughput_stream;
      std::ostringstream txt_snr_stream;
      std::ostringstream txt_3ddistance_stream;
      std::ostringstream txt_droneXYZ_stream;

      txt_throughput_stream << throughput;
      txt_snr_stream << (g_signalDbmAvg - g_noiseDbmAvg);
      txt_3ddistance_stream << sqrt(pow(pos.x,2)+pow(pos.y,2)+pow(pos.z,2));
      txt_droneXYZ_stream << "(" << pos.x << ", " <<  pos.y << ", " << pos.z << ")";

      std::string txt_throughput_str = txt_throughput_stream.str();
      std::string txt_snr_str = txt_snr_stream.str();
      std::string txt_3ddistance_str = txt_3ddistance_stream.str();
      std::string txt_droneXYZ_str = txt_droneXYZ_stream.str();
      std::string tmp1 = ", ";

      txt_throughput_str.append(tmp1);
      txt_snr_str.append(tmp1);
      txt_3ddistance_str.append(tmp1);
      txt_droneXYZ_str.append(tmp1);

      txt_throughput.push_back(txt_throughput_str);
      txt_snr.push_back(txt_snr_str);
      txt_3ddistance.push_back(txt_3ddistance_str);
      txt_droneXYZ.push_back(txt_droneXYZ_str);


	 	  }

    std::ofstream output_file("./scratch/result_output.txt");
    std::ostream_iterator<std::string> output_iterator(output_file); //output_file, "\n"
    std::copy(txt_throughput.begin(), txt_throughput.end(), output_iterator);
    std::copy(txt_snr.begin(), txt_snr.end(), output_iterator);
    std::copy(txt_3ddistance.begin(), txt_3ddistance.end(), output_iterator);
    std::copy(txt_droneXYZ.begin(), txt_droneXYZ.end(), output_iterator);

   return 0;
 }


