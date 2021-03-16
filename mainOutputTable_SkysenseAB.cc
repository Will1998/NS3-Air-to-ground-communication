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
  * Last Date edit: Dec 13, 2020
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
 #include "ns3/wifi-net-device.h"
 #include "ns3/wifi-mac.h"
 #include "ns3/yans-error-rate-model.h"

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

 NS_LOG_COMPONENT_DEFINE ("Table - SkysenseAB Scenario");

 int main (int argc, char *argv[])
 {

/*
 * ------------------------------------
 * (0) - Setting Initial Configuration
 * ------------------------------------
 */

   bool udp = true;
   double simulationTime = 1; //seconds
   std::string wifiType = "ns3::YansWifiPhy";                                // or ns3::SpectrumWifiPhy, ns3::YansWifiPhy
   std::string propagationLossType = "ns3::ThreeGppUmaPropagationLossModel"; // or ns3::FriisPropagationLossModel, ns3::AirToGroundPropagationLossModel
   std::string errorModelType = "ns3::YansErrorRateModel";                   // or ns3::NistErrorRateModel
   double frequency = 2.4; //either 2.4 or 5.0 GHz
   double txPower = 20;  // dBm
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
   std::vector<std::string> txt_droneX;
   txt_droneX.push_back("\n Drone's position x:  \n");
   std::vector<std::string> txt_droneY;
   txt_droneY.push_back("\n Drone's position y:  \n");
   std::vector<std::string> txt_droneZ;
   txt_droneZ.push_back("\n Drone's position z:  \n");

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

   std::vector<int> X = createArray(-120, 170, 10);
   std::vector<int> Y = createArray(-200, 250, 10);
   int Z = 16;

   /*
   for (uint16_t m = 0; m < X.size(); m++)
   		{
	   for (uint16_t n = 0; n < Y.size(); n++)
   		   {
   			   Vector pos = Vector(X[m], Y[n], Z);
   			   dronePos_XYZ.push_back(pos);
   			   std::cout << "Position (X,Y,Z) = (" << pos.x << ", " <<  pos.y << ", " << pos.z << ")" << std::endl;
   		   }
   		}
   */

   for (uint16_t m = 0; m < X.size(); m++)
        {
               Vector pos = Vector(X[m], Y[m], Z);
   			   dronePos_XYZ.push_back(pos);
   			   std::cout << "Position (X,Y,Z) = (" << pos.x << ", " <<  pos.y << ", " << pos.z << ")" << std::endl; 
        }       

   /*
   int X = 0;
   int Y = 0;
   std::vector<int> Z = createArray(20, 120, 2);
   for (uint16_t m = 0; m < Z.size(); m++)
   	{
	   Vector pos = Vector(X, Y, Z[m]);
	   dronePos_XYZ.push_back(pos);
	   std::cout << "Position (X,Y,Z) = (" << pos.x << ", " <<  pos.y << ", " << pos.z << ")" << std::endl;
   	}*/


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

		//Disable A-MPDU & set Drop-out packet rate
		Ptr<YansErrorRateModel> error = CreateObject<YansErrorRateModel> ();
		error->SetDropRate(0.2);

		Ptr<NetDevice> dev = wifiStaNode.Get (0)->GetDevice (0);
	    Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice> (dev);
	    wifi_dev->GetPhy()-> SetErrorRateModel(error);
	    wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (0));

	    dev = wifiApNode.Get (0)->GetDevice (0);
	    wifi_dev = DynamicCast<WifiNetDevice> (dev);
	    wifi_dev->GetPhy()-> SetErrorRateModel(error);
	    wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (0));

	  /*
	   * -------------------------
	   * (3) - Mobility
	   * -------------------------
	   */
	   //Mobility Allocation
       MobilityHelper mobility;

       Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

       positionAlloc->Add (Vector (0.0, 0.0, 11)); // AP position
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

		// (1) Skysense AB
		Ptr<Building> building = CreateObject <Building> ();
		building->SetBoundaries (Box (-18, 18, -37, 37, 0, 9));
		// (2) KSAK Motorflygförbund
		Ptr<Building> building2 = CreateObject <Building> ();
		building2->SetBoundaries (Box (50, 75, 5, 49, 0, 9));
		// (3) Klintberg & Way AB
		Ptr<Building> building3 = CreateObject <Building> ();
		building3->SetBoundaries (Box (-32, -16, -120, -80, 0, 14));
		// (4-5) Two small buildings
		Ptr<Building> building4 = CreateObject <Building> ();
		building4->SetBoundaries (Box (87, 93, -100.5, -93, 0, 4));
		Ptr<Building> building5 = CreateObject <Building> ();
		building5->SetBoundaries (Box (-38, -32.5, 24, 34, 0, 3));

		// (6) La Leif Arvidsson AB (building block #1)
		Ptr<Building> building60 = CreateObject <Building> ();
		building60->SetBoundaries (Box (96, 159, 20, 43, 0, 12));
		Ptr<Building> building61 = CreateObject <Building> ();
		building61->SetBoundaries (Box (144, 159, -33, 20, 0, 12));
		Ptr<Building> building62 = CreateObject <Building> ();
		building62->SetBoundaries (Box (110, 165, -51, -33, 0, 12));
		Ptr<Building> building63 = CreateObject <Building> ();
		building63->SetBoundaries (Box (159, 165, -51, 1, 0, 12));

		// (7) ? (building block #2)
		Ptr<Building> building70 = CreateObject <Building> ();
		building70->SetBoundaries (Box (126, 154, -139, -90, 0, 8));
		Ptr<Building> building71 = CreateObject <Building> ();
		building71->SetBoundaries (Box (154, 163, -123, -83, 0, 8));

		// (8) Bavaria Sverige Bil AB (building block #3)
		Ptr<Building> building80 = CreateObject <Building> ();
		building80->SetBoundaries (Box (-16, -9, -130, -100, 0, 9));
		Ptr<Building> building81 = CreateObject <Building> ();
		building81->SetBoundaries (Box (-9, 57, -188, -100, 0, 9));
		Ptr<Building> building82 = CreateObject <Building> ();
		building82->SetBoundaries (Box (57, 59, -116, -100, 0, 9));
		Ptr<Building> building83 = CreateObject <Building> ();
		building83->SetBoundaries (Box (57, 72, -140, -116, 0, 9));

		// (9) Axlås Solidlås AB Alarm & Access Control (building block #4)
		Ptr<Building> building90 = CreateObject <Building> ();
		building90->SetBoundaries (Box (-69, -59, -162, -80, 0, 15));
		Ptr<Building> building91 = CreateObject <Building> ();
		building91->SetBoundaries (Box (-91, -59, -162, -109, 0, 15));

		// (10) Huawei Technologies Sweden AB (building block #5)
		Ptr<Building> building10 = CreateObject <Building> ();
		building10->SetBoundaries (Box (-69, -44, -31, 35, 0, 17));
		Ptr<Building> building101 = CreateObject <Building> ();
		building101->SetBoundaries (Box (-111, -93, -31, 35, 0, 17));
		Ptr<Building> building102 = CreateObject <Building> ();
		building102->SetBoundaries (Box (-93, -69, -31, -9, 0, 17));
		Ptr<Building> building103 = CreateObject <Building> ();
		building103->SetBoundaries (Box (-93, -69, 12, 35, 0, 17));

		// (11) Ericsson, building 27 entrance (building block #6)
		Ptr<Building> building11 = CreateObject <Building> ();
		building11->SetBoundaries (Box (4, 62, 171, 177, 0, 26));
		Ptr<Building> building111 = CreateObject <Building> ();
		building111->SetBoundaries (Box (-13, 62, 177, 193, 0, 26));
		Ptr<Building> building112 = CreateObject <Building> ();
		building112->SetBoundaries (Box (-1, 62, 193, 218, 0, 26));
		Ptr<Building> building113 = CreateObject <Building> ();
		building113->SetBoundaries (Box (-13, 62, 218, 237, 0, 26));
		Ptr<Building> building114 = CreateObject <Building> ();
		building114->SetBoundaries (Box (4, 62, 237, 243, 0, 26));

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
      std::ostringstream txt_droneX_stream;
      std::ostringstream txt_droneY_stream;
      std::ostringstream txt_droneZ_stream;

      txt_throughput_stream << throughput;
      txt_snr_stream << (g_signalDbmAvg - g_noiseDbmAvg);
      txt_3ddistance_stream << sqrt(pow(pos.x,2)+pow(pos.y,2)+pow(pos.z,2));
      txt_droneXYZ_stream << "(" << pos.x << ", " <<  pos.y << ", " << pos.z << ")";
      txt_droneX_stream << pos.x;
      txt_droneY_stream << pos.y;
      txt_droneZ_stream << pos.z;

      std::string txt_throughput_str = txt_throughput_stream.str();
      std::string txt_snr_str = txt_snr_stream.str();
      std::string txt_3ddistance_str = txt_3ddistance_stream.str();
      std::string txt_droneXYZ_str = txt_droneXYZ_stream.str();
      std::string txt_droneX_str = txt_droneX_stream.str();
      std::string txt_droneY_str = txt_droneY_stream.str();
      std::string txt_droneZ_str = txt_droneZ_stream.str();
      std::string tmp1 = ", ";

      txt_throughput_str.append(tmp1);
      txt_snr_str.append(tmp1);
      txt_3ddistance_str.append(tmp1);
      txt_droneXYZ_str.append(tmp1);
      txt_droneX_str.append(tmp1);
      txt_droneY_str.append(tmp1);
      txt_droneZ_str.append(tmp1);

      txt_throughput.push_back(txt_throughput_str);
      txt_snr.push_back(txt_snr_str);
      txt_3ddistance.push_back(txt_3ddistance_str);
      txt_droneXYZ.push_back(txt_droneXYZ_str);
      txt_droneX.push_back(txt_droneX_str);
      txt_droneY.push_back(txt_droneY_str);
      txt_droneZ.push_back(txt_droneZ_str);


	 	  }

    /*
    std::ofstream output_file("./scratch/result_output_Skysense.txt");
    std::ostream_iterator<std::string> output_iterator(output_file); //output_file, "\n"
    std::copy(txt_throughput.begin(), txt_throughput.end(), output_iterator);
    std::copy(txt_snr.begin(), txt_snr.end(), output_iterator);
    std::copy(txt_3ddistance.begin(), txt_3ddistance.end(), output_iterator);
    std::copy(txt_droneXYZ.begin(), txt_droneXYZ.end(), output_iterator);
    */
    std::string combo_str = "[";

 
	for(uint16_t i = 0; i < X.size()-1; i++)
	{
		combo_str = combo_str + "\n    {\n";
		combo_str = combo_str + "            \"posX\": "+ txt_droneX[i+1] +" \n";
		combo_str = combo_str + "            \"posY\": "+ txt_droneY[i+1] +" \n";
		combo_str = combo_str + "            \"posZ\": "+ txt_droneZ[i+1] +" \n";
        combo_str = combo_str + "            \"SNR\": "+ txt_snr[i+1] +" \n";
		combo_str = combo_str + "            \"Throughput\": "+ txt_throughput[i+1] +"\n    },";
	}

    combo_str = combo_str + "\n    {\n";
	combo_str = combo_str + "            \"posX\": "+ txt_droneX[X.size()] +" \n";
	combo_str = combo_str + "            \"posY\": "+ txt_droneY[X.size()] +" \n";
	combo_str = combo_str + "            \"posZ\": "+ txt_droneZ[X.size()] +" \n";
    combo_str = combo_str + "            \"SNR\": "+ txt_snr[X.size()] +" \n";
	combo_str = combo_str + "            \"Throughput\": "+ txt_throughput[X.size()] +"\n    }";
    combo_str = combo_str + "\n]";

    std::cout << combo_str << std::endl;

    std::ofstream output_file("./scratch/SkysenseAB.json");
	output_file << combo_str;
	output_file.close();

   return 0;
 }