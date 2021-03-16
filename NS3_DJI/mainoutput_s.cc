
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
 #include "ns3/internet-stack-helper.h"
 #include "ns3/ipv4-address-helper.h"
 #include "ns3/udp-client-server-helper.h"
 #include "ns3/packet-sink-helper.h"
 #include "ns3/on-off-helper.h"
 #include "ns3/ipv4-global-routing-helper.h"
 #include "ns3/packet-sink.h"
 #include "ns3/yans-wifi-channel.h"
 #include "ns3/multi-model-spectrum-channel.h"
 #include "ns3/wifi-net-device.h"
#include "ns3/wifi-mac.h"
#include "ns3/double.h"
#include "ns3/qos-txop.h"
#include "ns3/pointer.h"
#include "ns3/wifi-psdu.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/stats-module.h"
#include "ns3/nstime.h"
#include "ns3/olsr-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
 //#include "ns3/propagation-loss-model.h"
  #include "ns3/yans-error-rate-model.h"
 #include <math.h>
 #include <vector>
 #include <fstream>
 #include <string>
 #include <iterator>

 using namespace ns3;

 // Global variables for use in callbacks.
 double g_signalDbmAvg;
 double g_noiseDbmAvg;
 uint32_t g_samples;

 double g_size;
 
 void MonitorSniffRx (Ptr<const Packet> packet,  
                uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise)
 {
   g_samples++;
   g_signalDbmAvg += ((signalNoise.signal - g_signalDbmAvg) / g_samples);
   g_noiseDbmAvg += ((signalNoise.noise - g_noiseDbmAvg) / g_samples);

   g_size += packet->GetSize();
  // std::cout<< " sniff count " << g_samples << " size " << g_size << std::endl;

 }

int rxPacket = 0;
double rxB = 0;
void MonitorMacRx (Ptr<const Packet> p)
{
   rxB += p->GetSize ();
   rxPacket ++;
  // std::cout<< " mac count " << rxPacket << " size " << p->GetSize() << std::endl;
}

int rxDropPacket = 0;
double rxDropB = 0;
void MonitorMacRxDrop (Ptr<const Packet> p)
{
   rxDropB += p->GetSize ();
   rxDropPacket ++;
}

int phyRxDrop;
double phyRxDropB;
void MonitorPhyRxDrop (Ptr<const Packet> p, WifiPhyRxfailureReason reason){
  phyRxDrop ++;
  phyRxDropB += p -> GetSize();
}

// RxOk: A packet has been received successfully.
// RxError: A packet has been received unsuccessfully.
int rxOk;
double rxOkB;
void MonitorPhyStateRxOK (Ptr<ns3::Packet const> p,double snr,WifiMode mode,WifiPreamble preamble){
  rxOk ++;
  rxOkB += p -> GetSize();
  //std::cout<< " phy count " << rxOk << " size " << p->GetSize() << std::endl;
}

int rxError;
double rxErrorB;
void MonitorPhyStateRxError (Ptr<ns3::Packet const> p,double snr){
  rxError ++;
  rxErrorB += p -> GetSize();
}



 NS_LOG_COMPONENT_DEFINE ("Table - Iterating distance");

 int main (int argc, char *argv[])
 {

CommandLine cmd;
cmd.Parse (argc, argv);
  NS_LOG_COMPONENT_DEFINE ("main");


/////////////////////////////////////////////////////////////////////////////////////////////////////
   bool verbose =0;
   double simulationTime =1; //seconds
   uint32_t pktnum =10000;   // 4294967295

   std::string wifiType = "ns3::YansWifiPhy";                                // or ns3::SpectrumWifiPhy   "ns3::YansWifiPhy"
   std::string propagationLossType = "ns3::FriisPropagationLossModel"; // or ns3::FriisPropagationLossModel
   std::string errorModelType = "ns3::YansErrorRateModel";                   // or ns3::NistErrorRateModel
   double frequency = 2.4; //either 2.4 or 5.0 GHz
   double txPower = 20;  // dBm


   bool enablePcap = 1;
   
   //bool isLoS = true;   //AtG propagation model: LoS or NLoS

   int channelWidth =10;           // 20 MHz, 40 MHz
   bool guardinterval = true;      // short guard interval (sgi)

   std::vector<Vector> dronePos_XYZ;
   std::vector<double> x_coor = {5.00, 3.54, 0.00, -3.54, -5.00, -3.54, 0.00, 3.54, 5.00, 3.54, 0.00, -3.54, -5.00, -3.54, 0.00, 3.54, 5.00, 3.54, 0.00, -3.54};
   std::vector<double> y_coor = {0.00, 3.54, 5.00, 3.54, 0.00, -3.54, -5.00, -3.54, 0.00, 3.54, 5.00, 3.54, 0.00, -3.54, -5.00, -3.54, 0.00, 3.54, 5.00, 3.54};
   std::vector<double> z_coor = {0.00, 0.75, 1.50, 2.25, 3.00, 3.75, 4.50, 5.25, 6.00, 6.75, 7.50, 8.25, 9.00, 9.75, 10.50, 11.25, 12.00, 12.75, 13.50, 14.25};

   int interation_distance = 1; // x_coor.size();


   for (uint16_t l = 0; l < interation_distance; l++)   
   {
	   Vector pos = Vector(0,0*450,0*400);  //5*z_coor[l]
	   dronePos_XYZ.push_back(pos);
	   //std::cout << "(" << dronePos_XYZ[l].x << "," << dronePos_XYZ[l].y << "," << dronePos_XYZ[l].z << ") m" << std::endl;
   }

/////////////////////////////////////////////////////////////////////////////////////////////////////
   
  // uint32_t rtsThreshold = 65535;
   bool udp = true;
   // save results to .txt file
   std::vector<std::string> txt_throughput;
   txt_throughput.push_back("Throughput  \n");
   std::vector<std::string> txt_snr;
   txt_snr.push_back("\n snr   \n");
   std::vector<std::string> txt_3ddistance;
   txt_3ddistance.push_back("\n 3d distance  \n");


  if (verbose){
  //LogComponentEnable ("UdpClient", LOG_LEVEL_INFO);
 // LogComponentEnable ("UdpServer", LOG_LEVEL_ALL);
  //LogComponentEnable ("YansWifiChannel", LOG_LEVEL_ALL);
// LogComponentEnable ("WifiPhy", LOG_LEVEL_ALL);
//LogComponentEnable ("WifiMode", LOG_LEVEL_ALL);
 // LogComponentEnable ("MacLow", LOG_LEVEL_ALL);
 // LogComponentEnable("WifiMac",LOG_LEVEL_ALL);
// LogComponentEnable("RegularWifiMac",LOG_LEVEL_ALL);
// LogComponentEnable("ApWifiMac",LOG_LEVEL_ALL);
// LogComponentEnable("WifiNetDevice",LOG_LEVEL_ALL);
 //  LogComponentEnable("Txop",LOG_LEVEL_ALL);             // Txop:UpdateBackoffSlotsNow(
// LogComponentEnable("SimpleNetDevice",LOG_LEVEL_ALL);
// LogComponentEnable("MpduAggregator",LOG_LEVEL_ALL);
//  LogComponentEnable("QosTxop",LOG_LEVEL_ALL);
//  LogComponentEnable("WifiPhyHeader",LOG_LEVEL_ALL);
// LogComponentEnable("MinstrelHtWifiManager",LOG_LEVEL_ALL);
//LogComponentEnable("IdealWifiManager",LOG_LEVEL_ALL);
//LogComponentEnable("BlockAckManager",LOG_LEVEL_ALL);
LogComponentEnable("YansErrorRateModel",LOG_LEVEL_ALL);

    

  
  }


   /* Cout -> Initial Parameters Configuration */
   //std::cout << "_______________________________________________________________________________________________________________________" << std::endl;
   //std::cout << "\n[wifiType: " << wifiType << "]  [Initial Drone position: " << "(" << dronePos_XYZ[0].x << "," << dronePos_XYZ[0].y << "," << dronePos_XYZ[0].z << ") m]  [Frequency: " << frequency << " GHz]  [TxPower: " << txPower << " dBm (" << std::setprecision (2) << std::fixed << std::pow (10.0,txPower / 10.0) << " mW)]" << std::endl;
   std::cout << "\n[Maximum Channel width: " << std::setprecision (0) << std::fixed << channelWidth << "MHz  [SGI: " << std::boolalpha << guardinterval << std::endl;
   //std::cout << "\n[propagationLossType: " << propagationLossType << "]  [errorModelType: " << errorModelType << "]  [SGI: " << std::boolalpha << guardinterval << "]" << std::endl;
   //std::cout << "_______________________________________________________________________________________________________________________" << std::endl;

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
           payloadSize = 60000; // 1000 bytes IPv4
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
               std::cout << "xx" << std::endl;
        	//    channel.AddPropagationLoss ("ns3::AirToGroundPropagationLossModel",
        	//            	                                          "Frequency", DoubleValue (frequency*1e9), "LoS",BooleanValue(isLoS));
           } 
           
           channel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");

           phy.SetChannel (channel.Create ());
          // phy.SetErrorRateModel (errorModelType);
           phy.Set ("TxPowerStart", DoubleValue (txPower));
           phy.Set ("TxPowerEnd", DoubleValue (txPower));
           phy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
           //phy.Set ("Frequency", UintegerValue (5180));
         }
         


       /* Set MAC */
       WifiHelper wifi;
       if (frequency == 5.0)
         {
            wifi.SetStandard (WIFI_STANDARD_80211n_5GHZ);
         }
       else // (frequency == 2.4)
         {
			wifi.SetStandard (WIFI_STANDARD_80211n_2_4GHZ);
          }

       WifiMacHelper mac;

       Ssid ssid = Ssid ("ns380211n");

      //uint32_t rtsThreshold = 65535;
      //wifi.SetRemoteStationManager ("ns3::MinstrelHtWifiManager", "RtsCtsThreshold", UintegerValue (rtsThreshold));

      //wifi.SetRemoteStationManager ("ns3::IdealWifiManager");   // Ideal
       std::string datamode = "HtMcs7";   // ErpOfdmRate54Mbps    HtMcs7
       std::string ctrlRate = "HtMcs0";   // OfdmRate24Mbps
       wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode" ,StringValue(datamode), "ControlMode", StringValue(ctrlRate));
      std::cout<<"Data mode   "<< datamode<<std::endl;

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


           //Disable A-MPDU
  Ptr<YansErrorRateModel> error = CreateObject<YansErrorRateModel> ();
   error->SetDropRate(0.7);
   Ptr<NetDevice> dev = wifiStaNode.Get (0)->GetDevice (0);
   Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice> (dev);
   wifi_dev->GetPhy()-> SetErrorRateModel(error);
  // wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (0));

  dev = wifiApNode.Get (0)->GetDevice (0);
  wifi_dev = DynamicCast<WifiNetDevice> (dev);
   wifi_dev->GetPhy()-> SetErrorRateModel(error);
  // wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (0));

  //  double min = 0.0;
  //   double max = 10.0;
 
//     Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable> ();
//     // x->SetAttribute ("Min", DoubleValue (min));
//     // x->SetAttribute ("Max", DoubleValue (max));
//    double value = x->GetValue ();
//  std::cout<<"random "<< value *100.0 << std::endl;
//   value = x->GetValue ();
//   std::cout<<"random "<< value*100.0  << std::endl;





Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmpduSize", UintegerValue (65535));

// // Modify EDCA configuration (TXOP limit) for AC_BE
//  PointerValue ptr;
//   Ptr<QosTxop> edca;
//   wifi_dev->GetMac ()->GetAttribute ("BE_Txop", ptr);
//   edca = ptr.Get<QosTxop> ();
//   edca->SetTxopLimit (MicroSeconds (56320));


      }

Time sifs = MicroSeconds(2500);
Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/Sifs", TimeValue(sifs));

Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/Txop/MinCw", UintegerValue(1));
Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/Txop/MaxCw", UintegerValue(2));

          // Setup Channel width & Guard interval (sgi)
          Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (channelWidth));
          //phy.Set("ChannelWidth",UintegerValue( channelWidth));
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
           client.SetAttribute ("MaxPackets", UintegerValue (pktnum));
           client.SetAttribute ("Interval", TimeValue (Time ("0.01"))); //packets/s
           client.SetAttribute ("PacketSize", UintegerValue (payloadSize));
           ApplicationContainer clientApp = client.Install (wifiApNode.Get (0));
           clientApp.Start (Seconds (1.0));
           clientApp.Stop (Seconds (simulationTime + 1));
         }

       /* Monitor Sniffer Rx */
       Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/Phy/MonitorSnifferRx", MakeCallback (&MonitorSniffRx));
      Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/Mac/MacRx",MakeCallback(&MonitorMacRx));
      Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/Mac/MacRxDrop",MakeCallback(&MonitorMacRxDrop));
       Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/Phy/PhyRxDrop",MakeCallback(&MonitorPhyRxDrop));
       Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/State/RxOk",MakeCallback(&MonitorPhyStateRxOK));
       Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/State/RxError",MakeCallback(&MonitorPhyStateRxError));

       if (enablePcap)
         {
    	   phy.EnablePcap ("test-", apDevice.Get(0));
    	   phy.EnablePcap ("test-", staDevice.Get(0));


       AsciiTraceHelper ascii;
        phy.EnableAsciiAll (ascii.CreateFileStream ("test.tr"));
        //phy.EnablePcap ("test", apDevice.Get(0));
       // Trace routing tables
       //  Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("wifi-simple-adhoc-grid.routes", std::ios::out);
       //   olsr.PrintRoutingTableAllEvery (Seconds (2), routingStream);


         }

      
      Ptr<FlowMonitor> flowMonitor;
      FlowMonitorHelper flowHelper;
      flowMonitor = flowHelper.InstallAll();



         
       g_signalDbmAvg = 0;
       g_noiseDbmAvg = 0;
       g_samples = 0;
        rxPacket = 0;
        rxB = 0;
        rxDropPacket = 0;
        rxDropB = 0;
         phyRxDrop =0;
        phyRxDropB =0;
        rxOk = 0;
        rxOkB = 0;
        rxError =0;
        rxErrorB = 0;



       Simulator::Stop (Seconds (simulationTime + 1));
       Simulator::Run ();

       /*
        * -------------------------
        * (5) - Simulator Outputs
        * -------------------------
        */
flowMonitor->SerializeToXmlFile("test.xml", true, true);
       double throughput = 0;
       uint64_t totalPacketsThrough = 0;
       if (udp)
         {
           //UDP
           totalPacketsThrough = DynamicCast<UdpServer> (serverApp.Get (0))->GetReceived ();
           throughput = totalPacketsThrough * payloadSize * 8 / (simulationTime * 1000000.0); //Mbit/s
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

       if (totalPacketsThrough >= 0)
         {
           std::cout << std::setw (14) << g_signalDbmAvg <<std::setw (15) << g_noiseDbmAvg <<std::setw (12) << (g_signalDbmAvg - g_noiseDbmAvg);
         }
       else
         {
           std::cout << std::setw (13) << "N/A" <<
             std::setw (14) << "N/A" <<
             std::setw (14) << "N/A";
         }
	   std::cout << std::setw (7) << "(" << pos.x << ", " <<  pos.y << ", " << pos.z << ")" <<
       std::endl;

       std::cout<< "mac rx:             "<< rxPacket << " received:   " << rxB << "  "<<rxB/1000000*8/simulationTime << " Mbi/st"<<std::endl;
       //std::cout<< "mac dropped rx:     "<< rxDropPacket << " dropped:   " << rxDropB << std::endl;
       //std::cout<< "phy dropped rx:     "<< phyRxDrop << " dropped:   " << phyRxDropB << std::endl;
       std::cout<< "phy state rx ok:    "<< rxOk << " received:   " << rxOkB << "  "<<rxOkB/1000000*8/simulationTime << " Mbit/s"<<std::endl;
       std::cout<< "phy state rx ERROR: "<< rxError << " dropped:   " << rxErrorB <<"  "<< rxErrorB/1000000*8/simulationTime <<" Mbit/s"<<std::endl;


       Simulator::Destroy ();

        // write output to .txt file
      std::ostringstream txt_throughput_stream;
      std::ostringstream txt_snr_stream;
      std::ostringstream txt_3ddistance_stream;
      txt_throughput_stream << throughput;
      txt_snr_stream << (g_signalDbmAvg - g_noiseDbmAvg);
      txt_3ddistance_stream << sqrt(pow(pos.x,2)+pow(pos.y,2)+pow(pos.z,2));
      std::string txt_throughput_str = txt_throughput_stream.str();
      std::string txt_snr_str = txt_snr_stream.str();
      std::string txt_3ddistance_str = txt_3ddistance_stream.str();
      std::string tmp1 = ", ";
      txt_throughput_str.append(tmp1);
      txt_snr_str.append(tmp1);
      txt_3ddistance_str.append(tmp1);
      txt_throughput.push_back(txt_throughput_str);
      txt_snr.push_back(txt_snr_str);
      txt_3ddistance.push_back(txt_3ddistance_str);


	 	  }

    std::ofstream output_file("./scratch/result_output.txt");
    std::ostream_iterator<std::string> output_iterator(output_file); //output_file, "\n"
    std::copy(txt_throughput.begin(), txt_throughput.end(), output_iterator);
    std::copy(txt_snr.begin(), txt_snr.end(), output_iterator);
    std::copy(txt_3ddistance.begin(), txt_3ddistance.end(), output_iterator);

   return 0;
 }