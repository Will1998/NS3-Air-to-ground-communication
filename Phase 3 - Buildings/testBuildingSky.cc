/*
 * Created by: Gerard Giramé Rizzo <gerardgr@kth.se>
 * Last edit: Nov 22, 2020
 */

 #include <math.h>
 #include <vector>
 #include <fstream>
 #include <string>
 #include <iterator>
 #include <iomanip>
 #include <numeric> //std::iota
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
 #include "ns3/propagation-loss-model.h"
 #include "ns3/channel-condition-model.h"
 #include "ns3/three-gpp-propagation-loss-model.h"
 #include "ns3/air-to-ground-propagation-loss-model.h"
 #include "ns3/constant-position-mobility-model.h"
 #include "ns3/constant-velocity-mobility-model.h"
 #include <ns3/buildings-helper.h>
 #include <ns3/hybrid-buildings-propagation-loss-model.h>
 #include <ns3/mobility-building-info.h>
 #include "ns3/buildings-channel-condition-model.h"
 #include "ns3/simulator.h"
 #include "ns3/netanim-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("Building example");

std::vector<int> createArray( int min, int max, int step )
{
    std::vector<int> array;
    for( int i = min; i <= max; i += step )
    {
        array.push_back( i );
    }
    return array;
}

int main (int argc, char *argv[])
 {

	// Create the nodes for AP and Drone
	NodeContainer nodes;
	nodes.Create (2);

	// Create the mobility models
	Ptr<MobilityModel> a = CreateObject<ConstantPositionMobilityModel> ();
	nodes.Get (0)->AggregateObject (a);
	Ptr<MobilityModel> b = CreateObject<ConstantPositionMobilityModel> ();
	nodes.Get (1)->AggregateObject (b);

	// Use a deterministic channel condition model (LOS, NLOS & Building)
	Ptr<ChannelConditionModel> losCondModel = CreateObject<AlwaysLosChannelConditionModel> ();
	Ptr<ChannelConditionModel> nlosCondModel = CreateObject<NeverLosChannelConditionModel> ();
	Ptr<ChannelConditionModel> buildCondModel = CreateObject<BuildingsChannelConditionModel> ();

	// Create the propagation loss model
	Ptr<ThreeGppUmaPropagationLossModel> lossModel = CreateObject<ThreeGppUmaPropagationLossModel> ();
	// Set the Attributes to PropagationLossModel - 3GPP Uma
	lossModel->SetAttribute ("ShadowingEnabled", BooleanValue (false)); // disable the shadow fading
	lossModel->SetAttribute ("Frequency", DoubleValue (2.4e9));
	lossModel->SetChannelConditionModel (buildCondModel);

/*
	// Create the propagation loss model AtG
	Ptr<AirToGroundPropagationLossModel> lossModel = CreateObject<AirToGroundPropagationLossModel> ();
	// Set the Attributes to PropagationLossModel - 3GPP Uma
	lossModel->SetAttribute ("Frequency", DoubleValue (5.0e9));
	lossModel->SetAttribute("LoS", BooleanValue(false));
*/

	// Setup building
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

	BuildingsHelper::Install (nodes);

	std::vector<Vector> dronePos_XYZ;
	std::cout << "Drone's Trajectory: " << std::endl;
	std::vector<int> X = createArray(-120, 170, 10);
	std::vector<int> Y = createArray(-200, 250, 10);
	int Z = 35;
	int LoScount = 0;
	int NLoScount = 0;

	for (uint16_t n = 0; n < Y.size(); n++)
		{
		for (uint16_t m = 0; m < X.size(); m++)
		   {
			   Vector pos = Vector(X[m], Y[n], Z);
			   dronePos_XYZ.push_back(pos);
			   std::cout << "Position (X,Y,Z) = (" << pos.x << ", " <<  pos.y << ", " << pos.z << ")" << std::endl;
		   }
		}


	for (uint32_t i = 0; i < dronePos_XYZ.size(); i++)
	{

		Vector posAP = Vector (0.0, 0.0, 11); // AP position
		Vector posDrone = Vector (dronePos_XYZ[i].x, dronePos_XYZ[i].y, dronePos_XYZ[i].z); // Drone position

		a->SetPosition (posAP);
		b->SetPosition (posDrone);

		Ptr<MobilityBuildingInfo> buildingInfoA = a->GetObject<MobilityBuildingInfo> ();
		buildingInfoA->MakeConsistent (a);
		Ptr<MobilityBuildingInfo> buildingInfoB = b->GetObject<MobilityBuildingInfo> ();
		buildingInfoB->MakeConsistent (b);

	    Ptr<ChannelCondition> cond = buildCondModel->GetChannelCondition (a, b);

		std::cout << "\nIteration: " << i << " -> (" << dronePos_XYZ[i].x << ", " <<  dronePos_XYZ[i].y << ", " << dronePos_XYZ[i].z << ")" << std::endl;
		std::cout << "RxPower (eq. 3GPP): " << lossModel->CalcRxPower(20, a, b) << std::endl;
		std::cout << "Channel Condition between (AP, drone): " << cond->GetLosCondition () << std::endl;
		std::cout << "LOS: " << ChannelCondition::LOS << "\tNLOS: " << ChannelCondition::NLOS << std::endl;

		if (cond->GetLosCondition () == ChannelCondition::NLOS){NLoScount++;}
		else{LoScount++;}
	}

	std::cout << "LOS count: " << LoScount << "\tNLOS count: " << NLoScount << std::endl;

	Simulator::Stop (Seconds (1000));
	Simulator::Run ();


	Simulator::Destroy ();
 }
