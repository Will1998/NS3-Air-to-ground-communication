/*
 * Created by: Gerard Giramé Rizzo <gerardgr@kth.se>
 * Last edit: Nov 29, 2020
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
	// (1) Side building
	Ptr<Building> building = CreateObject <Building> ();
	building->SetBoundaries (Box (-72, -60, 9, 37, 0, 5));

	// (2) Swerim AB (building block #1)
	Ptr<Building> building20 = CreateObject <Building> ();
	building20->SetBoundaries (Box (-86, -17, 14, 24, 0, 24));
	Ptr<Building> building21 = CreateObject <Building> ();
	building21->SetBoundaries (Box (-75, -17, 24, 46, 0, 24));
	Ptr<Building> building22 = CreateObject <Building> ();
	building22->SetBoundaries (Box (-86, -17, 46, 59, 0, 24));

	// (3) Add Health Media AB (building block #2)
	Ptr<Building> building30 = CreateObject <Building> ();
	building30->SetBoundaries (Box (-89, -19, 70, 82, 0, 22));
	Ptr<Building> building31 = CreateObject <Building> ();
	building31->SetBoundaries (Box (-89, -5, 82, 90, 0, 22));

	// (4) Intel Sweden AB (building block #3)
	Ptr<Building> building40 = CreateObject <Building> ();
	building40->SetBoundaries (Box (9, 70, 3, 17, 0, 21));
	Ptr<Building> building41 = CreateObject <Building> ();
	building41->SetBoundaries (Box (9, 51, 17, 23, 0, 21));
	Ptr<Building> building42 = CreateObject <Building> ();
	building42->SetBoundaries (Box (9, 33, 23, 37, 0, 21));
	Ptr<Building> building43 = CreateObject <Building> ();
	building43->SetBoundaries (Box (9, 76, 37, 50, 0, 21));
	Ptr<Building> building44 = CreateObject <Building> ();
	building44->SetBoundaries (Box (9, 61, 50, 57, 0, 21));
	Ptr<Building> building45 = CreateObject <Building> ();
	building45->SetBoundaries (Box (9, 38, 57, 73, 0, 21));
	Ptr<Building> building46 = CreateObject <Building> ();
	building46->SetBoundaries (Box (9, 86, 73, 83, 0, 21));
	Ptr<Building> building47 = CreateObject <Building> ();
	building47->SetBoundaries (Box (-5, 67, 83, 90, 0, 21));

	// (5) KTH Kista - ELectrum (building block #4)
	Ptr<Building> building50 = CreateObject <Building> ();
	building50->SetBoundaries (Box (-84, -43, -72, -12, 0, 26));
	Ptr<Building> building51 = CreateObject <Building> ();
	building51->SetBoundaries (Box (-43, -36, -64, -19, 0, 26));
	Ptr<Building> building52 = CreateObject <Building> ();
	building52->SetBoundaries (Box (-36, -15, -72, -19, 0, 26));
	Ptr<Building> building53 = CreateObject <Building> ();
	building53->SetBoundaries (Box (-15, -4, -19, -12, 0, 26));
	Ptr<Building> building54 = CreateObject <Building> ();
	building54->SetBoundaries (Box (-4, 6, -19, -16, 0, 26));
	Ptr<Building> building55 = CreateObject <Building> ();
	building55->SetBoundaries (Box (6, 38, -19, -12, 0, 26));
	Ptr<Building> building56 = CreateObject <Building> ();
	building56->SetBoundaries (Box (-15, 52, -30, -19, 0, 26));
	Ptr<Building> building57 = CreateObject <Building> ();
	building57->SetBoundaries (Box (-15, 41, -54, -30, 0, 26));
	Ptr<Building> building58 = CreateObject <Building> ();
	building58->SetBoundaries (Box (-15, 52, -65, -54, 0, 26));
	Ptr<Building> building59 = CreateObject <Building> ();
	building59->SetBoundaries (Box (-15, 38, -72, -65, 0, 26));
	Ptr<Building> building510 = CreateObject <Building> ();
	building510->SetBoundaries (Box (-15, 17, -102, -72, 0, 26));
	Ptr<Building> building511 = CreateObject <Building> ();
	building511->SetBoundaries (Box (45, 69, -102, -95, 0, 26));
	Ptr<Building> building512 = CreateObject <Building> ();
	building512->SetBoundaries (Box (-78, -56, -102, -95, 0, 26));
	Ptr<Building> building513 = CreateObject <Building> ();
	building513->SetBoundaries (Box (-83, 74, -112, -102, 0, 26));
	Ptr<Building> building514 = CreateObject <Building> ();
	building514->SetBoundaries (Box (-42, 48, -120, -112, 0, 26));

	BuildingsHelper::Install (nodes);

	std::vector<Vector> dronePos_XYZ;
	std::cout << "Drone's Trajectory: " << std::endl;
	std::vector<int> X = createArray(-115, 115, 10);
	std::vector<int> Y = createArray(-130, 115, 10);
	int Z = 30;
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

		Vector posAP = Vector (0.0, 0.0, 1.5); // AP position
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
