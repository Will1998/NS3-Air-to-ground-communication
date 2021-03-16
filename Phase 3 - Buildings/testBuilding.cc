/*
 * Created by: Gerard Giramé Rizzo <gerardgr@kth.se>
 * Last edit: Nov 19, 2020
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
	double x_min = 20.0, x_max = 30.0;
	double y_min = -10.0, y_max = 10.0;
	double z_min = 0.0, z_max = 15.0;
	Ptr<Building> building = CreateObject <Building> ();
	building->SetBoundaries (Box (x_min, x_max, y_min, y_max, z_min, z_max));
	Ptr<Building> building2 = CreateObject <Building> ();
	building2->SetBoundaries (Box (-x_max, -x_min, y_min, y_max, z_min, z_max));

	BuildingsHelper::Install (nodes);

	std::vector<Vector> dronePos_XYZ;
	std::vector<int> X = createArray(-50, 50, 5);
	std::vector<int> Y = createArray(-30, 30, 5);
	int Z = 25;
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

		Vector posAP = Vector (0.0, 0.0, 0.0); // AP position
		Vector posDrone = Vector (dronePos_XYZ[i].x, dronePos_XYZ[i].y, dronePos_XYZ[i].z); // Drone position

		a->SetPosition (posAP);
		b->SetPosition (posDrone);

		Ptr<MobilityBuildingInfo> buildingInfoA = a->GetObject<MobilityBuildingInfo> ();
		buildingInfoA->MakeConsistent (a);
		Ptr<MobilityBuildingInfo> buildingInfoB = b->GetObject<MobilityBuildingInfo> ();
		buildingInfoB->MakeConsistent (b);

	    Ptr<ChannelCondition> cond = buildCondModel->GetChannelCondition (a, b);

		std::cout << "\nIteration: " << i << " -> \nRxPower (eq. 3GPP): " << lossModel->CalcRxPower(10, a, b) << std::endl;
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
