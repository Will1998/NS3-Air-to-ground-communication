/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011, 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Author: Gerard Giramé Rizzo  <gerardgr@kth.se>
 * Date: Oct 8, 2020
 * 
 */
#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/boolean.h"
#include "ns3/mobility-model.h"
#include "ns3/propagation-loss-model.h"
#include <cmath>

#include "air-to-ground-propagation-loss-model.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("AirToGroundPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED (AirToGroundPropagationLossModel);


TypeId
AirToGroundPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::AirToGroundPropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<AirToGroundPropagationLossModel> ()
    .AddAttribute ("Frequency",
                   "The carrier frequency (in Hz) at which propagation occurs  (default is 5.180 GHz).",
                   DoubleValue (5.180e9),
                   MakeDoubleAccessor (&AirToGroundPropagationLossModel::SetFrequency,
                                       &AirToGroundPropagationLossModel::GetFrequency),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("SystemLoss", "The system loss",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (&AirToGroundPropagationLossModel::m_systemLoss),
				   MakeDoubleChecker<double> ())
	.AddAttribute ("LoS","Line-of-Sigth",
				   BooleanValue (true),
				   MakeBooleanAccessor (&AirToGroundPropagationLossModel::m_isLoS),
				   MakeBooleanChecker ())
  ;
  return tid;
}

AirToGroundPropagationLossModel::AirToGroundPropagationLossModel ()
  : PropagationLossModel ()
{
}

AirToGroundPropagationLossModel::~AirToGroundPropagationLossModel ()
{
}

void
AirToGroundPropagationLossModel::SetSystemLoss (double systemLoss)
{
  m_systemLoss = systemLoss;
}
double
AirToGroundPropagationLossModel::GetSystemLoss (void) const
{
  return m_systemLoss;
}
void
AirToGroundPropagationLossModel::SetFrequency (double frequency)
{
  m_frequency = frequency;
  static const double C = 299792458.0; // speed of light in vacuum
  m_lambda = C / frequency;
}
double
AirToGroundPropagationLossModel::GetFrequency (void) const
{
  return m_frequency;
}

double
AirToGroundPropagationLossModel::DbmToW (double dbm) const
{
  double mw = std::pow (10.0,dbm / 10.0);
  return mw / 1000.0;
}

double
AirToGroundPropagationLossModel::DbmFromW (double w) const
{
  double dbm = std::log10 (w * 1000.0) * 10.0;
  return dbm;
}

double
AirToGroundPropagationLossModel::GetLoss (Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{
  double loss = 0.0;
  double fghz = m_frequency / 1e9;
  double dist_3D = a->GetDistanceFrom (b);

  // get the height of the drone node
  double h = (a->GetPosition ().z > b->GetPosition ().z ? a->GetPosition ().z : b->GetPosition ().z);

  if (m_isLoS)
  {
	  loss = 28 + 22*log10(dist_3D) + 20*log10(fghz);
	//(22* log_f) - log_aHeight + (((44.9 - (6.55 * std::log10 (hb)) )) * std::log10 (dist)) - log_bHeight + C;
  } else
  {
	  loss = -17.5 + (46-7*log10(h))*log10(dist_3D) + 20*log10(40*M_PI*fghz/3);
  }

  //std::cout << "distance = " << dist_3D << ", Path Loss = " << loss << std::endl;
  return loss;
}

double 
AirToGroundPropagationLossModel::DoCalcRxPower (double txPowerDbm,
						Ptr<MobilityModel> a,
						Ptr<MobilityModel> b) const
{
  return (txPowerDbm - GetLoss (a, b));
}

int64_t
AirToGroundPropagationLossModel::DoAssignStreams (int64_t stream)
{
  return 0;
}


} // namespace ns3
