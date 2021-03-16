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

#ifndef AIR_TO_GROUND_PROPAGATION_LOSS_MODEL_H
#define AIR_TO_GROUND_PROPAGATION_LOSS_MODEL_H

#include <ns3/propagation-loss-model.h>

namespace ns3 {


/**
 * 
 * \brief this class implements the Air To Ground propagation loss model
 *
 * this class implements the Air To Ground propagation loss model
 * For more information about the model, please see the propagation module documentation:
 * 3GPP TR 36.777, “Enhanced LTE support for aerial vehicles,” Tech. Rep., 2018. [Online]. Available: ftp://www.3gpp.org
 *
 */
class AirToGroundPropagationLossModel : public PropagationLossModel
{

public:

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  AirToGroundPropagationLossModel ();
  virtual ~AirToGroundPropagationLossModel ();

  /**
    * \param frequency (Hz)
    *
    * Set the carrier frequency used in the AirToGround model calculation.
    */
   void SetFrequency (double frequency);

   /**
    * \param systemLoss (dimension-less)
    *
    * Set the system loss used by the AirToGround propagation model.
    */
   void SetSystemLoss (double systemLoss);

   /**
    * \returns the current frequency (Hz)
    */
   double GetFrequency (void) const;

   /**
    * \returns the current system loss (dimension-less)
    */
   double GetSystemLoss (void) const;

  /** 
   * \param a the first mobility model
   * \param b the second mobility model
   * 
   * \return the loss in dBm for the propagation between
   * the two given mobility models
   */
  double GetLoss (Ptr<MobilityModel> a, Ptr<MobilityModel> b) const;

private:
  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   */
  AirToGroundPropagationLossModel (const AirToGroundPropagationLossModel &);
  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \returns
   */
  AirToGroundPropagationLossModel & operator = (const AirToGroundPropagationLossModel &);

  virtual double DoCalcRxPower (double txPowerDbm,
                                Ptr<MobilityModel> a,
                                Ptr<MobilityModel> b) const;
  virtual int64_t DoAssignStreams (int64_t stream);
  
  /**
   * Transforms a Dbm value to Watt
   * \param dbm the Dbm value
   * \return the Watts
   */
  double DbmToW (double dbm) const;

  /**
   * Transforms a Watt value to Dbm
   * \param w the Watt value
   * \return the Dbm
   */
  double DbmFromW (double w) const;

  double m_lambda;        //!< the carrier wavelength
  double m_frequency;     //!< the carrier frequency
  double m_systemLoss;    //!< the system loss
  bool m_isLoS;           //!< LoS condition. True -> LoS; False -> NLoS
};

} // namespace ns3


#endif // AIR_TO_GROUND_PROPAGATION_LOSS_MODEL_H

