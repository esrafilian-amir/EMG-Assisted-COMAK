#ifndef _OPENSIM_COMAK_SETTINGS_H_
#define _OPENSIM_COMAK_SETTINGS_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  COMAKSettings.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Colin Smith                                                     *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


// INCLUDE
#include "osimJAMDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/FunctionSet.h>
//=============================================================================
//=============================================================================
namespace OpenSim {


class OSIMJAM_API COMAKSecondaryCoordinate : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(COMAKSecondaryCoordinate, Object)

public:
    OpenSim_DECLARE_PROPERTY(coordinate, std::string, 
        "Path to Coordinate in model.")

    /*OpenSim_DECLARE_PROPERTY(comak_damping, double,
        "Coefficient to penalize frame-to-frame changes in predicted "
        "secondary coordinate values. "
        "The default value is 1.0.")*/

    OpenSim_DECLARE_PROPERTY(max_change, double, "Limit on the maximum "
        "frame-to-frame changes in secondary coordinate values (rad or m), "
        "summed over the COMAK iterations of a time step. A value of 0 or "
        "less removes the limit for this coordinate, which shows how much "
        "the coordinate moves when it is free. The default value is 0, which means no limit.")

    COMAKSecondaryCoordinate();
    void constructProperties();
}; //END of class COMAKSecondaryCoordinate

class OSIMJAM_API COMAKCostFunctionParameter : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(COMAKCostFunctionParameter, Object)

public:
    OpenSim_DECLARE_PROPERTY(actuator, std::string, 
        "Path to actuator in model.")

    OpenSim_DECLARE_PROPERTY(emg_gamma_weight,
                      double, "weighting coefficient to following EMGs")

    OpenSim_DECLARE_PROPERTY(weight, Function, 
        "Time varying weighting coefficient that multiplies the entire "
        "actuator activation term in the COMAK optimization cost function."
        "W in the cost function: W*(a-d)^n. "
        "The Default value is Constant(1.0).")

    OpenSim_DECLARE_PROPERTY(desired_activation, Function, 
        "Time varying term that is subtracted from the actuator activation "
        "within the term rasied to the activation_exponent in the COMAK "
        "optimization cost function."
        "d in the cost function: W*(a-d)^n. "
        "The Default value is Constant(0.0).")
    
    OpenSim_DECLARE_PROPERTY(activation_lower_bound, Function, 
        "Time varying lower bound on the actuator activation in the "
        "COMAK optimization constraints."
        "The Default value is Constant(0.0).")

    OpenSim_DECLARE_PROPERTY(activation_upper_bound, Function, 
        "Time varying lower bound on the actuator activation in the "
        "COMAK optimization constraints."
        "The Default value is Constant(1.0).")

    OpenSim_DECLARE_PROPERTY(emg_mapping, std::string,
        "Which channels of the COMAKTool emg_file drive this muscle: terms "
        "separated by semicolons, each 'label' or 'weight*label'. The "
        "desired activation of the muscle is the sum of weight*channel at "
        "the time of the simulation minus the electromechanical delay, e.g. "
        "'0.5*vasmed; 0.5*vaslat' or 'Rectus Femoris'. Labels may contain "
        "spaces. They are matched ignoring case, leading and trailing "
        "spaces, repeated spaces, and the difference between '_' and ' '. "
        "A weight is optional (1). Values above 1 are set to 1. A muscle "
        "whose desired_activation is set explicitly keeps that function. "
        "Empty by default.")

    OpenSim_DECLARE_PROPERTY(passive_force_scale, double,
        "Scale of the passive fiber force of this muscle when "
        "use_passive_muscle_force of the COMAKTool is true: 1 keeps it, 0 "
        "removes it (e.g. for a muscle with a very long tendon such as the "
        "soleus, whose fiber length is unrealistic with a rigid tendon). It "
        "has no effect if use_passive_muscle_force is false, and it is not "
        "used with use_tendon_compliance, where the passive force comes from "
        "the equilibrium of the fiber and the tendon. Muscles without a "
        "COMAKCostFunctionParameter use 1. The default value is 1.")

    OpenSim_DECLARE_PROPERTY(emg_delay, double,
        "Electromechanical delay (s) of this muscle: the EMG at time t - "
        "emg_delay drives the activation at t. A value below 0 or a missing "
        "property uses the COMAKTool emg_electromechanical_delay. The default "
        "value is -1.")

    COMAKCostFunctionParameter();
    void constructProperties();
}; //END of class COMAKCostFunctionParameter

}; //namespace
//=============================================================================
//=============================================================================

#endif // _OPENSIM_COMAK_SETTINGS_H_


