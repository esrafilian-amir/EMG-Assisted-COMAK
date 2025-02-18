
/* -------------------------------------------------------------------------- *
 *           OpenSim:  JointMechanicsSettings.cpp                             *
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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
#include "JointMechanicsSettings.h"

using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/* Default constructor. */
JointMechanicsFrameTransform::JointMechanicsFrameTransform() {
    constructProperties();
}

/* Convenience constructor. */
JointMechanicsFrameTransform::JointMechanicsFrameTransform(
        const std::string& parent, const std::string& child) {
    constructProperties();
    set_parent_frame(parent);
    set_child_frame(child);
}

/*
 * Construct Properties
 */
void JointMechanicsFrameTransform::constructProperties() { 
    Array<std::string> default_rotation_seq;
    default_rotation_seq.append("z");
    default_rotation_seq.append("x");
    default_rotation_seq.append("y");

    constructProperty_parent_frame("");
    constructProperty_child_frame("");
    constructProperty_rotation_type("body");
    constructProperty_rotation_sequence(default_rotation_seq);
    constructProperty_output_coordinates(true);
    constructProperty_output_transformation_matrix(false);

}