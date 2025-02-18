  
#ifndef OPENSIM_JAM_ABOUT_H_
#define OPENSIM_JAM_ABOUT_H_
/* -------------------------------------------------------------------------- *
 * OpenSim JAM: About.h                                                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Colin Smith                                                     *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "osimJAMDLL.h"
#include <string>

namespace OpenSim {

OSIMJAM_API std::string GetJAMVersionAndDate();

OSIMJAM_API std::string GetJAMVersion();
}

#endif // OPENSIM_JAM_ABOUT_H_