#ifndef OPENSIM_COMAK_QP_SOLVER_H_
#define OPENSIM_COMAK_QP_SOLVER_H_
/* -------------------------------------------------------------------------- *
 *                             COMAKQPSolver.h                                *
 * -------------------------------------------------------------------------- *
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

#include "osimJAMDLL.h"

#include <SimTKcommon.h>
#include <string>

namespace OpenSim {

/** True if OpenSim was built with CasADi (OPENSIM_WITH_CASADI), which the
COMAKTool solvers 'casadi_ipopt' and 'casadi_qrqp' need. */
OSIMJAM_API bool isCasadiQPAvailable();

/** Solve the quadratic program

        minimize    0.5 * x' * diag(hdiag) * x + g' * x
        subject to  A * x = rhs,   lower <= x <= upper

with a CasADi conic solver plugin (only "qrqp", an active-set method, is
supported; "ipqp" did not converge on this problem). Each equality row is
scaled by its largest entry and relaxed with a free slack variable that is
penalized by 0.5 * slack_penalty times its square, so the problem is strictly
convex and always feasible. The largest slack, i.e. the largest violation of a
scaled row, is returned in max_slack.

Returns true if the solver reports success. status receives a short message.
Throws an Exception if OpenSim was built without CasADi
(OPENSIM_WITH_CASADI). */
OSIMJAM_API bool solveCasadiQP(const std::string& plugin,
        const SimTK::Vector& hdiag, const SimTK::Vector& g,
        const SimTK::Matrix& A, const SimTK::Vector& rhs,
        const SimTK::Vector& lower, const SimTK::Vector& upper,
        const SimTK::Vector& x0, double slack_penalty, SimTK::Vector& x,
        double& max_slack, std::string& status);

} // namespace OpenSim

#endif // OPENSIM_COMAK_QP_SOLVER_H_
