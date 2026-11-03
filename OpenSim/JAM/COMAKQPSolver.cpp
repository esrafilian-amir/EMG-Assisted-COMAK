/* -------------------------------------------------------------------------- *
 *                            COMAKQPSolver.cpp                               *
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

#include "COMAKQPSolver.h"

#include <OpenSim/Common/Exception.h>

#include <cmath>
#include <limits>
#include <map>
#include <vector>

#ifdef OPENSIM_JAM_WITH_CASADI
#include <casadi/casadi.hpp>
#endif

namespace OpenSim {

bool solveCasadiQP(const std::string& plugin, const SimTK::Vector& hdiag,
        const SimTK::Vector& g, const SimTK::Matrix& A,
        const SimTK::Vector& rhs, const SimTK::Vector& lower,
        const SimTK::Vector& upper, const SimTK::Vector& x0,
        double slack_penalty, SimTK::Vector& x, double& max_slack,
        std::string& status) {
#ifndef OPENSIM_JAM_WITH_CASADI
    OPENSIM_THROW(Exception,
            "COMAKTool solver '" + plugin + "' needs OpenSim to be built with "
            "CasADi (OPENSIM_WITH_CASADI=ON). Use solver 'ipopt' instead.")
    return false;
#else
    const int n = hdiag.size();
    const int m = rhs.size();
    const int nv = n + m; // x, then one free slack per equality row
    const double inf = std::numeric_limits<double>::infinity();

    // A tiny regularization makes the solution unique along directions with
    // no cost (e.g., the secondary coordinates).
    const double regularization = 1e-6;

    // Each row is scaled by its largest entry so that the slack penalty means
    // the same for every row.
    std::vector<double> h(nv, regularization), gv(nv, 0.0), lbx(nv), ubx(nv),
            x0v(nv, 0.0), b(m);
    for (int i = 0; i < n; ++i) {
        h[i] += hdiag[i];
        gv[i] = g[i];
        lbx[i] = lower[i];
        ubx[i] = upper[i];
        x0v[i] = x0[i];
    }
    // quadratic penalty on the slack: a strictly convex QP that is always
    // feasible
    for (int r = 0; r < m; ++r) {
        h[n + r] = slack_penalty;
        lbx[n + r] = -inf;
        ubx[n + r] = inf;
    }

    // Equality rows: (A x + s) / scale = rhs / scale
    casadi::DM Am = casadi::DM::zeros(m, nv);
    for (int r = 0; r < m; ++r) {
        double row_max = 1e-12;
        for (int c = 0; c < n; ++c) {
            row_max = std::max(row_max, std::abs(A(r, c)));
        }
        const double scale = 1.0 / row_max;
        for (int c = 0; c < n; ++c) { Am(r, c) = A(r, c) * scale; }
        Am(r, n + r) = scale;
        b[r] = rhs[r] * scale;
    }
    casadi::DM H = casadi::DM::diag(casadi::DM(h));

    // Keep one solver per problem size and plugin; building it is the
    // expensive part.
    static std::map<std::string, casadi::Function> solvers;
    const std::string key =
            plugin + ":" + std::to_string(n) + ":" + std::to_string(m);
    auto it = solvers.find(key);
    if (it == solvers.end()) {
        casadi::SpDict qp = {{"h", H.sparsity()}, {"a", Am.sparsity()}};
        casadi::Dict options;
        options["print_iter"] = false;
        options["print_header"] = false;
        options["print_info"] = false;
        // return the best point instead of throwing when a solve is hard
        options["error_on_fail"] = false;
        casadi::Function solver;
        try {
            solver = casadi::conic("comak_qp", plugin, qp, options);
        } catch (const std::exception&) {
            // the plugin does not know one of the printing options
            solver = casadi::conic("comak_qp", plugin, qp);
        }
        it = solvers.emplace(key, solver).first;
    }
    casadi::Function& solver = it->second;

    casadi::DMDict arg = {{"h", H}, {"g", casadi::DM(gv)}, {"a", Am},
            {"lbx", casadi::DM(lbx)}, {"ubx", casadi::DM(ubx)},
            {"lba", casadi::DM(b)}, {"uba", casadi::DM(b)},
            {"x0", casadi::DM(x0v)}};

    casadi::DMDict res;
    try {
        res = solver(arg);
    } catch (const std::exception& ex) {
        status = std::string("CasADi QP solver threw: ") + ex.what();
        return false;
    }

    std::vector<double> sol = res.at("x").get_elements();
    x.resize(n);
    max_slack = 0.0;
    for (int i = 0; i < n; ++i) { x[i] = sol[i]; }
    for (int r = 0; r < m; ++r) {
        max_slack = std::max(max_slack, std::abs(sol[n + r]));
    }

    bool success = true;
    status = "ok";
    try {
        casadi::Dict stats = solver.stats();
        auto found = stats.find("success");
        if (found != stats.end()) { success = (bool)found->second; }
        auto rs = stats.find("return_status");
        if (rs != stats.end()) { status = (std::string)rs->second; }
    } catch (const std::exception&) {
    }
    return success;
#endif
}

} // namespace OpenSim
