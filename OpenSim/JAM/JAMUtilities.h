#ifndef OPENSIM_JAM_UTILITIES_H_
#define OPENSIM_JAM_UTILITIES_H_
/* -------------------------------------------------------------------------- *
 *                             JAMUtilities.h                              *
 * -------------------------------------------------------------------------- *
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


//=============================================================================
// INCLUDES
//=============================================================================
//#include <OpenSim/Simulation/Model/Analysis.h>
#include <algorithm>
#include <limits>
#include <string>
#include <vector>

//=============================================================================
//=============================================================================



//=============================================================================
//STRING TOOLS
//=============================================================================
/** Split string at delimiter
*/
std::vector<std::string> split_string(std::string s, std::string delimiter);

bool contains_string(std::vector<std::string> s_vector, std::string s);
bool contains_string(std::vector<std::string> s_vector, std::string s, int& index);
int find_nearest(std::vector<double> in_vec, double value);
std::string erase_sub_string(std::string mainStr, const std::string & toErase);
std::string replace_string(std::string subject, const std::string& search,
        const std::string& replace);

//=============================================================================
// SETTLING SIMULATIONS
//=============================================================================
/** Decides when a settling simulation is finished, based on the largest change
in the settling coordinates over each simulation step (maxDelta).

Settling is finished when
  1. maxDelta is at or below the settle threshold, or
  2. (only if plateauTolerance > 0) maxDelta has plateaued: over the last
     plateauWindow steps it varied by no more than plateauTolerance (a
     fraction, e.g. 0.1 for 10%) of its largest value in that window, and it
     has already dropped to less than minPlateauReduction() of its value at the
     first step, or
  3. maxSteps() steps have been taken (hitSafetyLimit() is then true).
Condition 3 is a last resort against a run that never settles.
*/
class SettleMonitor {
public:
    /** A plateau only counts once the change per step has dropped below this
    fraction of its first value. */
    static double minPlateauReduction() { return 0.1; }
    /** Safety limit on the number of steps. */
    static int maxSteps() { return 20000; }

    SettleMonitor(double threshold, double plateauTolerance = 0.0,
            int plateauWindow = 10)
            : _threshold(threshold), _plateauTolerance(plateauTolerance),
              _plateauWindow(std::max(plateauWindow, 2)) {}

    /** Record the largest coordinate change of the latest step.
    Returns true if the simulation is settled and should stop. */
    bool update(double maxDelta) {
        _history.push_back(maxDelta);

        if (maxDelta <= _threshold) {
            _reason = "coordinate change below settle threshold";
            return true;
        }

        if (_plateauTolerance > 0 && numSteps() >= _plateauWindow) {
            double hi = 0.0;
            double lo = std::numeric_limits<double>::infinity();
            for (int i = numSteps() - _plateauWindow; i < numSteps(); ++i) {
                hi = std::max(hi, _history[i]);
                lo = std::min(lo, _history[i]);
            }
            if (hi - lo <= _plateauTolerance * hi &&
                    hi <= minPlateauReduction() * _history.front()) {
                _reason = "coordinate change plateaued";
                return true;
            }
        }

        if (numSteps() >= maxSteps()) {
            _hitSafetyLimit = true;
            _reason = "safety step limit reached";
            return true;
        }
        return false;
    }

    int numSteps() const { return (int)_history.size(); }
    bool hitSafetyLimit() const { return _hitSafetyLimit; }
    const std::string& reason() const { return _reason; }

private:
    double _threshold;
    double _plateauTolerance;
    int _plateauWindow;
    std::vector<double> _history;
    bool _hitSafetyLimit = false;
    std::string _reason;
};

 //namespace
#endif // #ifndef OPENSIM_JAM_UTILITIES_H_
