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
#include <OpenSim/Common/Logger.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
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

/** JAM works in SI units: meters, radians or degrees, Newtons. The .mot and
.sto files have no length unit (the .trc marker files do, and OpenSim converts
them to meters), so a translational coordinate that is larger than 5 is most
likely in mm or cm. Warn once per coordinate. */
inline void warnIfTranslationsNotMeters(const OpenSim::Model& model,
        OpenSim::Storage& store, const std::string& file) {
    const OpenSim::Array<std::string>& labels = store.getColumnLabels();
    const OpenSim::CoordinateSet& coordinates = model.getCoordinateSet();
    for (int i = 1; i < labels.getSize(); ++i) {
        const std::string& name = labels[i];
        const int index = coordinates.getIndex(name);
        if (index < 0 || coordinates.get(index).getMotionType() !=
                OpenSim::Coordinate::MotionType::Translational) { continue; }
        OpenSim::Array<double> values;
        store.getDataColumn(name, values);
        double largest = 0.0;
        for (int k = 0; k < values.getSize(); ++k) {
            largest = (std::max)(largest, std::abs(values[k]));
        }
        if (largest > 5.0) {
            OpenSim::log_warn("{}: the translational coordinate {} goes up "
                    "to {}. JAM expects meters (SI); this looks like mm or "
                    "cm. Convert the file to meters.", file, name, largest);
        }
    }
}

/** Print a prominent notice that a tool is running a setup (script or file)
that was written for an older version, with the changes made to it. */
inline void logOldSetupNotice(const std::string& tool,
        const std::vector<std::string>& changes, bool upgraded) {
    OpenSim::log_critical("");
    OpenSim::log_critical("********************************************************");
    OpenSim::log_critical("  YOU ARE USING AN OLD VERSION OF THE {} SETUP", tool);
    OpenSim::log_critical("  (no setup_version): a script, or an XML file saved by");
    OpenSim::log_critical("  an older OpenSim JAM.");
    if (upgraded) {
        OpenSim::log_critical("  It was upgraded to the current defaults:");
        for (const auto& c : changes) {
            OpenSim::log_critical("    {}", c);
        }
    } else {
        OpenSim::log_critical("  upgrade_old_setup is false: the old values are kept.");
    }
    OpenSim::log_critical("  Please update your script or XML: review the settings,");
    OpenSim::log_critical("  add set_setup_version(1), and save the setup again.");
    OpenSim::log_critical("  Updated examples are in the Resources/Code/Matlab/JAM");
    OpenSim::log_critical("  and Resources/Code/Python/JAM folders of your OpenSim");
    OpenSim::log_critical("  installation.");
    OpenSim::log_critical("********************************************************");
    OpenSim::log_critical("");
}

/** File to which an upgraded setup is saved. documentFile is the setup file
that was loaded (empty if the setup was made in a script). where is empty (the
file name with _v<version> before the extension, next to the setup file or in
the working directory), "inplace" (the setup file itself), a folder (ends with
a slash) or a file name. */
inline std::string upgradedSetupFile(const std::string& documentFile,
        const std::string& where, int version, const std::string& defaultName) {
    if (where == "inplace") {
        if (documentFile.empty()) {
            throw std::runtime_error("saveUpgradedSetup('inplace'): the setup "
                    "was not loaded from a file.");
        }
        return documentFile;
    }
    if (!where.empty() && where.back() != '/' && where.back() != '\\') {
        return where;
    }
    std::string dir = where;
    std::string name = defaultName;
    if (!documentFile.empty()) {
        const size_t slash = documentFile.find_last_of("/\\");
        const std::string base = (slash == std::string::npos)
                ? documentFile : documentFile.substr(slash + 1);
        if (dir.empty() && slash != std::string::npos) {
            dir = documentFile.substr(0, slash + 1);
        }
        const size_t dot = base.find_last_of('.');
        name = (dot == std::string::npos) ? base : base.substr(0, dot);
    }
    return dir + name + "_v" + std::to_string(version) + ".xml";
}

 //namespace
#endif // #ifndef OPENSIM_JAM_UTILITIES_H_
