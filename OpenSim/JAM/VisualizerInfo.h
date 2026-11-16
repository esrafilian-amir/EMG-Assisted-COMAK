#ifndef OPENSIM_JAM_VISUALIZER_INFO_H_
#define OPENSIM_JAM_VISUALIZER_INFO_H_
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  VisualizerInfo.h                               *
 * -------------------------------------------------------------------------- *
 * Information as screen text in the visualizer window of the COMAK tools:    *
 * rows of a label and a value in aligned columns (SolverInfo), drawn by a    *
 * decoration generator, and the contact forces per knee for those rows.      *
 * Used by the COMAK iterations and by the settling and sweep simulations of  *
 * the COMAK inverse kinematics tool.                                         *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Common/Logger.h>
#include <OpenSim/Simulation/Model/Blankevoort1991Ligament.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Smith2018ArticularContactForce.h>
#include <OpenSim/Simulation/Model/Smith2018ContactMesh.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace OpenSim {

// Rows of screen text: one line per row, the label and the value in aligned
// columns.
struct SolverInfo {
    std::vector<std::pair<std::string, std::string>> rows;
};

class SolverInfoDecorator : public SimTK::DecorationGenerator {
public:
    SolverInfoDecorator(std::shared_ptr<SolverInfo> info, double scale)
        : _info(info), _scale(scale) {}

    void generateDecorations(const SimTK::State&,
            SimTK::Array_<SimTK::DecorativeGeometry>& geometry) override {
        for (const auto& row : _info->rows) {
            std::string label = row.first;
            if (label.size() < 12) { label.append(12 - label.size(), ' '); }
            SimTK::DecorativeText text(label + row.second);
            text.setIsScreenText(true);
            text.setScaleFactors(SimTK::Vec3(_scale));
            text.setColor(SimTK::Vec3(0.15, 0.15, 0.15));
            geometry.push_back(text);
        }
    }

private:
    std::shared_ptr<SolverInfo> _info;
    double _scale;
};

// Adds a row with a printf formatted value.
template <typename... Args>
void addInfoRow(SolverInfo& info, const std::string& label, const char* format,
        Args... args) {
    char v[160];
    std::snprintf(v, sizeof(v), format, args...);
    info.rows.emplace_back(label, v);
}

// The side of a knee from the name of a contact force or mesh ("..._r",
// "..._l", "right", "left"): "Right", "Left" or "" if it has none.
inline std::string contactSide(const std::string& name) {
    std::string n = name;
    for (auto& ch : n) { ch = (char)std::tolower(ch); }
    if (n.find("right") != std::string::npos) { return "Right"; }
    if (n.find("left") != std::string::npos) { return "Left"; }
    const size_t k = n.size();
    if (k >= 2 && n[k - 2] == '_' && n[k - 1] == 'r') { return "Right"; }
    if (k >= 2 && n[k - 2] == '_' && n[k - 1] == 'l') { return "Left"; }
    if (n.find("_r_") != std::string::npos) { return "Right"; }
    if (n.find("_l_") != std::string::npos) { return "Left"; }
    return "";
}

// The ligament families of the rows of the visualizer: the total force of a
// family is the sum of the forces of its bundles (ACLam1..., ACLpl1... are the
// ACL), from the names of the Blankevoort1991Ligament components.
enum LigamentFamily { kACL = 0, kPCL, kLCL, kMCL, kPatellarTendon,
        kNumLigamentFamilies };

// The family of a ligament from its name ("ACLpl3", "ACLpl3_r", "PT2", ...),
// or -1 if it is none of them.
inline int ligamentFamily(std::string name) {
    // the side at the end of the name
    const size_t k = name.size();
    if (k >= 2 && name[k - 2] == '_' &&
            (name[k - 1] == 'r' || name[k - 1] == 'l')) {
        name.resize(k - 2);
    }
    // the number of the bundle
    while (!name.empty() && std::isdigit((unsigned char)name.back())) {
        name.pop_back();
    }
    if (name == "ACL" || name == "ACLam" || name == "ACLpl") { return kACL; }
    if (name == "PCL" || name == "PCLal" || name == "PCLpm") { return kPCL; }
    if (name == "LCL") { return kLCL; }
    if (name == "MCL" || name == "MCLs" || name == "MCLd" ||
            name == "MCLp") { return kMCL; }
    if (name == "PT") { return kPatellarTendon; }
    return -1;
}

// Forces (N) of one knee for the rows of the visualizer: contact forces and
// the total forces of the ligament families.
struct KneeForces {
    std::string side;
    SimTK::Vec3 medial = SimTK::Vec3(0);
    SimTK::Vec3 lateral = SimTK::Vec3(0);
    double patella = 0.0;
    bool have_tibia = false, have_patella = false;
    double ligament[kNumLigamentFamilies] = {0, 0, 0, 0, 0};
    bool have_ligament[kNumLigamentFamilies] = {false, false, false, false,
            false};
};

// Contact forces (N): medial and lateral tibial plateau (sum of the
// non-overlapping tibia regions) and patellofemoral, per knee (the side is
// taken from the names of the contact force and its meshes). The rows of a
// knee whose forces are switched off keep their last values. If the forces
// cannot be read, `failed` is set (and a warning logged once), `knees` is
// cleared and the caller does not call this again. The state must be realized
// to the dynamics stage.
inline void updateKneeForces(const Model& model, const SimTK::State& state,
        std::vector<KneeForces>& knees, bool& failed) {
    if (failed) { return; }
    std::set<std::string> knees_updated;
    auto lower = [](std::string s) {
        for (auto& ch : s) { ch = (char)std::tolower(ch); }
        return s;
    };
    try {
        for (const auto& cf :
                model.getComponentList<Smith2018ArticularContactForce>()) {
            if (!cf.appliesForce(state)) { continue; }
            const auto& cast = cf.getSocket<Smith2018ContactMesh>(
                    "casting_mesh").getConnectee();
            const auto& targ = cf.getSocket<Smith2018ContactMesh>(
                    "target_mesh").getConnectee();
            const std::string cn = lower(cast.getName());
            const std::string tn = lower(targ.getName());
            std::string side = contactSide(cf.getName());
            if (side.empty()) { side = contactSide(cast.getName()); }
            if (side.empty()) { side = contactSide(targ.getName()); }
            KneeForces* knee = nullptr;
            for (auto& k : knees) {
                if (k.side == side) { knee = &k; }
            }
            if (!knee) {
                knees.emplace_back();
                knees.back().side = side;
                knee = &knees.back();
            }
            if (knees_updated.insert(side).second) {
                knee->medial = SimTK::Vec3(0);
                knee->lateral = SimTK::Vec3(0);
                knee->patella = 0.0;
                knee->have_tibia = false;
                knee->have_patella = false;
            }
            if (cn.find("patella") != std::string::npos ||
                    tn.find("patella") != std::string::npos) {
                knee->patella += cf.getOutputValue<SimTK::Vec3>(
                        state, "casting_total_contact_force").norm();
                knee->have_patella = true;
                continue;
            }
            const bool tibia_cast = cn.find("tibia") != std::string::npos;
            const bool tibia_target = tn.find("tibia") != std::string::npos;
            // (the target regional force cache is not available during COMAK)
            if (!tibia_cast || tibia_target) { continue; }
            const SimTK::Vector_<SimTK::Vec3> regional = tibia_cast
                    ? cf.getOutputValue<SimTK::Vector_<SimTK::Vec3>>(
                              state, "casting_regional_contact_force")
                    : cf.getOutputValue<SimTK::Vector_<SimTK::Vec3>>(
                              state, "target_regional_contact_force");
            const std::vector<std::string>& names =
                    (tibia_cast ? cast : targ).getRegionalNames();
            const int n = std::min((int)names.size(), 12);
            for (int r = 0; r < n && r < regional.size(); ++r) {
                if (names[r].rfind("medial", 0) == 0) {
                    knee->medial += regional[r];
                } else if (names[r].rfind("lateral", 0) == 0) {
                    knee->lateral += regional[r];
                }
            }
            knee->have_tibia = true;
        }

        // Total forces of the ligament families (the sum of the forces of the
        // bundles; the ligaments of a knee that are switched off keep their
        // last values)
        std::set<std::string> ligaments_updated;
        for (const auto& lig :
                model.getComponentList<Blankevoort1991Ligament>()) {
            const int family = ligamentFamily(lig.getName());
            if (family < 0 || !lig.appliesForce(state)) { continue; }
            std::string side = contactSide(lig.getName());
            if (side.empty()) { side = contactSide(lig.getAbsolutePathString()); }
            KneeForces* knee = nullptr;
            for (auto& k : knees) {
                if (k.side == side) { knee = &k; }
            }
            if (!knee) {
                knees.emplace_back();
                knees.back().side = side;
                knee = &knees.back();
            }
            if (ligaments_updated.insert(side).second) {
                for (int f = 0; f < kNumLigamentFamilies; ++f) {
                    knee->ligament[f] = 0.0;
                    knee->have_ligament[f] = false;
                }
            }
            knee->ligament[family] +=
                    lig.getOutputValue<double>(state, "total_force");
            knee->have_ligament[family] = true;
        }
    } catch (const std::exception& ex) {
        knees.clear();
        failed = true;
        log_warn("Contact forces not shown in the visualizer: {}", ex.what());
    } catch (...) {
        knees.clear();
        failed = true;
    }
}

// The contact force rows of the knees (with several knees every row starts
// with the side).
inline void addKneeForceRows(const std::vector<KneeForces>& knees,
        SolverInfo& info) {
    const bool several_knees = knees.size() > 1;
    for (const auto& knee : knees) {
        const bool named = several_knees && !knee.side.empty();
        const std::string prefix = named ? knee.side + " " : "";
        const char* medial_label = named ? "medial tibia contact force"
                                         : "Medial tibia contact force";
        const char* lateral_label = named ? "lateral tibia contact force"
                                          : "Lateral tibia contact force";
        const char* patella_label = named ? "patella contact force"
                                          : "Patella contact force";
        if (knee.have_tibia) {
            addInfoRow(info, prefix + medial_label, "%.0f N",
                    knee.medial.norm());
            addInfoRow(info, prefix + lateral_label, "%.0f N",
                    knee.lateral.norm());
        }
        if (knee.have_patella) {
            addInfoRow(info, prefix + patella_label, "%.0f N", knee.patella);
        }
        static const char* ligament_labels[kNumLigamentFamilies] = {
                "ACL force", "PCL force", "LCL force", "MCL force",
                "Patellar tendon force"};
        for (int f = 0; f < kNumLigamentFamilies; ++f) {
            if (!knee.have_ligament[f]) { continue; }
            std::string label = ligament_labels[f];
            // (the first letter is lower case after the side, except the
            // names of the ligaments)
            if (f == kPatellarTendon && named) { label[0] = 'p'; }
            addInfoRow(info, prefix + label, "%.0f N", knee.ligament[f]);
        }
    }
}

} // namespace OpenSim

#endif // OPENSIM_JAM_VISUALIZER_INFO_H_
