/* -------------------------------------------------------------------------- *
 *                      Smith2018ContactMesh.cpp                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
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

#include "Smith2018ContactMesh.h"
#include <sstream>
#include <OpenSim/Common/PropertyStr.h>

#include "OpenSim/Common/Object.h"
#include "OpenSim/JAM/JAMUtilities.h"
#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/SimbodyEngine/Body.h"

#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
// #include "simmath/internal/common.h"
// #include "simmath/LinearAlgebra.h"
// #include "simmath/internal/ContactGeometry.h"
// #include "simmath/internal/OrientedBoundingBox.h"
// #include "simmath/internal/OBBTree.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <math.h>
#include <set>

using namespace OpenSim;

using std::set;

//=============================================================================
// CONSTRUCTOR
//=============================================================================

Smith2018ContactMesh::Smith2018ContactMesh()
        : ContactGeometry(), _decorative_mesh(nullptr) {
    setNull();
    constructProperties();
    _mesh_is_cached = false;
    _init_mesh_from_file = true;
}

Smith2018ContactMesh::Smith2018ContactMesh(
        const std::string& name, const std::string& mesh_file)
        : ContactGeometry(), _decorative_mesh(nullptr) {
    setNull();
    constructProperties();
    _mesh_is_cached = false;
    _init_mesh_from_file = true;

    setName(name);
    set_mesh_file(mesh_file);
}

Smith2018ContactMesh::Smith2018ContactMesh(const std::string& name,
        const std::string& mesh_file, const PhysicalFrame& parent_frame)
        : ContactGeometry(parent_frame), _decorative_mesh(nullptr) {
    setNull();
    constructProperties();
    _mesh_is_cached = false;
    _init_mesh_from_file = true;

    setName(name);
    set_mesh_file(mesh_file);
    updSocket<PhysicalFrame>("scale_frame").connect(parent_frame);
}

Smith2018ContactMesh::Smith2018ContactMesh(const std::string& name,
        const std::string& mesh_file, const PhysicalFrame& frame,
        const SimTK::Vec3& location, const SimTK::Vec3& orientation)
        : ContactGeometry(location, orientation, frame) {
    Smith2018ContactMesh(name, mesh_file, frame);
}

Smith2018ContactMesh::Smith2018ContactMesh(const std::string& name,
        const std::string& mesh_file, const PhysicalFrame& frame,
        const SimTK::Vec3& location, const SimTK::Vec3& orientation,
        bool use_variable_thickness, const std::string& mesh_back_file,
        double min_thickness, double max_thickness)
        : ContactGeometry(location, orientation, frame) {
    Smith2018ContactMesh(name, mesh_file, frame, location, orientation);
    set_use_variable_thickness(use_variable_thickness);
    set_mesh_back_file(mesh_back_file);
    set_min_thickness(min_thickness);
    set_max_thickness(max_thickness);
}

Smith2018ContactMesh::Smith2018ContactMesh(const std::string& name,
        SimTK::Vector_<SimTK::Vec3> vertices,
        std::vector<std::vector<int>> triangles) {
    setNull();
    constructProperties();

    setName(name);
    _mesh_is_cached = true;
    _init_mesh_from_file = false;

    _faces = triangles;

    _num_faces = static_cast<int>(triangles.size());

    _vertex_locations = vertices;
    _num_vertices = vertices.size();

    initializeMesh();
}

void Smith2018ContactMesh::setNull() {
    setAuthors("Colin Smith");
    setReferences(
            "Smith, C. R., Won Choi, K., Negrut, D., & Thelen, D. G. (2018)."
            "Efficient computation of cartilage contact pressures within "
            "dynamic "
            "simulations of movement. Computer Methods in Biomechanics and "
            "Biomedical Engineering: Imaging & Visualization, 6(5), 491-498.");

    // Set defaults
    _ray_intersect_distance = -1;
    _ray_intersect_point = -1;
    _ray_intersect_tri = -1;
}

void Smith2018ContactMesh::constructProperties() {
    constructProperty_mesh_file("");
    constructProperty_elastic_modulus(1000000.0);
    constructProperty_poissons_ratio(0.45);
    constructProperty_thickness(0.005);
    constructProperty_use_variable_thickness(false);
    constructProperty_mesh_back_file("");
    constructProperty_min_thickness(0.001);
    constructProperty_max_thickness(0.01);
    constructProperty_thickness_smoothing_iterations(0);
    constructProperty_scale_factors(SimTK::Vec3(1.0));
}

void Smith2018ContactMesh::extendScale(
        const SimTK::State& s, const ScaleSet& scaleSet) {
    Super::extendScale(s, scaleSet);
    SimTK::Vec3 scale_factors = getScaleFactors(
            scaleSet, getConnectee<PhysicalFrame>("scale_frame"));

    set_scale_factors(scale_factors);
    _mesh_is_cached = false;
}

void Smith2018ContactMesh::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();

    if (!_mesh_is_cached) { initializeMesh(); }

    // Create Decorative Mesh
    if (!isObjectUpToDateWithProperties()) {
        // (also when a property of the triangle thickness or material was
        // changed after the mesh was loaded)
        if (get_mesh_file() != _cached_mesh_file ||
            meshDataKey() != _cached_mesh_data_key) { initializeMesh(); }
        //_decorative_mesh.reset(
        //  new SimTK::DecorativeMeshFile(_full_mesh_file_path.c_str()));
        _decorative_mesh.reset(new SimTK::DecorativeMesh(getPolygonalMesh()));
        _decorative_mesh->setScaleFactors(get_scale_factors());
    }
}

void Smith2018ContactMesh::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);

    // Checked here and not in extendFinalizeFromProperties(): a component that
    // throws there cannot be added to a model with Model::addModelComponent()
    // The elastic foundation stiffness (1-v)E/((1+v)(1-2v)) and the pressure
    // depth relationship are not defined for these values
    OPENSIM_THROW_IF_FRMOBJ(!(get_elastic_modulus() > 0.0),
        InvalidPropertyValue, getProperty_elastic_modulus().getName(),
        "elastic_modulus must be greater than 0.");
    OPENSIM_THROW_IF_FRMOBJ(!(get_poissons_ratio() >= 0.0 &&
        get_poissons_ratio() < 0.5), InvalidPropertyValue,
        getProperty_poissons_ratio().getName(),
        "poissons_ratio must be at least 0 and less than 0.5.");
    OPENSIM_THROW_IF_FRMOBJ(!(get_thickness() > 0.0), InvalidPropertyValue,
        getProperty_thickness().getName(), "thickness must be greater than 0.");
}

void Smith2018ContactMesh::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    // Add in a local frame for the mesh reprsenting the resulting translation
    //  and orientation from this component's parent frame

    PhysicalOffsetFrame* mesh_frame =
            new PhysicalOffsetFrame(getFrame(), SimTK::Transform());

    mesh_frame->setName("mesh_frame");
    mesh_frame->set_translation(get_location());
    mesh_frame->set_orientation(get_orientation());

    adoptSubcomponent(mesh_frame);
    setNextSubcomponentInSystem(*mesh_frame);
}

std::string Smith2018ContactMesh::findMeshFile(const std::string& file) {
    /*This is a modified version of the code found in Geometry.cpp
    Mesh::extendFinalizeFromProperties to find geometry files in
    the same directory as the modelDir, modelDir/Geometry, or
    the installDir: OPENSIM_HOME/Geometry

    It plays some games to figure out the modelDir, because the
    Smith2018ContactMesh can't call getModel() at this stage
    */

    // File is a .vtp, .stl, or .obj; attempt to find it.
    const Component* rootModel = nullptr;
    if (!hasOwner()) {
        log_warn(
                "Smith2018ContactMesh {} not connected to a model.", getName());

        return file; // Orphan Mesh not part of a model
    }
    const Component* owner = &getOwner();
    while (owner != nullptr) {
        if (dynamic_cast<const Model*>(owner) != nullptr) {
            rootModel = owner;
            break;
        }
        if (owner->hasOwner())
            owner = &(owner->getOwner()); // traverse up Component tree
        else
            break; // can't traverse up.
    }

    if (rootModel == nullptr) {
        log_warn("Smith2018ContactMesh {} not connected to a model", getName());

        return file; // Orphan Mesh not descendant of a model
    }

    // Current interface to Visualizer calls generateDecorations on every
    // frame. On first time through, load file and create DecorativeMeshFile
    // and cache it so we don't load files from disk during live rendering.
    // const std::string& file = get_mesh_file();
    if (file.empty() || file.compare(PropertyStr::getDefaultStr()) == 0) {
        log_error("No mesh_file property defined in Smith2018ContactMesh: "
                  "{}",
                getName());
        OPENSIM_THROW_FRMOBJ(InvalidPropertyValue,
                getProperty_mesh_file().getName(),
                "The mesh file could not be used (see the error above).");
        // return;  // Return immediately if no file has been specified.
    }

    bool isAbsolutePath;
    std::string directory, fileName, extension;
    SimTK::Pathname::deconstructPathname(
            file, isAbsolutePath, directory, fileName, extension);
    const std::string lowerExtension = SimTK::String::toLower(extension);
    if (lowerExtension != ".vtp" && lowerExtension != ".obj" &&
            lowerExtension != ".stl") {
        log_error("Smith2018ContactMesh file type error: '{}'; "
                  "only .vtp, .stl, and .obj files currently supported.",
                file);
        OPENSIM_THROW_FRMOBJ(InvalidPropertyValue,
                getProperty_mesh_file().getName(),
                "The mesh file could not be used (see the error above).");
        // return;
    }

    SimTK::Array_<std::string> attempts;
    const Model& model = dynamic_cast<const Model&>(*rootModel);
    bool foundIt = ModelVisualizer::findGeometryFile(
            model, file, isAbsolutePath, attempts);

    if (!foundIt) {

        log_error("Couldn't find file '{}'.", file);

        log_error("The following locations were tried:");
        for (unsigned i = 0; i < attempts.size(); ++i) log_error(attempts[i]);

        if (!isAbsolutePath &&
                !SimTK::Pathname::environmentVariableExists("OPENSIM_HOME"))
            log_debug("Set environment variable OPENSIM_HOME to search "
                      "$OPENSIM_HOME/Geometry.");
        OPENSIM_THROW_FRMOBJ(InvalidPropertyValue,
                getProperty_mesh_file().getName(),
                "The mesh file could not be used (see the error above).");
        // return;
    }

    try {
        std::ifstream objFile;
        objFile.open(attempts.back().c_str());
        // objFile closes when destructed
        // if the file can be opened but had bad contents e.g. binary vtp
        // it will be handled downstream
    } catch (const std::exception& e) {
        log_warn("Visualizer couldn't open {} because: {}", attempts.back(),
                e.what());
        OPENSIM_THROW_FRMOBJ(InvalidPropertyValue,
                getProperty_mesh_file().getName(),
                "The mesh file could not be used (see the error above).");
        // return;
    }

    return attempts.back();
}

namespace {

enum class MeshBone { Other, Tibia, Femur, Patella, Meniscus };

std::string lowerCopy(std::string s) {
    for (auto& c : s) { c = (char)std::tolower((unsigned char)c); }
    return s;
}

// Bone a mesh belongs to, from a name, frame path or file name.
MeshBone boneFromText(const std::string& text) {
    const std::string t = lowerCopy(text);
    auto has = [&](const char* s) { return t.find(s) != std::string::npos; };
    if (has("menisc")) { return MeshBone::Meniscus; }
    if (has("patell")) { return MeshBone::Patella; }
    if (has("femur") || has("femor")) { return MeshBone::Femur; }
    if (has("tibi")) { return MeshBone::Tibia; }
    return MeshBone::Other;
}

// True if the text has a stand alone left side token ("_l", "-L", "left").
bool hasLeftToken(const std::string& text) {
    std::string token;
    auto isLeft = [](const std::string& s) { return s == "l" || s == "left"; };
    for (char c : lowerCopy(text)) {
        if (std::isalnum((unsigned char)c)) {
            token += c;
        } else {
            if (isLeft(token)) { return true; }
            token.clear();
        }
    }
    return isLeft(token);
}

} // namespace

void Smith2018ContactMesh::computeRegions() {
    /* Divide the triangles into regions used by the regional contact outputs.
    The bone is taken from the mesh name, the frame it is attached to, or the
    mesh file name (in that order), and the mesh is assumed to be expressed in
    the knee frame (x anterior, y proximal, z lateral for a right knee, origin
    at the knee joint center):

    Tibia:    medial/lateral (z), then each compartment in an inner (towards
              the midline) and outer half of its own z range, then in thirds of
              its own x range (anterior, central, posterior): 12 regions.
              Followed by 10 more regions that overlap the first 12, a
              central/anterior/posterior/internal/external division of each
              compartment (see below), so a triangle belongs to two regions.
              The 5 regions of each compartment: an ellipse around the center
              of the compartment (40% of its x and z extent) is "central" and
              the rest is divided by the diagonals of the compartment extent
              into anterior, posterior, internal (towards the midline) and
              external.
    Femur:    medial/lateral (z), then the angle in the sagittal plane about
              the origin, from +y towards posterior: 0-135 posterior,
              135-225 central, 225-360 anterior, then every angle (swept in 5 degree
              bins) is split in an inner (towards the midline) and outer half
              of the z range found at that angle: 12 regions.
    Patella:  medial/lateral (z), then each compartment in thirds of its own
              y range (proximal, central, distal).
    Meniscus: angle in the transverse plane about the center of the meniscus
              (circle fit, close to the center of the related tibial
              compartment), 120 degree slices: central (+-60 degrees about
              the direction away from the midline), anterior and posterior.
    Other:    half spaces of the x, y, z axes.
    */
    const int nTri = _mesh.getNumFaces();
    const double rad2deg = 180.0 / SimTK::Pi;

    std::string frame_path;
    try {
        frame_path = getSocket<PhysicalFrame>("frame").getConnecteePath();
    } catch (const std::exception&) {
        frame_path.clear();
    }
    std::string file_name = _full_mesh_file_path;
    file_name = file_name.substr(file_name.find_last_of("/\\") + 1);

    MeshBone bone = boneFromText(getName());
    if (bone == MeshBone::Other) { bone = boneFromText(frame_path); }
    if (bone == MeshBone::Other) { bone = boneFromText(file_name); }

    // Sign of z on the lateral side (a left knee is the mirror image)
    const bool left = hasLeftToken(getName()) || hasLeftToken(frame_path);
    const double lateral_sign = left ? -1.0 : 1.0;

    auto setRegions = [&](const std::vector<std::string>& names) {
        _regional_names = names;
        _regional_tri_ind.assign(names.size(), std::vector<int>());
        _regional_n_tri.assign(names.size(), 0);
    };
    auto assign = [&](int tri, int region) {
        _regional_tri_ind[region].push_back(tri);
        _regional_n_tri[region]++;
    };

    // Medial/lateral, then (optionally) the inner/outer half of the
    // compartment in z, then thirds of the compartment range along one axis
    auto splitInThirds = [&](int axis, const std::string& high,
                                 const std::string& mid,
                                 const std::string& low, bool halves) {
        const int nPart = halves ? 6 : 3;
        std::vector<std::string> names;
        for (const char* side : {"medial_", "lateral_"}) {
            for (const char* half : (halves ?
                    std::vector<const char*>{"inner_", "outer_"} :
                    std::vector<const char*>{""})) {
                for (const std::string& t : {high, mid, low}) {
                    names.push_back(std::string(side) + half + t);
                }
            }
        }
        setRegions(names);
        double zlo[2] = {SimTK::Infinity, SimTK::Infinity};
        double zhi[2] = {-SimTK::Infinity, -SimTK::Infinity};
        double lo[2] = {SimTK::Infinity, SimTK::Infinity};
        double hi[2] = {-SimTK::Infinity, -SimTK::Infinity};
        std::vector<int> side(nTri);
        for (int i = 0; i < nTri; ++i) {
            side[i] = (_tri_center(i)(2) * lateral_sign > 0.0) ? 1 : 0;
            lo[side[i]] = std::min(lo[side[i]], _tri_center(i)(axis));
            hi[side[i]] = std::max(hi[side[i]], _tri_center(i)(axis));
            const double z = _tri_center(i)(2) * lateral_sign;
            zlo[side[i]] = std::min(zlo[side[i]], z);
            zhi[side[i]] = std::max(zhi[side[i]], z);
        }
        for (int i = 0; i < nTri; ++i) {
            const int c = side[i];
            int third = 1;
            if (hi[c] > lo[c]) {
                third = (int)(3.0 * (_tri_center(i)(axis) - lo[c]) /
                              (hi[c] - lo[c]));
                third = std::min(std::max(third, 0), 2);
            }
            int half = 0;
            if (halves) {
                const double z = _tri_center(i)(2) * lateral_sign;
                const bool towardsMidline = (c == 1) ?
                    (z < 0.5 * (zlo[c] + zhi[c])) :
                    (z > 0.5 * (zlo[c] + zhi[c]));
                half = towardsMidline ? 0 : 1;
            }
            assign(i, c * nPart + half * 3 + (2 - third));
        }
    };

    if (bone == MeshBone::Tibia) {
        splitInThirds(0, "anterior", "central", "posterior", true);

        // Additional overlapping regions: central disc and four sectors
        const char* sector_names[5] = {"central_disc", "anterior_sector",
                "posterior_sector", "internal_sector", "external_sector"};
        const int first = (int)_regional_names.size();
        for (const char* side : {"medial", "lateral"}) {
            for (const char* sector : sector_names) {
                _regional_names.push_back(std::string(side) + "_" + sector);
            }
        }
        _regional_tri_ind.resize(first + 10);
        _regional_n_tri.resize(first + 10, 0);
        double xlo[2] = {SimTK::Infinity, SimTK::Infinity};
        double xhi[2] = {-SimTK::Infinity, -SimTK::Infinity};
        double zlo[2] = {SimTK::Infinity, SimTK::Infinity};
        double zhi[2] = {-SimTK::Infinity, -SimTK::Infinity};
        std::vector<int> side(nTri);
        for (int i = 0; i < nTri; ++i) {
            const double x = _tri_center(i)(0);
            const double z = _tri_center(i)(2) * lateral_sign;
            side[i] = (z > 0.0) ? 1 : 0;
            xlo[side[i]] = std::min(xlo[side[i]], x);
            xhi[side[i]] = std::max(xhi[side[i]], x);
            zlo[side[i]] = std::min(zlo[side[i]], z);
            zhi[side[i]] = std::max(zhi[side[i]], z);
        }
        for (int i = 0; i < nTri; ++i) {
            const int c = side[i];
            const double hx = 0.5 * (xhi[c] - xlo[c]);
            const double hz = 0.5 * (zhi[c] - zlo[c]);
            if (hx <= 0.0 || hz <= 0.0) { continue; }
            const double u = (_tri_center(i)(0) - 0.5 * (xlo[c] + xhi[c])) / hx;
            double v = (_tri_center(i)(2) * lateral_sign -
                        0.5 * (zlo[c] + zhi[c])) / hz;
            // positive towards the midline
            const double towards = (c == 1) ? -v : v;
            int sector;
            if (std::sqrt(u * u + v * v) < 0.4) {
                sector = 0;
            } else if (std::abs(u) >= std::abs(towards)) {
                sector = (u > 0.0) ? 1 : 2;
            } else {
                sector = (towards > 0.0) ? 3 : 4;
            }
            assign(i, first + c * 5 + sector);
        }

    } else if (bone == MeshBone::Patella) {
        splitInThirds(1, "proximal", "central", "distal", false);

    } else if (bone == MeshBone::Femur) {
        setRegions({"medial_inner_anterior", "medial_inner_central",
                "medial_inner_posterior", "medial_outer_anterior",
                "medial_outer_central", "medial_outer_posterior",
                "lateral_inner_anterior", "lateral_inner_central",
                "lateral_inner_posterior", "lateral_outer_anterior",
                "lateral_outer_central", "lateral_outer_posterior"});
        // Inner/outer halves: sweep the angle in 5 degree bins and, in every
        // bin of each compartment, split the z range of that bin in half
        const int nBins = 72;
        std::vector<int> bin(nTri);
        std::vector<double> zlo(2 * nBins, SimTK::Infinity);
        std::vector<double> zhi(2 * nBins, -SimTK::Infinity);
        for (int i = 0; i < nTri; ++i) {
            const double z = _tri_center(i)(2) * lateral_sign;
            // angle from +y (proximal) towards -x (posterior)
            double theta =
                    std::atan2(-_tri_center(i)(0), _tri_center(i)(1)) * rad2deg;
            if (theta < 0.0) { theta += 360.0; }
            const int b = std::min((int)(theta / (360.0 / nBins)), nBins - 1);
            bin[i] = ((z > 0.0) ? nBins : 0) + b;
            zlo[bin[i]] = std::min(zlo[bin[i]], z);
            zhi[bin[i]] = std::max(zhi[bin[i]], z);
        }
        for (int i = 0; i < nTri; ++i) {
            const int k = bin[i];
            const int c = k / nBins;
            const double theta_bin = (k % nBins + 0.5) * (360.0 / nBins);
            const int part = (theta_bin < 135.0) ? 2 : ((theta_bin < 225.0) ? 1 : 0);
            const double z = _tri_center(i)(2) * lateral_sign;
            const double zmid = 0.5 * (zlo[k] + zhi[k]);
            const bool towardsMidline = (c == 1) ? (z < zmid) : (z > zmid);
            assign(i, c * 6 + (towardsMidline ? 0 : 3) + part);
        }

    } else if (bone == MeshBone::Meniscus) {
        setRegions({"anterior", "central", "posterior"});

        // Circle fit (Kasa) of the triangle centers in the x-z plane
        double mx = 0.0, mz = 0.0;
        for (int i = 0; i < nTri; ++i) {
            mx += _tri_center(i)(0);
            mz += _tri_center(i)(2);
        }
        mx /= std::max(nTri, 1);
        mz /= std::max(nTri, 1);
        double suu = 0, svv = 0, suv = 0, suuu = 0, svvv = 0, suvv = 0,
               suuv = 0;
        for (int i = 0; i < nTri; ++i) {
            const double u = _tri_center(i)(0) - mx;
            const double v = _tri_center(i)(2) - mz;
            suu += u * u;
            svv += v * v;
            suv += u * v;
            suuu += u * u * u;
            svvv += v * v * v;
            suvv += u * v * v;
            suuv += u * u * v;
        }
        double cx = mx, cz = mz;
        const double det = suu * svv - suv * suv;
        if (std::abs(det) > 1e-30) {
            const double uc =
                    0.5 * ((suuu + suvv) * svv - (svvv + suuv) * suv) / det;
            const double vc =
                    0.5 * ((svvv + suuv) * suu - (suuu + suvv) * suv) / det;
            cx = mx + uc;
            cz = mz + vc;
        }

        // Outward is the side of z=0 on which the meniscus center is located
        const double outward = (cz >= 0.0) ? 1.0 : -1.0;
        for (int i = 0; i < nTri; ++i) {
            const double anterior = _tri_center(i)(0) - cx;
            const double lateral = outward * (_tri_center(i)(2) - cz);
            const double psi = std::atan2(anterior, lateral) * rad2deg;
            assign(i, (psi > 60.0) ? 0 : ((psi < -60.0) ? 2 : 1));
        }

    } else {
        setRegions({"x_negative", "x_positive", "y_negative", "y_positive",
                "z_negative", "z_positive"});
        for (int i = 0; i < nTri; ++i) {
            for (int j = 0; j < 3; ++j) {
                assign(i, j * 2 + ((_tri_center(i)(j) < 0.0) ? 0 : 1));
            }
        }
    }
}

std::string Smith2018ContactMesh::meshDataKey() const {
    std::ostringstream key;
    key.precision(17);
    key << get_use_variable_thickness() << '|' << get_thickness() << '|'
        << get_elastic_modulus() << '|' << get_poissons_ratio() << '|'
        << get_thickness_smoothing_iterations() << '|'
        << get_scale_factors() << '|';
    if (get_use_variable_thickness()) {
        key << get_mesh_back_file() << '|' << get_min_thickness() << '|'
            << get_max_thickness();
    }
    return key.str();
}

void Smith2018ContactMesh::initializeMesh() {
    _mesh_is_cached = true;
    _cached_mesh_file = get_mesh_file();
    _cached_mesh_data_key = meshDataKey();
    _mesh.clear();
    _mesh_back.clear();

    _obb = OBBTreeNode();
    _back_obb = OBBTreeNode();

    // Load Mesh from file
    if (_init_mesh_from_file) {
        _full_mesh_file_path = findMeshFile(get_mesh_file());
        _mesh.loadFile(_full_mesh_file_path);

        //_vertex_locations set below
        _num_vertices = _mesh.getNumVertices();
        _num_faces = _mesh.getNumFaces();

        // (a mesh that is loaded again must not keep the old faces)
        _faces.clear();
        for (int f = 0; f < _num_faces; ++f) {
            std::vector<int> face_vertex(3);
            for (int j = 0; j < 3; ++j) {
                face_vertex[j] = _mesh.getFaceVertex(f, j);
            }
            _faces.push_back(face_vertex);
        }

    } else {
        // initialize from tris and vertices passed into constructor
        for (int v = 0; v < _num_vertices; ++v) {
            _mesh.addVertex(_vertex_locations[v]);
        }

        for (int t = 0; t < _num_faces; ++t) {
            SimTK::Array_<int> face(_faces[t]);
            _mesh.addFace(face);
        }
    }
    // Scale Mesh
    SimTK::Real xscale = get_scale_factors()(0);
    SimTK::Real yscale = get_scale_factors()(1);
    SimTK::Real zscale = get_scale_factors()(2);

    SimTK::Rotation scale_rot;
    scale_rot.set(0, 0, xscale);
    scale_rot.set(1, 1, yscale);
    scale_rot.set(2, 2, zscale);
    SimTK::Transform scale_transform(scale_rot, SimTK::Vec3(0.0));
    _mesh.transformMesh(scale_transform);

    // Allocate space
    _tri_center.resize(_mesh.getNumFaces());
    _tri_normal.resize(_mesh.getNumFaces());
    _tri_area.resize(_mesh.getNumFaces());
    _tri_thickness.resize(_mesh.getNumFaces());
    _tri_elastic_modulus.resize(_mesh.getNumFaces());
    _tri_poissons_ratio.resize(_mesh.getNumFaces());

    _vertex_locations.resize(_mesh.getNumVertices());
    _face_vertex_locations.resize(_mesh.getNumFaces(), 3);

    // Compute Mesh Properties
    //========================

    for (int i = 0; i < _mesh.getNumFaces(); ++i) {

        // Get Triangle Vertice Positions
        int v1_i = _mesh.getFaceVertex(i, 0);
        int v2_i = _mesh.getFaceVertex(i, 1);
        int v3_i = _mesh.getFaceVertex(i, 2);

        SimTK::Vec3 v1 = _mesh.getVertexPosition(v1_i);
        SimTK::Vec3 v2 = _mesh.getVertexPosition(v2_i);
        SimTK::Vec3 v3 = _mesh.getVertexPosition(v3_i);

        // Degenerate triangles give zero area and undefined normals
        // (adopted from upstream opensim-jam-org/opensim-core 1f90450c6).
        OPENSIM_THROW_IF_FRMOBJ(v1 == v2 || v2 == v3 || v3 == v1, Exception,
            "Triangle (0-index) " + std::to_string(i) +
            " has overlapping vertices.");

        // Compute Triangle Center
        _tri_center(i) = (v1 + v2 + v3) / 3.0;

        // Compute Triangle Normal
        SimTK::Vec3 e1 = v3 - v1;
        SimTK::Vec3 e2 = v2 - v1;

        SimTK::Vec3 cross = SimTK::cross(e1, e2);

        double mag = cross.norm();

        for (int j = 0; j < 3; ++j) {
            _tri_normal(i).set(j, -cross[j] / mag);
        }

        // Compute Triangle Area
        // Compute length of each side of the triangle
        double s1 = 0;
        double s2 = 0;
        double s3 = 0;

        for (int j = 0; j < 3; j++) {
            s1 += (v2[j] - v1[j]) * (v2[j] - v1[j]);
            s2 += (v3[j] - v2[j]) * (v3[j] - v2[j]);
            s3 += (v1[j] - v3[j]) * (v1[j] - v3[j]);
        }

        s1 = sqrt(s1);
        s2 = sqrt(s2);
        s3 = sqrt(s3);

        // Now employ Heron's formula
        double s = (s1 + s2 + s3) / 2.0;
        _tri_area[i] = sqrt(s * (s - s1) * (s - s2) * (s - s3));
    }

    // Regional triangle indices (needs the triangle centers)
    computeRegions();

    // Vertex Locations
    for (int i = 0; i < _mesh.getNumVertices(); ++i) {
        _vertex_locations(i) = _mesh.getVertexPosition(i);
    }

    // Face Vertex Locations
    for (int i = 0; i < _mesh.getNumFaces(); ++i) {
        for (int j = 0; j < 3; ++j) {
            int v_ind = _mesh.getFaceVertex(i, j);
            _face_vertex_locations(i, j) = _mesh.getVertexPosition(v_ind);
        }
    }

    // Vertex Connectivity
    std::vector<std::vector<int>> ver_tri_ind(_mesh.getNumVertices());
    std::vector<int> ver_nTri(_mesh.getNumVertices());

    for (int i = 0; i < _mesh.getNumFaces(); ++i) {
        for (int j = 0; j < 3; ++j) {
            int ver = _mesh.getFaceVertex(i, j);
            ver_tri_ind[ver].push_back(i);
            ver_nTri[ver]++;
        }
    }

    // Triangle Neighbors
    _tri_neighbors.resize(_mesh.getNumFaces());

    for (int i = 0; i < _mesh.getNumFaces(); ++i) {
        for (int j = 0; j < 3; ++j) {

            int ver = _mesh.getFaceVertex(i, j);

            for (int k = 0; k < ver_nTri[ver]; ++k) {
                int tri = ver_tri_ind[ver][k];

                // triange can't be neighbor with itself
                if (tri == i) { continue; }
                _tri_neighbors[i].insert(tri);
            }
        }
    }

    // Construct the OBB Tree
    SimTK::Array_<int> allFaces(_mesh.getNumFaces());
    for (int i = 0; i < _mesh.getNumFaces(); ++i) { allFaces[i] = i; }

    createObbTree(_obb, _mesh, allFaces);

    // Triangle Material Properties
    if (get_use_variable_thickness()) {
        computeVariableThickness();
    } else {
        _tri_thickness = get_thickness();
    }

    _tri_elastic_modulus = get_elastic_modulus();
    _tri_poissons_ratio = get_poissons_ratio();
}

void Smith2018ContactMesh::computeVariableThickness() {

    // Get Mesh Properties
    double min_thickness = get_min_thickness();
    double max_thickness = get_max_thickness();

    // Load mesh_back_file
    std::string file = findMeshFile(get_mesh_back_file());
    _mesh_back.loadFile(file);

    // Scale _mesh_back
    SimTK::Real xscale = get_scale_factors()(0);
    SimTK::Real yscale = get_scale_factors()(1);
    SimTK::Real zscale = get_scale_factors()(2);

    SimTK::Rotation scale_rot;
    scale_rot.set(0, 0, xscale);
    scale_rot.set(1, 1, yscale);
    scale_rot.set(2, 2, zscale);
    SimTK::Transform scale_transform(scale_rot, SimTK::Vec3(0.0));
    _mesh_back.transformMesh(scale_transform);

    // Create OBB tree for back mesh
    SimTK::Array_<int> allFaces(_mesh_back.getNumFaces());

    for (int i = 0; i < _mesh_back.getNumFaces(); ++i) { allFaces[i] = i; }

    createObbTree(_back_obb, _mesh_back, allFaces);

    // Loop through all triangles in cartilage mesh
    for (int i = 0; i < _mesh.getNumFaces(); ++i) {

        // Use mesh_back OBB tree to find cartilage thickness
        //--------------------------------------------------

        int tri;
        SimTK::Vec3 intersection_point;
        double depth = 0.0;

        if (_back_obb.rayIntersectOBB(_mesh_back, _tri_center(i),
                    -_tri_normal(i), tri, intersection_point, depth)) {

            if (depth < min_thickness) {
                depth = min_thickness;
            } else if (depth > max_thickness) {
                depth = min_thickness;
            }
        } else { // Normal from mesh missed mesh back
            depth = min_thickness;
        }
        _tri_thickness(i) = depth;
    }

    // Smooth the thickness over the neighboring triangles
    const int nTri = _mesh.getNumFaces();
    for (int it = 0; it < get_thickness_smoothing_iterations(); ++it) {
        SimTK::Vector smoothed(nTri);
        for (int i = 0; i < nTri; ++i) {
            double sum = _tri_thickness(i);
            int n = 1;
            for (int nb : _tri_neighbors[i]) {
                sum += _tri_thickness(nb);
                ++n;
            }
            smoothed(i) = sum / n;
        }
        _tri_thickness = smoothed;
    }
}

void Smith2018ContactMesh::generateDecorations(bool fixed,
        const ModelDisplayHints& hints, const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const {
    Super::generateDecorations(fixed, hints, state, geometry);

    // There is no fixed geometry to generate here.
    if (fixed) { return; }

    if (!get_Appearance().get_visible()) return;
    if (!hints.get_show_contact_geometry()) return;

    if (_decorative_mesh.get() != nullptr) {
        try {
            // Force the loading of the mesh to see if it has bad contents
            // (e.g., binary vtp).
            // We do not want to do this in extendFinalizeFromProperties b/c
            // it's expensive to repeatedly load meshes.
            _decorative_mesh->getMesh();
        } catch (const std::exception& e) {
            log_warn("Visualizer couldn't open {} because: {}", get_mesh_file(),
                    e.what());
            // No longer try to visualize this mesh.
            _decorative_mesh.reset();
            return;
        }
    }
    //_decorative_mesh->setScaleFactors(get_scale_factors());

    // SET TRANSFORM
    //  B: base Frame (Body or Ground)
    //  F: PhysicalFrame that this ContactGeometry is connected to
    //  P: the frame defined (relative to F) by the location and orientation
    //     properties.

    const auto& X_BF = getFrame().findTransformInBaseFrame();
    const auto& X_FP = getTransform();
    const auto X_BP = X_BF * X_FP;

    _decorative_mesh->setTransform(X_BP);
    _decorative_mesh->setBodyId(getFrame().getMobilizedBodyIndex());

    // SET APPERANCE
    _decorative_mesh->setColor(get_Appearance().get_color());
    _decorative_mesh->setOpacity(get_Appearance().get_opacity());

    if (get_Appearance().get_visible()) {
        _decorative_mesh->setRepresentation(
                (VisualRepresentation)get_Appearance().get_representation());
    } else {
        _decorative_mesh->setRepresentation(SimTK::DecorativeGeometry::Hide);
    }

    geometry.push_back(*_decorative_mesh);
}

void Smith2018ContactMesh::createObbTree(OBBTreeNode& node,
        const SimTK::PolygonalMesh& mesh,
        const SimTK::Array_<int>&
                faceIndices) { // Find all vertices in the node and build the
                               // OrientedBoundingBox.
    node._numTriangles = faceIndices.size();

    set<int> vertexIndices;
    for (int i = 0; i < (int)faceIndices.size(); i++) {
        for (int j = 0; j < 3; j++) {
            vertexIndices.insert(mesh.getFaceVertex(faceIndices[i], j));
        }
    }
    SimTK::Vector_<SimTK::Vec3> points((int)vertexIndices.size());
    int index = 0;
    for (set<int>::iterator iter = vertexIndices.begin();
            iter != vertexIndices.end(); ++iter) {
        points[index++] = mesh.getVertexPosition(*iter);
    }
    node._bounds = SimTK::OrientedBoundingBox(points);
    if (faceIndices.size() > 3) {

        // Order the axes by size.

        int axisOrder[3];
        const SimTK::Vec3& size = node._bounds.getSize();
        if (size[0] > size[1]) {
            if (size[0] > size[2]) {
                axisOrder[0] = 0;
                if (size[1] > size[2]) {
                    axisOrder[1] = 1;
                    axisOrder[2] = 2;
                } else {
                    axisOrder[1] = 2;
                    axisOrder[2] = 1;
                }
            } else {
                axisOrder[0] = 2;
                axisOrder[1] = 0;
                axisOrder[2] = 1;
            }
        } else if (size[0] > size[2]) {
            axisOrder[0] = 1;
            axisOrder[1] = 0;
            axisOrder[2] = 2;
        } else {
            if (size[1] > size[2]) {
                axisOrder[0] = 1;
                axisOrder[1] = 2;
            } else {
                axisOrder[0] = 2;
                axisOrder[1] = 1;
            }
            axisOrder[2] = 0;
        }

        // Try splitting along each axis.

        for (int i = 0; i < 3; i++) {
            SimTK::Array_<int> child1Indices, child2Indices;
            splitObbAxis(mesh, faceIndices, child1Indices, child2Indices,
                    axisOrder[i]);
            if (child1Indices.size() > 0 && child2Indices.size() > 0) {
                // It was successfully split, so create the child nodes.

                node._child1 = new OBBTreeNode();
                node._child2 = new OBBTreeNode();
                createObbTree(*node._child1, mesh, child1Indices);
                createObbTree(*node._child2, mesh, child2Indices);
                return;
            }
        }
    }

    // This is a leaf node
    node._triangles.insert(
            node._triangles.begin(), faceIndices.begin(), faceIndices.end());
}

void Smith2018ContactMesh::splitObbAxis(const SimTK::PolygonalMesh& mesh,
        const SimTK::Array_<int>& parentIndices,
        SimTK::Array_<int>& child1Indices, SimTK::Array_<int>& child2Indices,
        int axis) { // For each face, find its minimum and maximum extent along
                    // the axis.
    SimTK::Vector minExtent(parentIndices.size());
    SimTK::Vector maxExtent(parentIndices.size());
    SimTK::Array_<int> vertexIndices(3);
    for (int i = 0; i < (int)parentIndices.size(); i++) {
        for (int j = 0; j < 3; j++) {
            vertexIndices[j] = mesh.getFaceVertex(parentIndices[i], j);
        }
        SimTK::Real minVal = mesh.getVertexPosition(vertexIndices[0])(axis);
        SimTK::Real maxVal = mesh.getVertexPosition(vertexIndices[0])(axis);
        minVal = std::min(
                minVal, mesh.getVertexPosition(vertexIndices[1])(axis));
        maxVal = std::max(
                maxVal, mesh.getVertexPosition(vertexIndices[1])(axis));
        minExtent[i] = std::min(
                minVal, mesh.getVertexPosition(vertexIndices[2])(axis));
        maxExtent[i] = std::max(
                maxVal, mesh.getVertexPosition(vertexIndices[2])(axis));
    }

    // Select a split point that tries to put as many faces as possible
    // entirely on one side or the other.

    SimTK::Real split = (median(minExtent) + median(maxExtent)) / 2;

    // Choose a side for each face.

    for (int i = 0; i < (int)parentIndices.size(); i++) {
        if (maxExtent[i] <= split)
            child1Indices.push_back(parentIndices[i]);
        else if (minExtent[i] >= split)
            child2Indices.push_back(parentIndices[i]);
        else if (0.5 * (minExtent[i] + maxExtent[i]) <= split)
            child1Indices.push_back(parentIndices[i]);
        else
            child2Indices.push_back(parentIndices[i]);
    }
}

bool Smith2018ContactMesh::rayIntersectionTest(const SimTK::Vec3& origin,
        const SimTK::UnitVec3& direction, const double& min_proximity,
        const double& max_proximity) {

    double obb_distance = -1;
    SimTK::Array_<int> obb_triangles;
    bool intersection_dectected = false;

    // dummy variables
    int tri;
    SimTK::Vec3 intersection_point;
    SimTK::Real distance;

    if (_obb.rayIntersectOBB(
                _mesh, origin, direction, tri, intersection_point, distance)) {

        if ((distance > min_proximity) && (distance < max_proximity)) {
            intersection_dectected = true;
        }
    }

    // Shoot the ray in the opposite direction
    if (min_proximity < 0.0) {
        if (_obb.rayIntersectOBB(_mesh, origin, -direction, tri,
                    intersection_point, distance)) {

            distance = -distance;
            if ((distance > min_proximity) && (distance < max_proximity)) {
                intersection_dectected = true;
            }
        }
    }

    // ray didn't intersect
    if (intersection_dectected == false) {
        distance = -1;
        intersection_point = -1;
        tri = -1;
    }

    // store values (only for matlab and python access)
    _ray_intersect_distance = distance;
    _ray_intersect_point = intersection_point;
    _ray_intersect_tri = tri;

    return intersection_dectected;
}

bool Smith2018ContactMesh::rayIntersectMesh(const SimTK::Vec3& origin,
        const SimTK::UnitVec3& direction, const double& min_proximity,
        const double& max_proximity, int& tri, SimTK::Vec3& intersection_point,
        SimTK::Real& distance) const {

    double obb_distance = -1;
    SimTK::Array_<int> obb_triangles;

    if (_obb.rayIntersectOBB(
                _mesh, origin, direction, tri, intersection_point, distance)) {

        if ((distance > min_proximity) && (distance < max_proximity)) {
            return true;
        }
    }

    // Shoot the ray in the opposite direction
    if (min_proximity < 0.0) {
        if (_obb.rayIntersectOBB(_mesh, origin, -direction, tri,
                    intersection_point, distance)) {

            distance = -distance;
            if ((distance > min_proximity) && (distance < max_proximity)) {
                return true;
            }
        }
    }

    // ray didn't intersect
    distance = -1;
    intersection_point = -1;
    tri = -1;

    return false;
}

void Smith2018ContactMesh::printMeshDebugInfo() const {

    log_trace("Mesh Properties: {}", getName());
    log_trace("{:<10} {:<15} {:<15} {:<15} {:<15} {:<35} {:<35}", "Tri #",
            "Area", "Thickness", "Elastic Modulus", "Poissons Ratio", "Center",
            "Normal");

    const SimTK::Vector& area = getTriangleAreas();
    const SimTK::Vector_<SimTK::Vec3>& center = getTriangleCenters();
    const SimTK::Vector_<SimTK::UnitVec3>& normal = getTriangleNormals();

    for (int i = 0; i < getNumFaces(); i++) {
        double t = getTriangleThickness(i);
        double E = getTriangleElasticModulus(i);
        double v = getTrianglePoissonsRatio(i);
        double a = area(i);
        SimTK::Vec3 c = center(i);
        SimTK::Vec3 n = normal(i);

        log_trace("{:<10} {:<15} {:<15} {:<15} {:<15} {:<35} {:<35}", i, a, t,
                E, v, c.toString(), n.toString());
    }
}

//=============================================================================
//               Smith2018ContactMesh :: OBBTreeNode
//=============================================================================
Smith2018ContactMesh::OBBTreeNode::OBBTreeNode(const OBBTreeNode& copy)
        : _bounds(copy._bounds), _triangles(copy._triangles),
          _numTriangles(copy._numTriangles) {
    if (copy._child1 == NULL) {
        _child1 = NULL;
        _child2 = NULL;
    } else {
        _child1 = new OBBTreeNode(*copy._child1);
        _child2 = new OBBTreeNode(*copy._child2);
    }
}

Smith2018ContactMesh::OBBTreeNode& Smith2018ContactMesh::OBBTreeNode::operator=(const OBBTreeNode& copy)
{
    _bounds = copy._bounds;
    _triangles = copy._triangles;
    _numTriangles = copy._numTriangles;
    if (copy._child1 == NULL) {
        _child1 = NULL;
        _child2 = NULL;
    }
    else {
        _child1 = new OBBTreeNode(*copy._child1);
        _child2 = new OBBTreeNode(*copy._child2);
    }
    return *this;
}

Smith2018ContactMesh::OBBTreeNode::~OBBTreeNode() {
    if (_child1 != NULL) delete _child1;
    if (_child2 != NULL) delete _child2;
}

bool Smith2018ContactMesh::OBBTreeNode::rayIntersectOBB(
        const SimTK::PolygonalMesh& mesh, const SimTK::Vec3& origin,
        const SimTK::UnitVec3& direction, int& tri_index,
        SimTK::Vec3& intersection_point, double& distance) const {

    if (_child1 != NULL) {
        // Recursively check the child nodes. Each child keeps its own hit
        // (triangle, point and distance), so that the triangle and point
        // returned are those of the closer hit.
        SimTK::Real child1distance, child2distance;
        bool child1intersects = _child1->_bounds.intersectsRay(
                origin, direction, child1distance);
        bool child2intersects = _child2->_bounds.intersectsRay(
                origin, direction, child2distance);

        int tri1 = -1;
        int tri2 = -1;
        SimTK::Vec3 point1(0);
        SimTK::Vec3 point2(0);
        double hit1 = SimTK::Infinity;
        double hit2 = SimTK::Infinity;
        bool found1 = false;
        bool found2 = false;

        if (child1intersects && child2intersects) {
            // The ray intersects both child nodes. First check the closer
            // one, then the other only if its box starts before the hit.
            if (child1distance < child2distance) {
                found1 = _child1->rayIntersectOBB(
                        mesh, origin, direction, tri1, point1, hit1);
                if (!found1 || child2distance < hit1) {
                    found2 = _child2->rayIntersectOBB(
                            mesh, origin, direction, tri2, point2, hit2);
                }
            } else {
                found2 = _child2->rayIntersectOBB(
                        mesh, origin, direction, tri2, point2, hit2);
                if (!found2 || child1distance < hit2) {
                    found1 = _child1->rayIntersectOBB(
                            mesh, origin, direction, tri1, point1, hit1);
                }
            }
        } else if (child1intersects) {
            found1 = _child1->rayIntersectOBB(
                    mesh, origin, direction, tri1, point1, hit1);
        } else if (child2intersects) {
            found2 = _child2->rayIntersectOBB(
                    mesh, origin, direction, tri2, point2, hit2);
        }

        // Return the closer hit
        if (found1 && (!found2 || hit1 <= hit2)) {
            tri_index = tri1;
            intersection_point = point1;
            distance = hit1;
            return true;
        }
        if (found2) {
            tri_index = tri2;
            intersection_point = point2;
            distance = hit2;
            return true;
        }
        return false;
    }

    // Reached a leaf node: return the nearest hit in front of the origin
    // among all containing triangles
    bool found = false;
    int best_tri = -1;
    SimTK::Vec3 best_point(0);
    double best_distance = SimTK::Infinity;
    for (int i = 0; i < (int)_triangles.size(); i++) {
        SimTK::Vec3 point;
        double d;
        if (rayIntersectTri(mesh, origin, direction, _triangles[i], point, d) &&
                d >= 0.0 && d < best_distance) {
            found = true;
            best_tri = _triangles[i];
            best_point = point;
            best_distance = d;
        }
    }
    if (found) {
        tri_index = best_tri;
        intersection_point = best_point;
        distance = best_distance;
    }
    return found;
}

bool Smith2018ContactMesh::OBBTreeNode::rayIntersectTri(
        const SimTK::PolygonalMesh& mesh, SimTK::Vec3 origin,
        SimTK::Vec3 direction, int tri_index, SimTK::Vec3& intersection_pt,
        double& distance) const {

    /*
    origin - reference point of casting ray
            (i.e. center of triangle from which ray is cast)

    direction - casting ray direction vector
                (i.e. normal to triangle from which ray is cast)

    tri_index - The triangle index of the test target

    www.lighthouse3d.com/tutorials/maths/ray-triangle-intersection/
    */
    int v0_ind = mesh.getFaceVertex(tri_index, 0);
    int v1_ind = mesh.getFaceVertex(tri_index, 1);
    int v2_ind = mesh.getFaceVertex(tri_index, 2);

    SimTK::Vec3 v0 = mesh.getVertexPosition(v0_ind);
    SimTK::Vec3 v1 = mesh.getVertexPosition(v1_ind);
    SimTK::Vec3 v2 = mesh.getVertexPosition(v2_ind);

    SimTK::Vec3 e1, e2, h, s, q;
    double a, f, u, v;

    // find triangle edges
    for (int i = 0; i < 3; ++i) {
        e1(i) = v1(i) - v0(i);
        e2(i) = v2(i) - v0(i);
    }

    h = SimTK::cross(direction, e2);
    a = dot(e1, h);

    // If the ray is parallel to the triangle plane, there is no intersection.
    // a = 2 * (triangle area) * cos(angle between ray and triangle normal),
    // so the tolerance is relative to the size of the triangle (an absolute
    // tolerance would reject more grazing rays on finer meshes).
    if (std::abs(a) < 1e-6 * e1.norm() * e2.norm()) {
        return (false);
    }

    // Else on to second test
    // find triangle edges
    f = 1 / a;
    for (int i = 0; i < 3; i++) { s(i) = origin(i) - v0(i); }

    u = f * SimTK::dot(s, h);
    if (u < 0 || u > 1.0) return (false);

    q = SimTK::cross(s, e1);

    v = f * SimTK::dot(direction, q);
    double w = 1 - u - v;

    if (v < 0.0 || w < 0.0)
        return (false);

    else {
        // else there is a line intersection
        // at this stage we can compute the distance to the intersection
        // point on the line
        //     point(t) = p + t * d
        //  where
        //      p is a point in the line
        //      d is a vector that provides the line's direction
        //      t is the distance
        intersection_pt = w * v0 + u * v1 + v * v2;
        distance = f * SimTK::dot(e2, q);
        return (true);
    }
}

const SimTK::OrientedBoundingBox&
Smith2018ContactMesh::OBBTreeNode::getBounds() const {
    return _bounds;
}

bool Smith2018ContactMesh::OBBTreeNode::isLeafNode() const {
    return (_child1 == NULL);
}

const Smith2018ContactMesh::OBBTreeNode
Smith2018ContactMesh::OBBTreeNode::getFirstChildNode() const {
    SimTK_ASSERT_ALWAYS(_child1, "Called getFirstChildNode() on a leaf node");
    return OBBTreeNode(*_child1);
}

const Smith2018ContactMesh::OBBTreeNode
Smith2018ContactMesh::OBBTreeNode::getSecondChildNode() const {
    SimTK_ASSERT_ALWAYS(_child2, "Called getFirstChildNode() on a leaf node");
    return OBBTreeNode(*_child2);
}

const SimTK::Array_<int>&
Smith2018ContactMesh::OBBTreeNode::getTriangles() const {
    SimTK_ASSERT_ALWAYS(
            _child2 == NULL, "Called getTriangles() on a non-leaf node");
    return _triangles;
}

int Smith2018ContactMesh::OBBTreeNode::getNumTriangles() const {
    return _numTriangles;
}
