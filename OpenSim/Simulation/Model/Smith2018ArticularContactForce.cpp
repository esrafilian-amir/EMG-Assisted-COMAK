/* -------------------------------------------------------------------------- *
 *                 Smith2018ArticularContactForce.cpp                         *
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



//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/GCVSpline.h>
#include "Smith2018ArticularContactForce.h"
#include "Smith2018ContactMesh.h"
#include "Smith2018DistanceField.h"
#include <map>
#include <mutex>
#include <cctype>
#include <cmath>

//=============================================================================
// USING
//=============================================================================
using namespace OpenSim;
using namespace SimTK;

//=============================================================================
// CONSTRUCTOR(S) 
//=============================================================================

Smith2018ArticularContactForce::Smith2018ArticularContactForce() : Force()
{
    setNull();
    constructProperties();
}

Smith2018ArticularContactForce::Smith2018ArticularContactForce(
    const std::string& name,
    Smith2018ContactMesh& target_mesh,Smith2018ContactMesh& casting_mesh)
{
    setNull();
    constructProperties();

    setName(name);

    updSocket<Smith2018ContactMesh>("target_mesh").connect(target_mesh);
    updSocket<Smith2018ContactMesh>("casting_mesh").connect(casting_mesh);
}

void Smith2018ArticularContactForce::setNull()
{
    _distance_field_cache = std::make_shared<DistanceFieldCache>();
    setAuthors("Colin Smith");
    setReferences(
        "Smith, C. R., Won Choi, K., Negrut, D., & Thelen, D. G. (2018)."
        "Efficient computation of cartilage contact pressures within dynamic "
        "simulations of movement. Computer Methods in Biomechanics and "
        "Biomedical Engineering: Imaging & Visualization, 6(5), 491-498.");
}

void Smith2018ArticularContactForce::constructProperties()
{
    constructProperty_min_proximity(0.00);
    constructProperty_max_proximity(0.01);
    constructProperty_elastic_foundation_formulation("linear");
    constructProperty_use_lumped_contact_model(true);
    constructProperty_max_strain(0.8);
    constructProperty_contact_method("ray_cast");
    constructProperty_field_voxel_size(0.001);
    constructProperty_field_edge_taper_width(0.003);
}

// Signed distance grids of the meshes used as target (contact_method
// 'distance_field'), built on first use.
struct Smith2018ArticularContactForce::DistanceFieldCache {
    std::mutex mutex;
    std::map<const Smith2018ContactMesh*,
        std::shared_ptr<Smith2018DistanceField>> fields;
    std::map<const Smith2018ContactMesh*, double> signatures;
};

const Smith2018DistanceField& Smith2018ArticularContactForce::getDistanceField(
    const Smith2018ContactMesh& mesh) const
{
    std::lock_guard<std::mutex> lock(_distance_field_cache->mutex);
    auto& slot = _distance_field_cache->fields[&mesh];
    // rebuild if the mesh or the settings changed (the mesh object can be
    // the same after a new mesh_file was set)
    const SimTK::Vector_<SimTK::Vec3>& verts = mesh.getVertexLocations();
    double signature = get_field_voxel_size() * 1e3 + get_max_proximity() * 1e5 +
        get_field_edge_taper_width() * 1e7 +
        verts.size() * 1e-3;
    for (int k = 0; k < 8 && verts.size() > 0; ++k) {
        const SimTK::Vec3& v = verts[(int)((long long)(verts.size() - 1) * k / 7)];
        signature += (k + 1) * (v[0] * 1.3 + v[1] * 2.7 + v[2] * 5.1);
    }
    auto sig_it = _distance_field_cache->signatures.find(&mesh);
    if (sig_it == _distance_field_cache->signatures.end() ||
        sig_it->second != signature) {
        slot.reset();
        _distance_field_cache->signatures[&mesh] = signature;
    }
    if (!slot) {
        slot = std::make_shared<Smith2018DistanceField>(
            mesh.getVertexLocations(), mesh.getTriangleConnectivity(),
            get_field_voxel_size(), std::max(get_max_proximity(), 0.0),
            get_field_edge_taper_width());
        log_info("{}: distance field of '{}' at {} m{}.",
            getName(), mesh.getName(), slot->getVoxelSize(),
            slot->isClosed() ? " (closed mesh)" :
            " (open mesh, " + std::to_string(slot->getNumBoundaryEdges()) +
            " free edges)");
    }
    return *slot;
}

void Smith2018ArticularContactForce::checkPropertyValues() const
{
    const std::string& formulation = get_elastic_foundation_formulation();
    OPENSIM_THROW_IF_FRMOBJ(formulation != "linear" && formulation != "nonlinear",
        InvalidPropertyValue, getProperty_elastic_foundation_formulation().getName(),
        "elastic_foundation_formulation must be 'linear' or 'nonlinear', but "
        "it is '" + formulation + "'.");

    OPENSIM_THROW_IF_FRMOBJ(get_contact_method() != "ray_cast" &&
        get_contact_method() != "distance_field",
        InvalidPropertyValue, getProperty_contact_method().getName(),
        "contact_method must be 'ray_cast' or 'distance_field', but it is '" +
        get_contact_method() + "'.");

    OPENSIM_THROW_IF_FRMOBJ(!(get_field_voxel_size() > 0.0),
        InvalidPropertyValue, getProperty_field_voxel_size().getName(),
        "field_voxel_size must be greater than 0.");
    OPENSIM_THROW_IF_FRMOBJ(!(get_field_edge_taper_width() >= 0.0),
        InvalidPropertyValue, getProperty_field_edge_taper_width().getName(),
        "field_edge_taper_width must be 0 or greater.");

    OPENSIM_THROW_IF_FRMOBJ(!(get_max_strain() > 0.0 && get_max_strain() < 1.0),
        InvalidPropertyValue, getProperty_max_strain().getName(),
        "max_strain must be greater than 0 and less than 1.");
}

namespace {

// Nonlinear elastic foundation (Bei and Fregly 2004) of one or two elastic
// layers in series, for the total overlap depth d of the layers:
//     d = h1 (1 - exp(-P/k1)) + h2 (1 - exp(-P/k2)),   k = (1-v)E/((1+v)(1-2v))
// Pass h2 = 0 for a single (lumped) layer.
//
// The relationship is only defined for strains (depth/thickness) below one, so
// the pressure is continued linearly (same slope) beyond the strain
// max_strain of a layer. The pressure and the energy are therefore finite,
// continuous and have a continuous derivative for any depth, and the energy is
// the integral of the pressure (dE/dd = A * P, with the energy per unit area
// returned here times the area A).
//
// The pressure is found with a Newton iteration started at the linear
// pressure. The depth is a concave increasing function of the pressure and
// the linear pressure is never larger than the solution, so the iteration
// converges monotonically (a bisection is the fallback).
//
// Returns the energy per unit area (J/m^2) and sets the pressure (Pa) and
// whether the strain cap was used.
double nonlinearElasticFoundation(double d, double h1, double k1, double h2,
        double k2, double max_strain, double& pressure, bool& capped) {

    const bool two_layers = h2 > 0.0;

    // depth, its slope w.r.t. the pressure and the energy per area
    auto depth = [&](double P) {
        double depth = h1 * (-std::expm1(-P / k1));
        if (two_layers) { depth += h2 * (-std::expm1(-P / k2)); }
        return depth;
    };
    auto slope = [&](double P) {
        double slope = h1 / k1 * std::exp(-P / k1);
        if (two_layers) { slope += h2 / k2 * std::exp(-P / k2); }
        return slope;
    };
    auto layer_energy = [](double P, double k, double h) {
        const double x = P / k;
        return k * h * (-std::expm1(-x) - x * std::exp(-x));
    };
    auto energy = [&](double P) {
        double energy = layer_energy(P, k1, h1);
        if (two_layers) { energy += layer_energy(P, k2, h2); }
        return energy;
    };

    // pressure at which the most strained layer reaches max_strain
    double P_cap = -k1 * std::log1p(-max_strain);
    if (two_layers) { P_cap = std::min(P_cap, -k2 * std::log1p(-max_strain)); }
    const double d_cap = depth(P_cap);

    capped = d > d_cap;
    if (capped) {
        const double excess = d - d_cap;
        const double stiffness = 1.0 / slope(P_cap);
        pressure = P_cap + stiffness * excess;
        return energy(P_cap) + P_cap * excess + 0.5 * stiffness * excess * excess;
    }

    double P;
    if (!two_layers) {
        P = -k1 * std::log1p(-d / h1);
    } else {
        const double tol = 1e-13 * (h1 + h2);
        P = d / (h1 / k1 + h2 / k2);
        for (int it = 0; it < 50 && std::abs(depth(P) - d) > tol; ++it) {
            P -= (depth(P) - d) / slope(P);
        }
        if (!(std::abs(depth(P) - d) <= 1e-9 * (h1 + h2))) {
            double lo = 0.0, hi = P_cap;
            for (int it = 0; it < 200; ++it) {
                P = 0.5 * (lo + hi);
                if (depth(P) < d) { lo = P; } else { hi = P; }
            }
        }
    }
    pressure = P;
    return energy(P);
}

} // namespace

void Smith2018ArticularContactForce::
extendAddToSystem(MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    // Checked here and not when the properties are finalized: a component that
    // throws in finalizeFromProperties() cannot be added to a model with
    // Model::addModelComponent() (generic tests that fill the properties of all
    // components with random values rely on that)
    checkPropertyValues();

    int target_mesh_nTri =
        getSocket<Smith2018ContactMesh>("target_mesh").
        getConnectee().getNumFaces();
    int casting_mesh_nTri = 
        getSocket<Smith2018ContactMesh>("casting_mesh").
        getConnectee().getNumFaces();

    const int target_mesh_nRegions = getSocket<Smith2018ContactMesh>("target_mesh").
        getConnectee().getNumRegions();
    const int casting_mesh_nRegions = getSocket<Smith2018ContactMesh>("casting_mesh").
        getConnectee().getNumRegions();

    Vector target_mesh_def_vec(target_mesh_nTri);
    Vector casting_mesh_def_vec(casting_mesh_nTri);
    target_mesh_def_vec = -1;
    casting_mesh_def_vec = -1;

    Vector_<Vec3> target_mesh_def_vec3(target_mesh_nTri,Vec3(0.0));
    Vector_<Vec3> casting_mesh_def_vec3(casting_mesh_nTri,Vec3(0.0));
    

    std::vector<int> target_mesh_def_vector_int(target_mesh_nTri,-1);
    std::vector<int> casting_mesh_def_vector_int(casting_mesh_nTri,-1);

    //TODO: These need to be accessed at in Stage::Position in computeProximity()
    // for rechecking the same triangle that was in contact in the previous
    // state. Is there a better way to make them accessible without setting
    // the stage to LowestRuntime???

    this->_target_triangle_previous_contacting_triangleCV = 
        addCacheVariable(
        "target_triangle_previous_contacting_triangle",
        target_mesh_def_vector_int, Stage::LowestRuntime);

    this->_casting_triangle_previous_contacting_triangleCV =
        addCacheVariable(
        "casting_triangle_previous_contacting_triangle",
        casting_mesh_def_vector_int, Stage::LowestRuntime);

    //Triangles with ray intersections
    this->_target_num_active_trianglesCV = 
        addCacheVariable("target_num_active_triangles",
        0, Stage::Position);

    this->_casting_num_active_trianglesCV =
        addCacheVariable("casting_num_active_triangles",
        0, Stage::Position);

    //Subset of num_active_triangles with positive proximity
    this->_target_num_contacting_trianglesCV = 
        addCacheVariable("target_num_contacting_triangles",
        0, Stage::Position);

    this->_casting_num_contacting_trianglesCV = 
        addCacheVariable("casting_num_contacting_triangles",
        0, Stage::Position);
    
    //same, neighbor, and different are useful for debugging issues with
    //newly constructed contact meshes
    //Subset of n_contacting_tri that contact same triangle as previous step
    this->_target_num_contacting_triangles_sameCV =
        addCacheVariable("target_num_contacting_triangles_same",
        0, Stage::Position);

    this->_casting_num_contacting_triangles_sameCV =
        addCacheVariable("casting_num_contacting_triangles_same",
        0, Stage::Position);

    //Subset of n_contacting_tri that contact 
    //neighboring triangle to previous step
    this->_target_num_contacting_triangles_neighborCV =
        addCacheVariable("target_num_contacting_triangles_neighbor",
        0, Stage::Position);

    this->_casting_num_contacting_triangles_neighborCV = 
        addCacheVariable("casting_num_contacting_triangles_neighbor",
        0, Stage::Position);

    //Subset of n_contacting_tri that contact different triangle from previous 
    //step (not same or neighbor), this means expensive OBB check was used
    this->_target_num_contacting_triangles_differentCV =
        addCacheVariable("target_num_contacting_triangles_different",
        0, Stage::Position);

    this->_casting_num_contacting_triangles_differentCV = 
        addCacheVariable("casting_num_contacting_triangles_different",
        0, Stage::Position);


    this->_target_triangle_proximityCV =
        addCacheVariable("target_triangle_proximity",
        target_mesh_def_vec, Stage::Position);
    this->_casting_triangle_proximityCV =
        addCacheVariable("casting_triangle_proximity",
        casting_mesh_def_vec, Stage::Position);
    
    this->_target_triangle_pressureCV =
        addCacheVariable("target_triangle_pressure",
        target_mesh_def_vec, Stage::Position);
    this->_casting_triangle_pressureCV =
        addCacheVariable("casting_triangle_pressure",
        casting_mesh_def_vec, Stage::Position);

    this->_target_triangle_potential_energyCV = 
        addCacheVariable("target_triangle_potential_energy",
        target_mesh_def_vec, Stage::Position);
    this->_casting_triangle_potential_energyCV =
        addCacheVariable("casting_triangle_potential_energy",
        casting_mesh_def_vec, Stage::Position);

    this->_target_triangle_forceCV =
        addCacheVariable("target_triangle_force",
        target_mesh_def_vec3, Stage::Position);
    this->_casting_triangle_forceCV = 
        addCacheVariable("casting_triangle_force",
        casting_mesh_def_vec3, Stage::Position);


    //Lazy Evaluations (only computed getXX is called)
    this->_target_total_contact_areaCV = 
        addCacheVariable
        ("target_total_contact_area", 0.0, Stage::Position);
    this->_target_total_mean_proximityCV =
        addCacheVariable
        ("target_total_mean_proximity", 0.0, Stage::Position);
    this->_target_total_max_proximityCV = 
        addCacheVariable
        ("target_total_max_proximity", 0.0, Stage::Position);
    this->_target_total_center_of_proximityCV =
        addCacheVariable
        ("target_total_center_of_proximity", Vec3(0), Stage::Position);
    this->_target_total_mean_pressureCV =
        addCacheVariable
        ("target_total_mean_pressure", 0.0, Stage::Position);
    this->_target_total_max_pressureCV =
        addCacheVariable
        ("target_total_max_pressure", 0.0, Stage::Position);
    this->_target_total_center_of_pressureCV =
        addCacheVariable
        ("target_total_center_of_pressure", Vec3(0), Stage::Position);
    this->_target_total_contact_forceCV = 
        addCacheVariable
        ("target_total_contact_force", Vec3(0), Stage::Position);
    this->_target_total_contact_momentCV =
        addCacheVariable(
        "target_total_contact_moment", Vec3(0), Stage::Position);
    this->_casting_total_contact_areaCV =
        addCacheVariable(
        "casting_total_contact_area", 0.0, Stage::Position);
    this->_casting_total_mean_proximityCV =
        addCacheVariable(
        "casting_total_mean_proximity", 0.0, Stage::Position);
    this->_casting_total_max_proximityCV =
        addCacheVariable(
        "casting_total_max_proximity", 0.0, Stage::Position);
    this->_casting_total_center_of_proximityCV =
        addCacheVariable(
        "casting_total_center_of_proximity", Vec3(0), Stage::Position);
    this->_casting_total_mean_pressureCV =
        addCacheVariable(
        "casting_total_mean_pressure", 0.0, Stage::Position);
    this->_casting_total_max_pressureCV =
        addCacheVariable(
        "casting_total_max_pressure", 0.0, Stage::Position);
    this->_casting_total_center_of_pressureCV =
        addCacheVariable(
        "casting_total_center_of_pressure", Vec3(0), Stage::Position);
    this->_casting_total_contact_forceCV =
        addCacheVariable(
        "casting_total_contact_force", Vec3(0), Stage::Position);
    this->_casting_total_contact_momentCV =
        addCacheVariable(
        "casting_total_contact_moment", Vec3(0), Stage::Position);
    this->_target_regional_contact_areaCV =
        addCacheVariable(
        "target_regional_contact_area", Vector(target_mesh_nRegions,0.0), Stage::Position);
    this->_target_regional_mean_proximityCV =
        addCacheVariable(
        "target_regional_mean_proximity", Vector(target_mesh_nRegions,0.0), Stage::Position);
    this->_target_regional_max_proximityCV =
        addCacheVariable(
        "target_regional_max_proximity", Vector(target_mesh_nRegions,0.0), Stage::Position);
    this->_target_regional_center_of_proximityCV =
        addCacheVariable(
        "target_regional_center_of_proximity", Vector_<Vec3>(target_mesh_nRegions,Vec3(0)),
        Stage::Position);
    this->_target_regional_mean_pressureCV =
        addCacheVariable(
        "target_regional_mean_pressure", Vector(target_mesh_nRegions,0.0), Stage::Position);
    this->_target_regional_max_pressureCV =
        addCacheVariable(
        "target_regional_max_pressure",Vector(target_mesh_nRegions,0.0), Stage::Position);
    this->_target_regional_center_of_pressureCV =
        addCacheVariable(
        "target_regional_center_of_pressure", Vector_<Vec3>(target_mesh_nRegions,Vec3(0)),
        Stage::Position);
    this->_target_regional_contact_forceCV =
        addCacheVariable(
        "target_regional_contact_force", Vector_<Vec3>(target_mesh_nRegions,Vec3(0)),
        Stage::Position);
    this->_target_regional_contact_momentCV =
        addCacheVariable(
        "target_regional_contact_moment", Vector_<Vec3>(target_mesh_nRegions,Vec3(0)),
        Stage::Position);
    this->_casting_regional_contact_areaCV =
        addCacheVariable(
        "casting_regional_contact_area", Vector(casting_mesh_nRegions,0.0), Stage::Position);
    this->_casting_regional_mean_proximityCV =
        addCacheVariable(
        "casting_regional_mean_proximity", Vector(casting_mesh_nRegions,0.0), Stage::Position);
    this->_casting_regional_max_proximityCV =
        addCacheVariable(
        "casting_regional_max_proximity", Vector(casting_mesh_nRegions,0.0), Stage::Position);
    this->_casting_regional_center_of_proximityCV =
        addCacheVariable(
        "casting_regional_center_of_proximity", Vector_<Vec3>(casting_mesh_nRegions, Vec3(0)),
        Stage::Position);
    this->_casting_regional_mean_pressureCV =
        addCacheVariable(
        "casting_regional_mean_pressure", Vector(casting_mesh_nRegions,0.0), Stage::Position);
    this->_casting_regional_max_pressureCV =
        addCacheVariable(
        "casting_regional_max_pressure", Vector(casting_mesh_nRegions,0.0), Stage::Position);
    this->_casting_regional_center_of_pressureCV =
        addCacheVariable(
        "casting_regional_center_of_pressure", Vector_<Vec3>(casting_mesh_nRegions,Vec3(0)),
        Stage::Position);
    this->_casting_regional_contact_forceCV =
        addCacheVariable(
        "casting_regional_contact_force", Vector_<Vec3>(casting_mesh_nRegions,Vec3(0)),
        Stage::Position);
    this->_casting_regional_contact_momentCV =
        addCacheVariable(
        "casting_regional_contact_moment", Vector_<Vec3>(casting_mesh_nRegions,Vec3(0)),
        Stage::Position);

    //Modeling Options
    //----------------
    addModelingOption("flip_meshes", 1);
}

void Smith2018ArticularContactForce::computeMeshProximity(
    const State& state, const Smith2018ContactMesh& casting_mesh,
    const Smith2018ContactMesh& target_mesh, const std::string& cache_mesh_name) const
{
    SimTK::Vector triangle_proximity;
    computeMeshProximity(state, casting_mesh, target_mesh,
        cache_mesh_name, triangle_proximity);
}

void Smith2018ArticularContactForce::computeMeshProximity(
    const State& state, const Smith2018ContactMesh& casting_mesh,
    const Smith2018ContactMesh& target_mesh, const std::string& cache_mesh_name,
    SimTK::Vector& triangle_proximity) const
{
    // Get Mesh Properties
    Vector_<SimTK::Vec3> tri_cen = casting_mesh.getTriangleCenters();
    Vector_<SimTK::UnitVec3> tri_nor = casting_mesh.getTriangleNormals();

    Transform MeshCtoMeshT = casting_mesh.getMeshFrame().
        findTransformBetween(state, target_mesh.getMeshFrame());

    //Initialize contact variables
    //----------------------------

    //Number of triangles with positive ray intersection tests
    int nActiveTri = 0;

    //Subset of nActiveTri with positive proximity
    int nContactingTri = 0;

    triangle_proximity.resize(casting_mesh.getNumFaces());
    triangle_proximity = 0;

    std::vector<int>& target_tri = (cache_mesh_name == "target") ?
        this->updCacheVariableValue(
            state, this->_target_triangle_previous_contacting_triangleCV) :
        this->updCacheVariableValue(
            state, this->_casting_triangle_previous_contacting_triangleCV);

    //Keep track of triangle collision type for debugging
    int nSameTri = 0;
    int nNeighborTri = 0;
    int nDiffTri = 0;

    //Collision Detection
    //-------------------
    const Smith2018DistanceField* field =
        (get_contact_method() == "distance_field") ?
        &getDistanceField(target_mesh) : nullptr;

    //Loop through all triangles in casting mesh
    for (int i = 0; i < casting_mesh.getNumFaces(); ++i) {
        bool contact_detected = false;
        double distance = 0.0;
        SimTK::Vec3 contact_point;
        SimTK::Vec3 origin = MeshCtoMeshT.shiftFrameStationToBase(tri_cen(i));
        SimTK::UnitVec3 direction(
            MeshCtoMeshT.xformFrameVecToBase(tri_nor(i)));

        // Distance field: depth of the triangle center below the target
        // surface, no ray
        if (field) {
            Smith2018DistanceField::Query q;
            // the depth fades out at a free edge of the target (weight)
            if (field->evaluate(origin, q) && q.weight > 0.0 &&
                q.depth <= get_max_proximity() &&
                q.depth * q.weight >= get_min_proximity()) {
                const double depth = q.depth * q.weight;
                target_tri[i] = q.triangle;
                triangle_proximity(i) = depth;
                nActiveTri++;
                nDiffTri++;
                if (depth > 0.0) { nContactingTri++; }
            }
            else {
                target_tri[i] = -1;
            }
            continue;
        }

        //If triangle was in contact in previous timestep, 
        //recheck same contact triangle and neighbors
        if (target_tri[i] >= 0) {
            //same triangle
            if (target_mesh._obb.rayIntersectTri(
                target_mesh.getPolygonalMesh(), origin, -direction,
                target_tri[i], contact_point, distance))
            {
                if (distance >= get_min_proximity() &&
                    distance <= get_max_proximity()) {

                    triangle_proximity(i) = distance;

                    nActiveTri++;
                    nSameTri++;

                    if (triangle_proximity(i) > 0.0) { nContactingTri++; }
                }
                continue;

            }

            //neighboring triangles
            const std::set<int>& neighborTris =
                target_mesh.getNeighborTris(target_tri[i]);

            for (int neighbor_tri : neighborTris) {
                if (target_mesh._obb.rayIntersectTri(
                    target_mesh.getPolygonalMesh(), origin, -direction,
                    neighbor_tri, contact_point, distance))
                {
                    if (distance >= get_min_proximity() &&
                        distance <= get_max_proximity()) {

                        triangle_proximity(i) = distance;

                        target_tri[i] = neighbor_tri;

                        nActiveTri++;
                        nNeighborTri++;
                        if (triangle_proximity(i) > 0.0) { nContactingTri++; }

                        contact_detected = true;
                        break;
                    }
                }
            }
            if (contact_detected) {
                continue;
            }
        }

        //No luck in rechecking same triangle and neighbors
        //Go through the expensive OBB hierarchy
        int contact_target_tri = -1;

        if (target_mesh.rayIntersectMesh(origin, -direction,
            get_min_proximity(), get_max_proximity(),
            contact_target_tri, contact_point, distance)) {

            target_tri[i] = contact_target_tri;
            triangle_proximity(i) = distance;

            nActiveTri++;
            nDiffTri++;
            if (triangle_proximity(i) > 0.0) { nContactingTri++; }
            continue;
        }

        //Else - triangle is not in contact
        target_tri[i] = -1;
    }

    //Store Contact Info
    if (cache_mesh_name == "casting"){
        this->setCacheVariableValue(state, 
            this->_casting_triangle_proximityCV, triangle_proximity);
        this->setCacheVariableValue(state, 
            this->_casting_triangle_previous_contacting_triangleCV, target_tri);
        this->setCacheVariableValue(state, 
            this->_casting_num_active_trianglesCV, nActiveTri);
        this->setCacheVariableValue(state, 
            this->_casting_num_contacting_trianglesCV, nContactingTri);
        this->setCacheVariableValue(state, 
            this->_casting_num_contacting_triangles_sameCV, nSameTri);
        this->setCacheVariableValue(state, 
            this->_casting_num_contacting_triangles_neighborCV, nNeighborTri);
        this->setCacheVariableValue(state, 
            this->_casting_num_contacting_triangles_differentCV, nDiffTri);
    }
    else {
        this->setCacheVariableValue(state, 
            this->_target_triangle_proximityCV, triangle_proximity);
        this->setCacheVariableValue(state, 
            this->_target_triangle_previous_contacting_triangleCV, target_tri);
        this->setCacheVariableValue(state, 
            this->_target_num_active_trianglesCV, nActiveTri);
        this->setCacheVariableValue(state, 
            this->_target_num_contacting_trianglesCV, nContactingTri);
        this->setCacheVariableValue(state, 
            this->_target_num_contacting_triangles_sameCV, nSameTri);
        this->setCacheVariableValue(state, 
            this->_target_num_contacting_triangles_neighborCV, nNeighborTri);
        this->setCacheVariableValue(state, 
            this->_target_num_contacting_triangles_differentCV, nDiffTri);
    }
}

void Smith2018ArticularContactForce::computeMeshDynamics(
    const State& state, const Smith2018ContactMesh& casting_mesh,
    const Smith2018ContactMesh& target_mesh) const
{
    SimTK::Vector_<SimTK::Vec3> triangle_force;
    SimTK::Vector triangle_pressure;
    SimTK::Vector triangle_energy;

    computeMeshDynamics(
        state, casting_mesh, target_mesh,
        triangle_force, triangle_pressure, triangle_energy);
}

void Smith2018ArticularContactForce::computeMeshDynamics(
    const State& state, const Smith2018ContactMesh& casting_mesh,
    const Smith2018ContactMesh& target_mesh,
    SimTK::Vector_<SimTK::Vec3>& triangle_force,
    SimTK::Vector& triangle_pressure,
    SimTK::Vector& triangle_energy) const
{
    std::string casting_path = getConnectee<Smith2018ContactMesh>
        ("casting_mesh").getAbsolutePathString();

    bool flipped_meshes;
    if (casting_path == casting_mesh.getAbsolutePathString())
    {
        flipped_meshes = false;
    }
    else {
        flipped_meshes = true;
    }

    const Vector& triangle_proximity = (flipped_meshes) ?
        this->getCacheVariableValue(state, 
            this->_target_triangle_proximityCV) :
        this->getCacheVariableValue(state, 
            this->_casting_triangle_proximityCV);

    const std::vector<int>& target_tri = (flipped_meshes) ?
        this->getCacheVariableValue(state,
            this->_target_triangle_previous_contacting_triangleCV) :
        this->getCacheVariableValue(state,
            this->_casting_triangle_previous_contacting_triangleCV);

    const Vector& triangle_area = casting_mesh.getTriangleAreas();

    triangle_pressure.resize(casting_mesh.getNumFaces());
    triangle_pressure = 0;
    triangle_energy.resize(casting_mesh.getNumFaces());
    triangle_energy = 0;

    double hT, hC; //thickness
    double ET, EC; //elastic modulus
    double vT, vC; //poissons ratio

    //Compute Tri Pressure and Potential Energy
    //-----------------------------------------
    for (int i = 0; i < casting_mesh.getNumFaces(); ++i) {
        if (triangle_proximity(i) <= 0) {
            triangle_pressure(i) = 0;
            triangle_energy(i) = 0;
            continue;
        }

        if (target_tri[i] < 0 || target_tri[i] >= target_mesh.getNumFaces()) {
            continue;
        }
        //Material Properties
        hT = target_mesh.getTriangleThickness(target_tri[i]);
        ET = target_mesh.getTriangleElasticModulus(target_tri[i]);
        vT = target_mesh.getTrianglePoissonsRatio(target_tri[i]);
        
        hC = casting_mesh.getTriangleThickness(i);
        EC = casting_mesh.getTriangleElasticModulus(i);
        vC = casting_mesh.getTrianglePoissonsRatio(i);

        const bool linear = get_elastic_foundation_formulation() == "linear";
        const double area = triangle_area(i);
        const double d = triangle_proximity(i);
        bool capped = false;

        //Compute pressure & energy using the lumped contact model
        if (get_use_lumped_contact_model()) {
            double E = (ET + EC) / 2;
            double v = (vT + vC) / 2;
            double h = (hT + hC);

            double K = (1 - v)*E / ((1 + v)*(1 - 2 * v));

            if (linear) {
                triangle_pressure(i) = K * d / h;
                triangle_energy(i) = 0.5 * area * K * SimTK::square(d) / h;
            }
            else {
                triangle_energy(i) = area * nonlinearElasticFoundation(
                    d, h, K, 0.0, 1.0, get_max_strain(),
                    triangle_pressure(i), capped);
            }
        }

        //Compute pressure & energy using variable property model
        else {
            if (linear) {
                double kT = ((1 - vT)*ET) / ((1 + vT)*(1 - 2 * vT)*hT);
                double kC = ((1 - vC)*EC) / ((1 + vC)*(1 - 2 * vC)*hC);

                triangle_pressure(i) = (kT*kC) / (kT + kC) * d;

                double depthT = kC / (kT + kC) * d;
                double depthC = kT / (kT + kC) * d;

                double energyC = 0.5 * area * kC * SimTK::square(depthC);
                double energyT = 0.5 * area * kT * SimTK::square(depthT);
                triangle_energy(i) = energyC + energyT;
            }
            else {
                // stiffness without the thickness (as in the lumped model)
                double KT = ((1 - vT)*ET) / ((1 + vT)*(1 - 2 * vT));
                double KC = ((1 - vC)*EC) / ((1 + vC)*(1 - 2 * vC));
                triangle_energy(i) = area * nonlinearElasticFoundation(
                    d, hC, KC, hT, KT, get_max_strain(),
                    triangle_pressure(i), capped);
            }
        }

        // A triangle deeper than max_proximity has no contact: its force
        // drops to zero abruptly, which can stop a simulation (the
        // integrator takes steps of 1e-13). Warn when a triangle gets close.
        if (!_max_proximity_warning_issued &&
            d > 0.8 * get_max_proximity()) {
            _max_proximity_warning_issued = true;
            log_warn("{}: the penetration of at least one triangle ({} m) "
                "reached 80% of max_proximity ({} m). A triangle deeper than "
                "max_proximity gets no contact: its force drops to zero "
                "abruptly, which can stop a simulation (the integrator takes "
                "extremely small steps). Increase max_proximity, e.g. to "
                "several times the layer thickness; this warning is shown "
                "once.", getName(), d, get_max_proximity());
        }

        if (capped && !_max_strain_warning_issued) {
            _max_strain_warning_issued = true;
            log_warn("{}: the overlap of at least one triangle pair is above "
                "max_strain ({}) of the layer thickness. The pressure is "
                "continued linearly beyond that strain. This usually means "
                "that the meshes are too deep in each other (check the "
                "pose); this warning is shown once.",
                getName(), get_max_strain());
        }
    }

    //Compute Triangle Forces
    //-----------------------
    const Vector_<UnitVec3>& triangle_normal = casting_mesh.getTriangleNormals();

    triangle_force.resize(casting_mesh.getNumFaces());
    triangle_force = Vec3(0.0);

    const bool use_field = get_contact_method() == "distance_field";
    const Smith2018DistanceField* field =
        use_field ? &getDistanceField(target_mesh) : nullptr;
    const Transform MeshCtoMeshT = casting_mesh.getMeshFrame().
        findTransformBetween(state, target_mesh.getMeshFrame());
    const Vector_<Vec3>& triangle_center = casting_mesh.getTriangleCenters();

    for (int i = 0; i < casting_mesh.getNumFaces(); ++i) {
        if (field) {
            // The force pushes the casting triangle out of the target along
            // the gradient of the signed distance.
            if (triangle_pressure(i) <= 0) { continue; }
            Smith2018DistanceField::Query q;
            if (!field->evaluate(MeshCtoMeshT.shiftFrameStationToBase(
                    triangle_center(i)), q)) { continue; }
            triangle_force(i) = MeshCtoMeshT.xformBaseVecToFrame(q.outward) *
                (triangle_pressure(i) * triangle_area(i));
            continue;
        }
        for (int j = 0; j < 3; ++j) {
            triangle_force(i)(j) =
                triangle_pressure(i) * triangle_area(i) * -triangle_normal(i)(j);
        }
    }

    
    if (flipped_meshes) {
        this->setCacheVariableValue(state,
            this->_target_triangle_pressureCV, triangle_pressure);
        this->setCacheVariableValue(state,
            this->_target_triangle_potential_energyCV, triangle_energy);
        this->setCacheVariableValue(state, 
            this->_target_triangle_forceCV, triangle_force);
    }
    else {
        this->setCacheVariableValue(state,
            this->_casting_triangle_pressureCV, triangle_pressure);
        this->setCacheVariableValue(state,
            this->_casting_triangle_potential_energyCV, triangle_energy);
        this->setCacheVariableValue(state, 
            this->_casting_triangle_forceCV, triangle_force);
    }

    return;
}

void Smith2018ArticularContactForce::computeForce(const State& state,
    Vector_<SpatialVec>& bodyForces,
    Vector& generalizedForces) const
{
    const Smith2018ContactMesh& casting_mesh =
        getConnectee<Smith2018ContactMesh>("casting_mesh");

    const Smith2018ContactMesh& target_mesh =
        getConnectee<Smith2018ContactMesh>("target_mesh");

    //Proximity
    SimTK::Vector casting_triangle_proximity;
    if (!this->isCacheVariableValid(state, this->_casting_triangle_proximityCV)) {
        computeMeshProximity(state, casting_mesh, target_mesh,
            "casting", casting_triangle_proximity);
    }
    else {
        casting_triangle_proximity = 
            this->getCacheVariableValue(state, this->_casting_triangle_proximityCV);
    }

    SimTK::Vector casting_triangle_pressure;
    SimTK::Vector casting_triangle_energy;
    SimTK::Vector_<Vec3> casting_triangle_force;

    //Pressure
    computeMeshDynamics(state, casting_mesh, target_mesh, casting_triangle_force,
        casting_triangle_pressure, casting_triangle_energy);

    //Force
    const PhysicalFrame& target_frame = target_mesh.getMeshFrame();
    const PhysicalFrame& casting_frame = casting_mesh.getMeshFrame();

    const Vector_<Vec3>& triangle_center = casting_mesh.getTriangleCenters();

    Transform T_casting_to_ground = casting_frame.getTransformInGround(state);

    // Triangles without contact carry no force. The forces of all contacting
    // triangles are summed into one force and one couple on each body
    // (equivalent to applying them one by one), which avoids two calls per
    // triangle for the whole mesh.
    const Vec3 origin_casting = T_casting_to_ground.p();
    const Vec3 origin_target = target_frame.getTransformInGround(state).p();
    Vec3 total_force_ground(0);
    Vec3 moment_casting_ground(0);
    Vec3 moment_target_ground(0);

    for (int i = 0; i < casting_mesh.getNumFaces(); ++i) {
        if (casting_triangle_pressure(i) <= 0) { continue; }

        const Vec3 force_ground =
            T_casting_to_ground.xformFrameVecToBase(casting_triangle_force(i));
        const Vec3 point_ground =
            T_casting_to_ground.shiftFrameStationToBase(triangle_center(i));

        total_force_ground += force_ground;
        moment_casting_ground += (point_ground - origin_casting) % force_ground;
        moment_target_ground += (point_ground - origin_target) % (-force_ground);
    }

    applyForceToPoint(state, casting_frame, Vec3(0), total_force_ground,
        bodyForces);
    applyTorque(state, casting_frame, moment_casting_ground, bodyForces);

    applyForceToPoint(state, target_frame, Vec3(0), -total_force_ground,
        bodyForces);
    applyTorque(state, target_frame, moment_target_ground, bodyForces);

    //Compute Proximity and Pressure for Target mesh (optional for analysis)
    if (getModelingOption(state, "flip_meshes")) {
        SimTK::Vector target_triangle_proximity;

        //target proximity
        if (!this->isCacheVariableValid(state,
            this->_target_triangle_proximityCV)) {
            computeMeshProximity(state, target_mesh, casting_mesh,
                "target", target_triangle_proximity);
        }
        else {
            target_triangle_proximity =
                this->getCacheVariableValue(state,
                    this->_target_triangle_proximityCV);
        }
    
        
        //target pressure
        SimTK::Vector target_triangle_pressure;
        SimTK::Vector_<SimTK::Vec3> target_triangle_force;
        SimTK::Vector target_triangle_energy;

        computeMeshDynamics(state, target_mesh, casting_mesh,
            target_triangle_force, target_triangle_pressure, target_triangle_energy);

        /*setCacheVariableValue(state,
            "target.triangle.proximity", target_triangle_proximity);

        setCacheVariableValue(state,
            "target.triangle.pressure", target_triangle_pressure);

        setCacheVariableValue(state,
            "target.triangle.potential_energy", target_triangle_energy);*/
    }
}

//Compute Contact Stats
void Smith2018ArticularContactForce::realizeContactMetricCaches(
    const SimTK::State& state) const
{
    const Smith2018ContactMesh& casting_mesh =
        getConnectee<Smith2018ContactMesh>("casting_mesh");

    const Smith2018ContactMesh& target_mesh =
        getConnectee<Smith2018ContactMesh>("target_mesh");

    const SimTK::Vector& casting_triangle_proximity = 
        this->getCacheVariableValue(
            state, this->_casting_triangle_proximityCV);

    const SimTK::Vector& casting_triangle_pressure = 
        this->getCacheVariableValue(
            state, this->_casting_triangle_pressureCV);

    std::vector<int> casting_faces;
    for (int i = 0; i < casting_mesh.getNumFaces(); ++i) {
        casting_faces.push_back(i);
    }

    auto stats = computeContactStats(casting_mesh, casting_triangle_proximity,
        casting_triangle_pressure,casting_faces);
    
    this->setCacheVariableValue(state, 
        this->_casting_total_contact_areaCV, stats.contact_area);
    this->setCacheVariableValue(state, 
        this->_casting_total_mean_proximityCV, stats.mean_proximity);
    this->setCacheVariableValue(state, 
        this->_casting_total_max_proximityCV, stats.max_proximity);
    this->setCacheVariableValue(state, 
        this->_casting_total_center_of_proximityCV, stats.center_of_proximity);
    this->setCacheVariableValue(state, 
        this->_casting_total_mean_pressureCV, stats.mean_pressure);
    this->setCacheVariableValue(state, 
        this->_casting_total_max_pressureCV, stats.max_pressure);
    this->setCacheVariableValue(state, 
        this->_casting_total_center_of_pressureCV, stats.center_of_pressure);
    this->setCacheVariableValue(state, 
        this->_casting_total_contact_forceCV, stats.contact_force);
    this->setCacheVariableValue(state, 
        this->_casting_total_contact_momentCV, stats.contact_moment);
 

    //Target mesh computations (not used in applied contact force calculation)
    SimTK::Vector target_triangle_proximity;
    SimTK::Vector target_triangle_pressure;

    if (getModelingOption(state, "flip_meshes")) {
        
        target_triangle_proximity = 
            this->getCacheVariableValue(state, 
                this->_target_triangle_proximityCV);

        target_triangle_pressure = 
            this->getCacheVariableValue(state, 
                this->_target_triangle_pressureCV);

        //target contact stats
        std::vector<int> Tcasting_faces;
        for (int i = 0; i < target_mesh.getNumFaces(); ++i) {
            Tcasting_faces.push_back(i);
        }

        auto Tstats = computeContactStats(target_mesh, target_triangle_proximity,
            target_triangle_pressure,Tcasting_faces);
    
        this->setCacheVariableValue(state,
            this->_target_total_contact_areaCV, Tstats.contact_area);
        this->setCacheVariableValue(state,
            this->_target_total_mean_proximityCV, Tstats.mean_proximity);
        this->setCacheVariableValue(state,
            this->_target_total_max_proximityCV, Tstats.max_proximity);
        this->setCacheVariableValue(state,
            this->_target_total_center_of_proximityCV, Tstats.center_of_proximity);
        this->setCacheVariableValue(state,
            this->_target_total_mean_pressureCV, Tstats.mean_pressure);
        this->setCacheVariableValue(state,
            this->_target_total_max_pressureCV, Tstats.max_pressure);
        this->setCacheVariableValue(state,
            this->_target_total_center_of_pressureCV, Tstats.center_of_pressure);
        this->setCacheVariableValue(state,
            this->_target_total_contact_forceCV, Tstats.contact_force);
        this->setCacheVariableValue(state,
            this->_target_total_contact_momentCV, Tstats.contact_moment);
    }

    //regional casting stats
    std::vector<std::vector<int>> casting_region_tri_ind = 
        casting_mesh.getRegionalTriangleIndices();

    const int nCastingRegions = casting_mesh.getNumRegions();

    SimTK::Vector reg_contact_area(nCastingRegions, 0.0);
    SimTK::Vector reg_mean_proximity(nCastingRegions, 0.0);
    SimTK::Vector reg_max_proximity(nCastingRegions, 0.0);
    SimTK::Vector_<SimTK::Vec3> reg_COPrx(nCastingRegions, SimTK::Vec3(0));
    SimTK::Vector reg_mean_pressure(nCastingRegions, 0.0);
    SimTK::Vector reg_max_pressure(nCastingRegions, 0.0);
    SimTK::Vector_<SimTK::Vec3> reg_COP(nCastingRegions, SimTK::Vec3(0));
    SimTK::Vector_<SimTK::Vec3> reg_contact_force(nCastingRegions, SimTK::Vec3(0));
    SimTK::Vector_<SimTK::Vec3> reg_contact_moment(nCastingRegions, SimTK::Vec3(0));

    for (int i = 0; i < nCastingRegions; ++i) {
        auto stats = computeContactStats(casting_mesh, 
            casting_triangle_proximity, casting_triangle_pressure,
            casting_region_tri_ind[i]);

        reg_contact_area(i) = stats.contact_area;
        reg_mean_proximity(i) = stats.mean_proximity;
        reg_max_proximity(i) = stats.max_proximity;
        reg_COPrx(i) = stats.center_of_proximity;
        reg_mean_pressure(i) = stats.mean_pressure;
        reg_max_pressure(i) = stats.max_pressure;
        reg_COP(i) = stats.center_of_pressure;
        reg_contact_force(i) = stats.contact_force;
        reg_contact_moment(i) = stats.contact_moment;
    }
    this->setCacheVariableValue(state,
        this->_casting_regional_contact_areaCV, reg_contact_area);
    this->setCacheVariableValue(state,
        this->_casting_regional_mean_proximityCV, reg_mean_proximity);
    this->setCacheVariableValue(state,
        this->_casting_regional_max_proximityCV, reg_max_proximity);
    this->setCacheVariableValue(state,
        this->_casting_regional_center_of_proximityCV, reg_COPrx);
    this->setCacheVariableValue(state,
        this->_casting_regional_mean_pressureCV, reg_mean_pressure);
    this->setCacheVariableValue(state,
        this->_casting_regional_max_pressureCV, reg_max_pressure);
    this->setCacheVariableValue(state,
        this->_casting_regional_center_of_pressureCV, reg_COP);
    this->setCacheVariableValue(state,
        this->_casting_regional_contact_forceCV, reg_contact_force);
    this->setCacheVariableValue(state,
        this->_casting_regional_contact_momentCV, reg_contact_moment);

    //target
    if (getModelingOption(state, "flip_meshes")) {
        
        std::vector<std::vector<int>> target_region_tri_ind =
            target_mesh.getRegionalTriangleIndices();

        const int nTargetRegions = target_mesh.getNumRegions();
        reg_contact_area.resize(nTargetRegions);
        reg_mean_proximity.resize(nTargetRegions);
        reg_max_proximity.resize(nTargetRegions);
        reg_COPrx.resize(nTargetRegions);
        reg_mean_pressure.resize(nTargetRegions);
        reg_max_pressure.resize(nTargetRegions);
        reg_COP.resize(nTargetRegions);
        reg_contact_force.resize(nTargetRegions);
        reg_contact_moment.resize(nTargetRegions);

        for (int i = 0; i < nTargetRegions; ++i) {
            auto stats = computeContactStats(target_mesh, 
                target_triangle_proximity, target_triangle_pressure, target_region_tri_ind[i]);

            reg_contact_area(i) = stats.contact_area;
            reg_mean_proximity(i) = stats.mean_proximity;
            reg_max_proximity(i) = stats.max_proximity;
            reg_COPrx(i) = stats.center_of_proximity;
            reg_mean_pressure(i) = stats.mean_pressure;
            reg_max_pressure(i) = stats.max_pressure;
            reg_COP(i) = stats.center_of_pressure;
            reg_contact_force(i) = stats.contact_force;
            reg_contact_moment(i) = stats.contact_moment;
        }

        this->setCacheVariableValue(state,
            this->_target_regional_contact_areaCV, reg_contact_area);
        this->setCacheVariableValue(state,
            this->_target_regional_mean_proximityCV, reg_mean_proximity);
        this->setCacheVariableValue(state,
            this->_target_regional_max_proximityCV, reg_max_proximity);
        this->setCacheVariableValue(state,
            this->_target_regional_center_of_proximityCV, reg_COPrx);
        this->setCacheVariableValue(state,
            this->_target_regional_mean_pressureCV, reg_mean_pressure);
        this->setCacheVariableValue(state,
            this->_target_regional_max_pressureCV, reg_max_pressure);
        this->setCacheVariableValue(state,
            this->_target_regional_center_of_pressureCV, reg_COP);
        this->setCacheVariableValue(state,
            this->_target_regional_contact_forceCV, reg_contact_force);
        this->setCacheVariableValue(state,
            this->_target_regional_contact_momentCV, reg_contact_moment);
    }
}

double Smith2018ArticularContactForce::
computePotentialEnergy(const SimTK::State& state) const
{
    if (!this->isCacheVariableValid(state, this->_casting_triangle_potential_energyCV)) {
        const Smith2018ContactMesh& casting_mesh =
            getConnectee<Smith2018ContactMesh>("casting_mesh");

        const Smith2018ContactMesh& target_mesh =
            getConnectee<Smith2018ContactMesh>("target_mesh");

        computeMeshDynamics(state,target_mesh, casting_mesh);
    }
    const SimTK::Vector& triangle_energy = this->getCacheVariableValue(
        state, this->_casting_triangle_potential_energyCV);
    return triangle_energy.sum();
}

Vec3 Smith2018ArticularContactForce::
computeContactForceVector(double pressure, double area, Vec3 normal) const
{
    Vec3 force = normal * pressure * area;
    return force;
}

Vec3 Smith2018ArticularContactForce::
computeContactMomentVector(double pressure, double area, Vec3 normal,
    Vec3 center) const
{
    Vec3 force = normal * pressure * area;
    Vec3 moment = SimTK::cross(center,force);
    return moment;
}

Smith2018ArticularContactForce::ContactStats
Smith2018ArticularContactForce::computeContactStats(
    const Smith2018ContactMesh& mesh,
    const SimTK::Vector& total_triangle_proximity,
    const SimTK::Vector& total_triangle_pressure,
    const std::vector<int>& triIndices) const
{
    Smith2018ArticularContactForce::ContactStats stats;

    int nTri = static_cast<int>(triIndices.size());

    SimTK::Vector triangle_proximity(nTri);
    SimTK::Vector triangle_pressure(nTri);

    const SimTK::Vector& total_triangle_area = mesh.getTriangleAreas();
    SimTK::Vector triangle_area(nTri);

    const SimTK::Vector_<UnitVec3>& total_triangle_normal = mesh.getTriangleNormals();
    SimTK::Vector_<UnitVec3> triangle_normal(nTri);

    const SimTK::Vector_<Vec3>& total_triangle_center = mesh.getTriangleCenters();
    SimTK::Vector_<Vec3> triangle_center(nTri);
    int nContactingTri = 0;

    for (int i = 0; i < nTri; ++i) {
        triangle_area(i) = total_triangle_area(triIndices[i]);
        triangle_normal(i) = total_triangle_normal(triIndices[i]);
        triangle_proximity(i) = total_triangle_proximity(triIndices[i]);
        triangle_pressure(i) = total_triangle_pressure(triIndices[i]);
        triangle_center(i) = total_triangle_center(triIndices[i]);

        if(triangle_pressure(i) > 0.0){
            nContactingTri++;
        }
    }
    
    SimTK::Vector triangle_cenX(nTri);
    SimTK::Vector triangle_cenY(nTri);
    SimTK::Vector triangle_cenZ(nTri);

    for (int i = 0; i < nTri; ++i) {
        triangle_cenX(i) = triangle_center(i)(0);
        triangle_cenY(i) = triangle_center(i)(1);
        triangle_cenZ(i) = triangle_center(i)(2);
    }

    //Mean Pressure
    if (nContactingTri == 0) {
        stats.mean_pressure = 0.0;
        stats.mean_proximity = 0.0;
    }
    else
    {
        stats.mean_pressure = triangle_pressure.sum() / nContactingTri;
        stats.mean_proximity = triangle_proximity.sum() / nContactingTri;
    }
    //Max Pressure
    stats.max_pressure = triangle_pressure.normInf();
    stats.max_proximity = triangle_proximity.normInf();

    //Contact Area
    double contact_area = 0.0;

    for (int i = 0; i < nTri; i++) {
        if (triangle_pressure(i) > 0.0) {
            contact_area += triangle_area(i);
        }
    }

    stats.contact_area = contact_area;

    //Center of Proximity
    SimTK::Vector num_prx = triangle_proximity.elementwiseMultiply(triangle_area);
    SimTK::Vector den_prx = triangle_proximity.elementwiseMultiply(triangle_area);
    double denVal_prx = den_prx.sum();

    SimTK::Vector xNum_prx = num_prx.elementwiseMultiply(triangle_cenX);
    double xNumVal_prx = xNum_prx.sum();
    double COPrx_x = xNumVal_prx / denVal_prx;

    SimTK::Vector yNum_prx = num_prx.elementwiseMultiply(triangle_cenY);
    double yNumVal_prx = yNum_prx.sum();
    double COPrx_y = yNumVal_prx / denVal_prx;

    SimTK::Vector zNum_prx = num_prx.elementwiseMultiply(triangle_cenZ);
    double zNumVal_prx = zNum_prx.sum();
    double COPrx_z = zNumVal_prx / denVal_prx;

    if (denVal_prx == 0) {
        stats.center_of_proximity(0) = -1;
        stats.center_of_proximity(1) = -1;
        stats.center_of_proximity(2) = -1;
    }
    else {
        stats.center_of_proximity(0) = COPrx_x;
        stats.center_of_proximity(1) = COPrx_y;
        stats.center_of_proximity(2) = COPrx_z;
    }
    //Center of Pressure
    SimTK::Vector num = triangle_pressure.elementwiseMultiply(triangle_area);
    SimTK::Vector den = triangle_pressure.elementwiseMultiply(triangle_area);
    double denVal = den.sum();

    SimTK::Vector xNum = num.elementwiseMultiply(triangle_cenX);
    double xNumVal = xNum.sum();
    double COPx = xNumVal / denVal;

    SimTK::Vector yNum = num.elementwiseMultiply(triangle_cenY);
    double yNumVal = yNum.sum();
    double COPy = yNumVal / denVal;

    SimTK::Vector zNum = num.elementwiseMultiply(triangle_cenZ);
    double zNumVal = zNum.sum();
    double COPz = zNumVal / denVal;

    if (denVal_prx == 0) {
        stats.center_of_pressure(0) = -1;
        stats.center_of_pressure(1) = -1;
        stats.center_of_pressure(2) = -1;
    }
    else {
        stats.center_of_pressure(0) = COPx;
        stats.center_of_pressure(1) = COPy;
        stats.center_of_pressure(2) = COPz;
    }
    //Contact Force
    stats.contact_force = 0.0;
    stats.contact_moment = 0.0;    

    for (int i = 0; i < nTri; ++i) {
        stats.contact_force += computeContactForceVector(
            triangle_pressure(i), triangle_area(i), -triangle_normal(i));
        stats.contact_moment += computeContactMomentVector(
            triangle_pressure(i), triangle_area(i), -triangle_normal(i),
            triangle_center(i));
    }

    return stats;
}

namespace {

// Labels of the total and regional columns of one mesh, in the order written
// by getRecordValues(). Regional columns are named with the anatomical region
// names of the mesh (e.g. regional.contact_force_medial_anterior_x).
void appendMeshLabels(OpenSim::Array<std::string>& labels,
        const std::string& prefix, const std::vector<std::string>& regions) {

    labels.append(prefix + ".total.contact_area");
    labels.append(prefix + ".total.mean_proximity");
    labels.append(prefix + ".total.max_proximity");
    labels.append(prefix + ".total.center_of_proximity_x");
    labels.append(prefix + ".total.center_of_proximity_y");
    labels.append(prefix + ".total.center_of_proximity_z");
    labels.append(prefix + ".total.mean_pressure");
    labels.append(prefix + ".total.max_pressure");
    labels.append(prefix + ".total.center_of_pressure_x");
    labels.append(prefix + ".total.center_of_pressure_y");
    labels.append(prefix + ".total.center_of_pressure_z");
    labels.append(prefix + ".total.contact_force_x");
    labels.append(prefix + ".total.contact_force_y");
    labels.append(prefix + ".total.contact_force_z");
    labels.append(prefix + ".total.contact_moment_x");
    labels.append(prefix + ".total.contact_moment_y");
    labels.append(prefix + ".total.contact_moment_z");

    struct Metric { const char* name; bool vec3; };
    const Metric metrics[] = {
        {"contact_area", false}, {"mean_proximity", false},
        {"max_proximity", false}, {"center_of_proximity", true},
        {"mean_pressure", false}, {"max_pressure", false},
        {"center_of_pressure", true}, {"contact_force", true},
        {"contact_moment", true}};
    const char* axes[] = {"x", "y", "z"};

    for (const Metric& metric : metrics) {
        for (const std::string& region : regions) {
            const std::string label = prefix + ".regional." + metric.name +
                "_" + region;
            if (metric.vec3) {
                for (const char* axis : axes) {
                    labels.append(label + "_" + axis);
                }
            }
            else {
                labels.append(label);
            }
        }
    }
}

} // namespace

OpenSim::Array<std::string> Smith2018ArticularContactForce::
getRecordLabels() const {
    // Can only return casting_mesh computations because target_mesh is
    // not used in computation of force, so if the flip_meshes ModelingOption
    // is not set, target_mesh values will not be valid

    const Smith2018ContactMesh& casting_mesh =
        getSocket<Smith2018ContactMesh>("casting_mesh").getConnectee();

    const Smith2018ContactMesh& target_mesh =
        getSocket<Smith2018ContactMesh>("target_mesh").getConnectee();

    OpenSim::Array<std::string> labels("");

    appendMeshLabels(labels, getName() + "." + casting_mesh.getName(),
        casting_mesh.getRegionalNames());
    appendMeshLabels(labels, getName() + "." + target_mesh.getName(),
        target_mesh.getRegionalNames());

    return labels;
}


OpenSim::Array<double> Smith2018ArticularContactForce::
getRecordValues(const SimTK::State& state) const {
    
    if (!this->isCacheVariableValid(state, _casting_total_contact_areaCV)) {
        realizeContactMetricCaches(state);
    }

    //Total
    const double& casting_contact_area = this->getCacheVariableValue(state,
        this->_casting_total_contact_areaCV);

    const double& casting_mean_proximity = this->getCacheVariableValue(state, 
        this->_casting_total_mean_proximityCV);

    const double& casting_max_proximity = this->getCacheVariableValue(state, 
        this->_casting_total_max_proximityCV);

    const SimTK::Vec3& casting_center_of_proximity = 
        this->getCacheVariableValue(state, 
            this->_casting_total_center_of_proximityCV);

    const double& casting_mean_pressure = this->getCacheVariableValue(state, 
        this->_casting_total_mean_pressureCV);

    const double& casting_max_pressure = this->getCacheVariableValue(state, 
        this->_casting_total_max_pressureCV);

    const SimTK::Vec3& casting_center_of_pressure = 
        this->getCacheVariableValue(state, 
            this->_casting_total_center_of_pressureCV);

    const SimTK::Vec3& casting_contact_force = this->getCacheVariableValue(
        state, this->_casting_total_contact_forceCV);

    const SimTK::Vec3& casting_contact_moment =  this->getCacheVariableValue(
        state, this->_casting_total_contact_momentCV);

    // Regional
    const SimTK::Vector& casting_regional_contact_area = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_contact_areaCV);

    const SimTK::Vector& casting_regional_mean_proximity = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_mean_proximityCV);

    const SimTK::Vector& casting_regional_max_proximity = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_max_proximityCV);

    const SimTK::Vector_<SimTK::Vec3>& casting_regional_center_of_proximity = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_center_of_proximityCV);
    
    const SimTK::Vector& casting_regional_mean_pressure = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_mean_pressureCV);

    const SimTK::Vector& casting_regional_max_pressure = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_max_pressureCV);

    const SimTK::Vector_<SimTK::Vec3>& casting_regional_center_of_pressure = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_center_of_pressureCV);

    const SimTK::Vector_<SimTK::Vec3>& casting_regional_contact_force = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_contact_forceCV);

    const SimTK::Vector_<SimTK::Vec3>& casting_regional_contact_moment = 
        this->getCacheVariableValue(state, 
            this->_casting_regional_contact_momentCV);

    OpenSim::Array<double> values;

    // Regional values are written in the same order as the labels, region by
    // region within each metric
    auto appendScalars = [&values](const SimTK::Vector& v) {
        for (int r = 0; r < v.size(); ++r) { values.append(v[r]); }
    };
    auto appendVec3 = [&values](const SimTK::Vector_<SimTK::Vec3>& v) {
        for (int r = 0; r < v.size(); ++r) {
            for (int k = 0; k < 3; ++k) { values.append(v[r][k]); }
        }
    };
    
    values.append(casting_contact_area);
    values.append(casting_mean_proximity);
    values.append(casting_max_proximity); 
    values.append(casting_center_of_proximity(0));
    values.append(casting_center_of_proximity(1));
    values.append(casting_center_of_proximity(2));
    values.append(casting_mean_pressure);
    values.append(casting_max_pressure);    
    values.append(casting_center_of_pressure(0));
    values.append(casting_center_of_pressure(1));
    values.append(casting_center_of_pressure(2));
    values.append(casting_contact_force(0));
    values.append(casting_contact_force(1));
    values.append(casting_contact_force(2));
    values.append(casting_contact_moment(0));
    values.append(casting_contact_moment(1));
    values.append(casting_contact_moment(2));

    appendScalars(casting_regional_contact_area);
    appendScalars(casting_regional_mean_proximity);
    appendScalars(casting_regional_max_proximity);
    appendVec3(casting_regional_center_of_proximity);
    appendScalars(casting_regional_mean_pressure);
    appendScalars(casting_regional_max_pressure);
    appendVec3(casting_regional_center_of_pressure);
    appendVec3(casting_regional_contact_force);
    appendVec3(casting_regional_contact_moment);

    
    if (getModelingOption(state, "flip_meshes")) {
        //Total
        const double& target_contact_area = this->getCacheVariableValue(
            state, this->_target_total_contact_areaCV);

        const double& target_mean_proximity = this->getCacheVariableValue(
            state, this->_target_total_mean_proximityCV);

        const double& target_max_proximity = this->getCacheVariableValue(
            state, this->_target_total_max_proximityCV);

        const SimTK::Vec3& target_center_of_proximity = 
            this->getCacheVariableValue(state,
               this->_target_total_center_of_proximityCV);

        const double& target_mean_pressure = this->getCacheVariableValue(
            state, this->_target_total_mean_pressureCV);

        const double& target_max_pressure = this->getCacheVariableValue(
            state, this->_target_total_max_pressureCV);

        const SimTK::Vec3& target_center_of_pressure = 
            this->getCacheVariableValue(
                state, this->_target_total_center_of_pressureCV);

        const SimTK::Vec3& target_contact_force = 
            this->getCacheVariableValue(state,
                this->_target_total_contact_forceCV);

        const SimTK::Vec3& target_contact_moment = 
            this->getCacheVariableValue(state, 
                this->_target_total_contact_momentCV);
        // Regional
        const SimTK::Vector& target_regional_contact_area = 
            this->getCacheVariableValue(state,
                this->_target_regional_contact_areaCV);

        const SimTK::Vector& target_regional_mean_proximity = 
            this->getCacheVariableValue(state,
                this->_target_regional_mean_proximityCV);

        const SimTK::Vector& target_regional_max_proximity = 
            this->getCacheVariableValue(state, 
                this->_target_regional_max_proximityCV);

        const SimTK::Vector_<SimTK::Vec3>& target_regional_center_of_proximity = 
            this->getCacheVariableValue(state, 
                this->_target_regional_center_of_proximityCV);
    
        const SimTK::Vector& target_regional_mean_pressure = 
            this->getCacheVariableValue(state, 
                this->_target_regional_mean_pressureCV);

        const SimTK::Vector& target_regional_max_pressure = 
            this->getCacheVariableValue(state, 
                this->_target_regional_max_pressureCV);

        const SimTK::Vector_<SimTK::Vec3>& target_regional_center_of_pressure = 
            this->getCacheVariableValue(state, 
                this->_target_regional_center_of_pressureCV);

        const SimTK::Vector_<SimTK::Vec3>& target_regional_contact_force = 
            this->getCacheVariableValue(state, 
                this->_target_regional_contact_forceCV);

        const SimTK::Vector_<SimTK::Vec3>& target_regional_contact_moment = 
            this->getCacheVariableValue(state, 
                this->_target_regional_contact_momentCV);
    
        values.append(target_contact_area);
        values.append(target_mean_proximity);
        values.append(target_max_proximity); 
        values.append(target_center_of_proximity(0));
        values.append(target_center_of_proximity(1));
        values.append(target_center_of_proximity(2));
        values.append(target_mean_pressure);
        values.append(target_max_pressure);
        values.append(target_center_of_pressure(0));
        values.append(target_center_of_pressure(1));
        values.append(target_center_of_pressure(2));
        values.append(target_contact_force(0));
        values.append(target_contact_force(1));
        values.append(target_contact_force(2));
        values.append(target_contact_moment(0));
        values.append(target_contact_moment(1));
        values.append(target_contact_moment(2));

        appendScalars(target_regional_contact_area);
        appendScalars(target_regional_mean_proximity);
        appendScalars(target_regional_max_proximity);
        appendVec3(target_regional_center_of_proximity);
        appendScalars(target_regional_mean_pressure);
        appendScalars(target_regional_max_pressure);
        appendVec3(target_regional_center_of_pressure);
        appendVec3(target_regional_contact_force);
        appendVec3(target_regional_contact_moment);
    }
    else {
    // No Target data computed
        // 17 total columns plus 17 per region (5 scalar + 4 Vec3 metrics)
        int nTargetLabels = 17 + 17 *
            getSocket<Smith2018ContactMesh>("target_mesh").getConnectee().
            getNumRegions();

        for (int i = 0; i < nTargetLabels; i++) {
            values.append(-1);
        }
    }

    return values;
}