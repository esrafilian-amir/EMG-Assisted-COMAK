/* -------------------------------------------------------------------------- *
 *               OpenSim:  testSmith2018ArticularContactForce.cpp              *
 * -------------------------------------------------------------------------- *
 * Tests of the Smith2018ArticularContactForce and Smith2018ContactMesh with
 * simple geometry (half sphere, disk, plane): contact area, proximity,
 * pressure, force and a forward simulation.
 *
 * This code used to be at the end of testForces.cpp, where it was never
 * called. It is a separate file so that testForces.cpp stays identical to
 * upstream. The test is hidden ([.], run it with "[smith2018_legacy]") until it
 * has been checked: it has not been run since it was written.
 * -------------------------------------------------------------------------- */

#include <OpenSim/Analyses/osimAnalyses.h>
#include <OpenSim/Actuators/SpringGeneralizedForce.h>
#include <OpenSim/Simulation/osimSimulation.h>

#include <tests/Testing.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;
using namespace std;

// The assertion macros of the old OpenSim test helpers (auxiliaryTestFunctions.h),
// on top of Catch2.
namespace {
// Size of the difference between two numbers or two SimTK vectors
template <class A, class B>
double legacyDifference(const A& expected, const B& found) {
    if constexpr (std::is_arithmetic_v<A> && std::is_arithmetic_v<B>) {
        return std::abs(static_cast<double>(expected) -
                        static_cast<double>(found));
    } else {
        return (expected - found).norm();
    }
}
} // namespace
#define ASSERT_EQUAL(expected, found, tolerance, file, line, message)     do {         INFO(message);         CHECK(legacyDifference((expected), (found)) <= (tolerance));     } while (0)
#define ASSERT(condition, file, line, message)     do { INFO(message); CHECK(condition); } while (0)

/*
Test 1
Half sphere is collided with plane

 o                  o   
  o                o
    o            o 
       o  o  o
--------------------

Test 2 
Cylinder pin (flat circular end head) is collided with plane

    |        |
    |        |
    |        |
    |        |
    |________|
-------------------
T2.1: cylinder vertically penetrates plane
T2.2: cylinder slides horizontally
T2.3: cylinder twists (rotation around vertical axis)


*/
void testSmith2018ArticularContactForce() {
    Model model = Model();
    
    double gravity_acc = 9.8067;
    model.setGravity(SimTK::Vec3(0, -gravity_acc, 0));

    double mass = 1.0;
    double radius = 0.1;
    
    OpenSim::Body* plane = new OpenSim::Body("plane", 0.0, SimTK::Vec3(0),
        mass * SimTK::Inertia::brick(0.05, 0.05, 0.05));

    OpenSim::Body* indenter = new OpenSim::Body("indenter", mass, SimTK::Vec3(0),
        mass * SimTK::Inertia::brick(0.05, 0.05, 0.05));

    //indenter->attachGeometry(new Mesh("half_sphere_10cm_radius.stl"));

    model.addBody(plane);
    model.addBody(indenter);

    SpatialTransform indenter_transform = SpatialTransform();
    indenter_transform.upd_rotation1().set_coordinates(0, "plane_indenter_rx");
    indenter_transform.upd_rotation1().set_axis(SimTK::Vec3(1, 0, 0));
    indenter_transform.upd_rotation1().set_function(LinearFunction());

    indenter_transform.upd_rotation2().set_coordinates(0, "plane_indenter_ry");
    indenter_transform.upd_rotation2().set_axis(SimTK::Vec3(0, 1, 0));
    indenter_transform.upd_rotation2().set_function(LinearFunction());

    indenter_transform.upd_rotation3().set_coordinates(0, "plane_indenter_rz");
    indenter_transform.upd_rotation3().set_axis(SimTK::Vec3(0, 0, 1));
    indenter_transform.upd_rotation3().set_function(LinearFunction());

    indenter_transform.upd_translation1().set_coordinates(0, "plane_indenter_tx");
    indenter_transform.upd_translation1().set_axis(SimTK::Vec3(1, 0, 0));
    indenter_transform.upd_translation1().set_function(LinearFunction());

    indenter_transform.upd_translation2().set_coordinates(0, "plane_indenter_ty");
    indenter_transform.upd_translation2().set_axis(SimTK::Vec3(0, 1, 0));
    indenter_transform.upd_translation2().set_function(LinearFunction());

    indenter_transform.upd_translation3().set_coordinates(0, "plane_indenter_tz");
    indenter_transform.upd_translation3().set_axis(SimTK::Vec3(0, 0, 1));
    indenter_transform.upd_translation3().set_function(LinearFunction());

    WeldJoint* ground_plane = new WeldJoint("ground_plane", model.getGround(), *plane);
    CustomJoint* plane_indenter = new CustomJoint("plane_indenter", *plane, *indenter, indenter_transform);

    model.addJoint(ground_plane);
    model.addJoint(plane_indenter);

    Smith2018ContactMesh* plane_mesh = new Smith2018ContactMesh("plane", "x_z_plane.stl", *plane);
    Smith2018ContactMesh* indenter_mesh = new Smith2018ContactMesh("indenter", "half_sphere_10cm_radius.stl", *indenter);

    double E_indenter = 1e6;
    double v_indenter = 0.46;
    double E_plane = 1e6;
    double v_plane = 0.46;

    plane_mesh->set_elastic_modulus(E_plane);
    plane_mesh->set_poissons_ratio(v_plane);
    plane_mesh->set_thickness(0.2);

    indenter_mesh->set_elastic_modulus(E_indenter);
    indenter_mesh->set_poissons_ratio(v_indenter);
    indenter_mesh->set_thickness(0.1);

    model.addContactGeometry(plane_mesh);
    model.addContactGeometry(indenter_mesh);

    Smith2018ArticularContactForce* contact = new Smith2018ArticularContactForce("contact", *plane_mesh, *indenter_mesh);
    contact->set_max_proximity(0.2);
    contact->set_elastic_foundation_formulation("linear");
    contact->set_use_lumped_contact_model(true);
    model.addForce(contact);

    //model.setUseVisualizer(true);
    SimTK::State& state = model.initSystem();
    contact->setModelingOption(state, "flip_meshes", 1);

    model.print("contact_test.osim");

    Coordinate& tx = model.updCoordinateSet().get("plane_indenter_tx");
    Coordinate& ty = model.updCoordinateSet().get("plane_indenter_ty");
    Coordinate& tz = model.updCoordinateSet().get("plane_indenter_tz");
    Coordinate& rx = model.updCoordinateSet().get("plane_indenter_rx");
    Coordinate& ry = model.updCoordinateSet().get("plane_indenter_ry");
    Coordinate& rz = model.updCoordinateSet().get("plane_indenter_rz");

    model.realizeReport(state);

    //const ModelVisualizer& viz = model.getVisualizer();
    
    // Verify Contact Area
    double expected_casting_area = 2 * SimTK::Pi * SimTK::square(radius);
    double reported_casting_area = contact->getOutputValue<double>(
            state, "casting_total_contact_area");

     ASSERT_EQUAL(expected_casting_area, reported_casting_area, 1e-3,
     __FILE__, __LINE__,
            "Expected the contact area of the half sphere to be equal to the "
            "analytical formula.");

    double expected_target_area = SimTK::Pi * SimTK::square(radius);
    double reported_target_area =
            contact->getOutputValue<double>(state, "target_total_contact_area");

     ASSERT_EQUAL(expected_target_area, reported_target_area, 1e-3, __FILE__,
     __LINE__,
            "Expected the contact area of the half sphere to be equal to the "
            "analytical formula.");
       
    //Move half sphere to half depth
     double depth = 0.05;
    ty.setValue(state, depth);
    model.realizeReport(state);

    // Max Proximity
    double reported_casting_max_prox = contact->getOutputValue<double>(
            state, "casting_total_max_proximity");

    double reported_target_max_prox = contact->getOutputValue<double>(
            state, "target_total_max_proximity");

    double expected_max_prox = radius/2;

    ASSERT_EQUAL(expected_max_prox, reported_casting_max_prox, 1e-3, __FILE__,
            __LINE__,
            "Expected the maximum casting proximity "
            "to be equal to the half sphere radius.");

    ASSERT_EQUAL(expected_max_prox, reported_target_max_prox, 1e-3, __FILE__,
            __LINE__,
            "Expected the maximum target proximity "
            "to be equal to the half sphere radius.");
    
    //Test 2
    //===================================================================
    indenter_mesh->set_mesh_file("x_z_disk_10cm_radius.stl");    

    model.finalizeFromProperties();

    state = model.initSystem();
    contact->setModelingOption(state, "flip_meshes", 1);
    model.realizeReport(state);
    
    depth = 0.01;
    ty.setValue(state, (0.0 - depth));
    model.realizeReport(state);


    //Proximity

    double reported_max_prox = contact->getOutputValue<double>(
            state, "casting_total_max_proximity");

    double reported_mean_prox = contact->getOutputValue<double>(
            state, "casting_total_mean_proximity");

    ASSERT_EQUAL(depth, reported_max_prox, 1e-3, __FILE__,
            __LINE__,
            "Expected the max proximity to be equal to indentation depth "
            "of flat cylinder bottom.");
    
    ASSERT_EQUAL(reported_mean_prox, reported_max_prox, 1e-3, __FILE__,
            __LINE__,
            "Expected the mean and max proximity to be equal because "
            "the cylinder bottom is flat.");

    // Pressure
    double reported_casting_total_max_pressure = contact->getOutputValue<double>(
            state, "casting_total_max_pressure");

    double reported_mean_pressure = contact->getOutputValue<double>(
            state, "casting_total_mean_pressure");

    double area = contact->getOutputValue<double>(
        state, "casting_total_contact_area");

    SimTK::Vec3 reported_force = contact->getOutputValue<SimTK::Vec3>(
            state, "casting_total_contact_force");

    double expected_pressure = reported_force.norm() / area;

    ASSERT_EQUAL(expected_pressure, reported_mean_pressure, 1e-3, __FILE__, __LINE__,
            "Expected the mean pressure to be equal to force/area "
            "for flat disk.");

    ASSERT_EQUAL(reported_mean_pressure, reported_casting_total_max_pressure, 1e-3, __FILE__,
            __LINE__,
            "Expected the mean and max pressure to be equal because "
            "the disk is flat.");

    // Force
    SimTK::Vec3 reported_moment = contact->getOutputValue<SimTK::Vec3>(
        state, "casting_total_contact_moment");

    SimTK::Vec3 reported_COprox = contact->getOutputValue<SimTK::Vec3>(
        state, "casting_total_center_of_proximity");

    SimTK::Vec3 reported_COpressure = contact->getOutputValue<SimTK::Vec3>(
        state, "casting_total_center_of_pressure");

    ASSERT_EQUAL(0., reported_force(0), 1e-3, __FILE__, __LINE__,
        "Expected the x (horizontal) contact force component to be "
        "zero for flat disk contacting plane.");

    ASSERT_EQUAL(0., reported_force(2), 1e-3, __FILE__, __LINE__,
        "Expected the z (horizontal) contact force component to be "
        "zero for flat disk contacting plane.");

    ASSERT_EQUAL(0., reported_moment.norm(), 1e-3, __FILE__, __LINE__,
        "Expected the contact moment to be "
        "zero for flat disk contacting plane.");

    ASSERT_EQUAL(reported_COprox, reported_COpressure, 1e-3, __FILE__, __LINE__,
        "Expected the center of proximity and pressure to be the same "
        "flat disk contacting plane.");

    // Test Regional Metrics

    SimTK::Vec3 reported_target_force = contact->getOutputValue<SimTK::Vec3>(
        state, "target_total_contact_force");

    SimTK::Vec3 reported_target_moment = contact->getOutputValue<SimTK::Vec3>(
        state, "target_total_contact_moment");

    SimTK::Vec3 reported_target_COprox = contact->getOutputValue<SimTK::Vec3>(
        state, "target_total_center_of_proximity");

    SimTK::Vec3 reported_target_COpressure = contact->getOutputValue<SimTK::Vec3>(
        state, "target_total_center_of_pressure");

    reported_target_area = contact->getOutputValue<double>(
        state, "target_total_contact_area");

    SimTK::Vector_<SimTK::Vec3> reported_target_regional_force = contact->getOutputValue<SimTK::Vector_<SimTK::Vec3>>(
        state, "target_regional_contact_force");

    SimTK::Vector_<SimTK::Vec3> reported_target_regional_moment = contact->getOutputValue<SimTK::Vector_<SimTK::Vec3>>(
        state, "target_regional_contact_moment");

    SimTK::Vector_<SimTK::Vec3> reported_target_regional_COprox = contact->getOutputValue<SimTK::Vector_<SimTK::Vec3>>(
        state, "target_regional_center_of_proximity");

    SimTK::Vector_<SimTK::Vec3> reported_target_regional_COpressure = contact->getOutputValue<SimTK::Vector_<SimTK::Vec3>>(
        state, "target_regional_center_of_pressure");

    SimTK::Vector reported_target_regional_area = contact->getOutputValue<SimTK::Vector>(
        state, "target_regional_contact_area");

    //regional area
    ASSERT_EQUAL(
        reported_target_regional_area(0) + reported_target_regional_area(1),
        reported_target_area, 1e-3, __FILE__, __LINE__,
        "Expected the regional contact area in the +x and -x regions (0,1) "
        "to sum to the total contact area.");

    ASSERT_EQUAL(
        reported_target_regional_area(4) + reported_target_regional_area(5),
        reported_target_area, 1e-3, __FILE__, __LINE__,
        "Expected the regional contact area in the +z and -z regions (4,5) "
        "to sum to the total contact area.");

    ASSERT_EQUAL(
        reported_target_regional_force(0) + reported_target_regional_force(1),
        reported_target_force, 1e-3, __FILE__, __LINE__,
        "Expected the regional contact force in the +x and -x regions (0,1) "
        "to sum to the total contact force.");

    ASSERT_EQUAL(
        reported_target_regional_force(4) + reported_target_regional_force(5),
        reported_target_force, 1e-3, __FILE__, __LINE__,
        "Expected the regional contact force in the +z and -z regions (4,5) "
        "to sum to the total contact force.");

    ASSERT_EQUAL(
        reported_target_regional_moment(0) + reported_target_regional_moment(1),
        reported_target_moment, 1e-3, __FILE__, __LINE__,
        "Expected the regional contact moment in the +x and -x regions (0,1) "
        "to sum to the total contact moment.");

    ASSERT_EQUAL(
        reported_target_regional_moment(4) + reported_target_regional_moment(5),
        reported_target_moment, 1e-3, __FILE__, __LINE__,
        "Expected the regional contact moment in the +z and -z regions (4,5) "
        "to sum to the total contact moment.");
    
    // Translate +x +z to test different regions
    SimTK::Vector tx_values = SimTK::Vector(2, -1);
    tx_values.set(0, -0.2);
    tx_values.set(1, 0.2);

    SimTK::Vector tz_values = SimTK::Vector(2, -1);
    tz_values.set(0, -0.2);
    tz_values.set(1, 0.2);


    for (int i = 0; i < tx_values.size(); ++i) {
        for (int j = 0; j < tz_values.size(); ++j) {
            tx.setValue(state, tx_values(i));
            tz.setValue(state, tz_values(j));
            model.realizeReport(state);

            reported_target_force = contact->getOutputValue<SimTK::Vec3>(
                state, "target_total_contact_force");

            reported_target_moment = contact->getOutputValue<SimTK::Vec3>(
                state, "target_total_contact_moment");

            reported_target_COprox = contact->getOutputValue<SimTK::Vec3>(
                state, "target_total_center_of_proximity");

            reported_target_COpressure = contact->getOutputValue<SimTK::Vec3>(
                state, "target_total_center_of_pressure");

            ASSERT_EQUAL(tx_values(i), reported_target_COpressure(0), 
                1e-3, __FILE__, __LINE__,
                "Expected the plane COP x component to be located at the "
                "center ofthe flat disk.");

            ASSERT_EQUAL(tz_values(j), reported_target_COpressure(2), 
                1e-3, __FILE__, __LINE__,
                "Expected the plane COP z component to be located at the "
                "center of the flat disk.");

            SimTK::Vec3 expected_moment = reported_target_COpressure % reported_target_force;

            ASSERT_EQUAL(expected_moment, reported_target_moment, 1e-3, __FILE__, __LINE__,
                "Expected the contact moment to be equal to COP x Force .");

            // Regional Metrics

            reported_target_regional_force = contact->getOutputValue<SimTK::Vector_<SimTK::Vec3>>(
                state, "target_regional_contact_force");

            reported_target_regional_moment = contact->getOutputValue<SimTK::Vector_<SimTK::Vec3>>(
                state, "target_regional_contact_moment");

            reported_target_regional_COprox = contact->getOutputValue<SimTK::Vector_<SimTK::Vec3>>(
                state, "target_regional_center_of_proximity");

            reported_target_regional_COpressure = contact->getOutputValue<SimTK::Vector_<SimTK::Vec3>>(
                state, "target_regional_center_of_pressure");

            ASSERT_EQUAL(reported_target_regional_COprox(0),
                reported_target_regional_COpressure(0), 1e-3, __FILE__, __LINE__,
                "Expected the regional center of pressure and proximity "
                "to be equal for flat disk.");

            if (tx_values(i) < 0.0) {
                ASSERT_EQUAL(reported_target_regional_COpressure(0)(0), tx_values(i), 1e-3, __FILE__, __LINE__,
                    "Expected the region 0 plane COP x component to be "
                    "located at the center of the flat disk when contact is "
                    "in -x half space");
                ASSERT_EQUAL(reported_target_regional_COpressure(1), SimTK::Vec3(-1), 1e-3, __FILE__, __LINE__,
                    "Expected the region 1 plane COP to be -1 when contact is "
                    "in -x half space.");
                ASSERT_EQUAL(reported_target_regional_force(0), reported_target_force, 1e-3, __FILE__, __LINE__,
                    "Expected the region 0 plane force to be equal to the "
                    "total force when contact is only in -x half space");
                ASSERT_EQUAL(reported_target_regional_force(1), SimTK::Vec3(0), 1e-3, __FILE__, __LINE__,
                    "Expected the region 1 plane force to be 0 when contact is "
                    "in -x half space.");
                ASSERT_EQUAL(reported_target_regional_moment(0), reported_target_moment, 1e-3, __FILE__, __LINE__,
                    "Expected the region 0 plane moment to be equal to the "
                    "total force when contact is only in -x half space");
                ASSERT_EQUAL(reported_target_regional_moment(1), SimTK::Vec3(0), 1e-3, __FILE__, __LINE__,
                    "Expected the region 1 plane moment to be 0 when contact is "
                    "in -x half space.");
            }
            else {
                ASSERT_EQUAL(reported_target_regional_COpressure(1)(0), tx_values(i), 1e-3, __FILE__, __LINE__,
                    "Expected the region 1 plane COP x component to be "
                    "located at the center of the flat disk when contact is "
                    "in +x half space");
                ASSERT_EQUAL(reported_target_regional_COpressure(0), SimTK::Vec3(-1), 1e-3, __FILE__, __LINE__,
                    "Expected the region 0 plane COP to be -1 when contact is "
                    "in +x half space.");
                ASSERT_EQUAL(reported_target_regional_force(1), reported_target_force, 1e-3, __FILE__, __LINE__,
                    "Expected the region 0 plane force to be equal to the "
                    "total force when contact is only in +x half space");
                ASSERT_EQUAL(reported_target_regional_force(0), SimTK::Vec3(0), 1e-3, __FILE__, __LINE__,
                    "Expected the region 1 plane force to be 0 when contact is "
                    "in +x half space.");
                ASSERT_EQUAL(reported_target_regional_moment(1), reported_target_moment, 1e-3, __FILE__, __LINE__,
                    "Expected the region 0 plane moment to be equal to the "
                    "total force when contact is only in +x half space");
                ASSERT_EQUAL(reported_target_regional_moment(0), SimTK::Vec3(0), 1e-3, __FILE__, __LINE__,
                    "Expected the region 1 plane moment to be 0 when contact is "
                    "in +x half space.");
            }

            if (tz_values(j) < 0.0) {
                ASSERT_EQUAL(reported_target_regional_COpressure(4)(2), 
                    tz_values(j), 1e-3, __FILE__, __LINE__,
                    "Expected the region 4 plane COP z component to be "
                    "located at the center of the flat disk when contact is "
                    "in -z half space");
                ASSERT_EQUAL(reported_target_regional_COpressure(5), 
                    SimTK::Vec3(-1), 1e-3, __FILE__, __LINE__,
                    "Expected the region 5 plane COP to be -1 when contact is "
                    "in -z half space.");
                ASSERT_EQUAL(reported_target_regional_force(4), 
                    reported_target_force, 1e-3, __FILE__, __LINE__,
                    "Expected the region 4 plane force to be equal to the "
                    "total force when contact is only in -z half space");
                ASSERT_EQUAL(reported_target_regional_force(5), 
                    SimTK::Vec3(0), 1e-3, __FILE__, __LINE__,
                    "Expected the region 5 plane force to be 0 when contact is"
                    " in -z half space.");
                ASSERT_EQUAL(reported_target_regional_moment(4), 
                    reported_target_moment, 1e-3, __FILE__, __LINE__,
                    "Expected the region 4 plane moment to be equal to the "
                    "total force when contact is only in -z half space");
                ASSERT_EQUAL(reported_target_regional_moment(5), 
                    SimTK::Vec3(0), 1e-3, __FILE__, __LINE__,
                    "Expected the region 5 plane moment to be 0 when contact "
                    "is in -z half space.");
            }
            else {
                ASSERT_EQUAL(reported_target_regional_COpressure(5)(2), tz_values(j), 1e-3, __FILE__, __LINE__,
                    "Expected the region 5 plane COP z component to be "
                    "located at the center of the flat disk when contact is "
                    "in +z half space");
                ASSERT_EQUAL(reported_target_regional_COpressure(4), SimTK::Vec3(-1), 1e-3, __FILE__, __LINE__,
                    "Expected the region 4 plane COP to be -1 when contact is "
                    "in +z half space.");
                ASSERT_EQUAL(reported_target_regional_force(5), reported_target_force, 1e-3, __FILE__, __LINE__,
                    "Expected the region 4 plane force to be equal to the "
                    "total force when contact is only in +z half space");
                ASSERT_EQUAL(reported_target_regional_force(4), SimTK::Vec3(0), 1e-3, __FILE__, __LINE__,
                    "Expected the region 5 plane force to be 0 when contact is "
                    "in +z half space.");
                ASSERT_EQUAL(reported_target_regional_moment(5), reported_target_moment, 1e-3, __FILE__, __LINE__,
                    "Expected the region 4 plane moment to be equal to the "
                    "total force when contact is only in +z half space");
                ASSERT_EQUAL(reported_target_regional_moment(4), SimTK::Vec3(0), 1e-3, __FILE__, __LINE__,
                    "Expected the region 5 plane moment to be 0 when contact is "
                    "in +z half space.");
            }                
        }
    }

    // Pressure - depth relationship
    double depth1 = 0;
    double depth2 = 0.01;
    double depth3 = 0.02;

    tx.setValue(state, 0.0);
    tz.setValue(state, 0.0);
    ty.setValue(state, -depth1);

    model.realizeReport(state);

    double reported_casting_total_max_pressure1 = contact->getOutputValue<double>(
        state, "casting_total_max_pressure");
    double reported_target_total_max_pressure1 = contact->getOutputValue<double>(
        state, "target_total_max_pressure");

    ty.setValue(state, -depth2);
    model.realizeReport(state);

    double reported_casting_total_max_pressure2 = contact->getOutputValue<double>(
        state, "casting_total_max_pressure");
    double reported_target_total_max_pressure2 = contact->getOutputValue<double>(
        state, "target_total_max_pressure");

    ty.setValue(state, -depth3);
    model.realizeReport(state);

    double reported_casting_total_max_pressure3 = contact->getOutputValue<double>(
        state, "casting_total_max_pressure");
    double reported_target_total_max_pressure3 = contact->getOutputValue<double>(
        state, "target_total_max_pressure");

    double casting_pressure_diff_21 = 
        reported_casting_total_max_pressure2 - reported_casting_total_max_pressure1;
    double target_pressure_diff_21 = 
        reported_target_total_max_pressure2 - reported_target_total_max_pressure1;

    double casting_pressure_diff_32 =
        reported_casting_total_max_pressure3 - reported_casting_total_max_pressure2;
    double target_pressure_diff_32 =
        reported_target_total_max_pressure3 - reported_target_total_max_pressure2;

    ASSERT_EQUAL(casting_pressure_diff_21, casting_pressure_diff_32, 
        1e-3, __FILE__, __LINE__,
        "Expected the pressure depth relationship to be linear when "
        "elastic_foundation_formulation is 'linear'.");

    // Run Forward Simulation
    
    // two disks - different area
    // settled contact force = gravity
    // settled depth of model1 > model2

    for (Coordinate& coord : model.updComponentList<Coordinate>()) {
        coord.setValue(state, 0);
        coord.set_locked(true);
    }
    ty.set_locked(false);
    
    indenter->setMass(10.0);
    
    state = model.initSystem();
    ty.setValue(state, -0.01);
   
    SimTK::CPodesIntegrator integrator(model.getSystem(), 
        SimTK::CPodes::BDF, SimTK::CPodes::Newton);
    integrator.setAccuracy(1e-8);
    integrator.setMaximumStepSize(1e-3);
    SimTK::TimeStepper timestepper(model.getSystem(), integrator);
    timestepper.initialize(state);
    
    state.setTime(0.0);
    model.realizeReport(state);    

    double KE0 = model.calcKineticEnergy(state);
    double PE0 = model.calcPotentialEnergy(state);
    double E0 = KE0 + PE0;

    // Integrate Forward in Time
    double dt = 0.01;
    double stop_time = 2.0;
    int nSteps = (int)lround(stop_time / dt);

    StatesTrajectory states_trj;
    states_trj.append(state);

    for (int i = 0; i <= nSteps; ++i) {
        double t = (i + 1) * dt;

        timestepper.stepTo(t);
        state = timestepper.getState();        

        // Record Result
        model.realizeReport(state);
        states_trj.append(state);
    }

    double KE1 = model.calcKineticEnergy(state);
    double PE1 = model.calcPotentialEnergy(state);
    double E1 = KE1 + PE1;

    ASSERT_EQUAL(E0, E1,
        1e-3, __FILE__, __LINE__,
        "Expected energy to be conserved with contact force and no damping.");

    model.print("test.osim");
    STOFileAdapter().write(states_trj.exportToTable(model),"test_states.sto");

    //Add damper to settle contact
    SpringGeneralizedForce* sgf = new SpringGeneralizedForce("plane_indenter_ty");
    sgf->setName("damper");
    sgf->setViscosity(200.0);
    model.addForce(sgf);

    state = model.initSystem();
    //contact->setModelingOption(state, "flip_meshes", 1);
    ty.setValue(state, 0);
    SimTK::CPodesIntegrator integrator1(model.getSystem(),
        SimTK::CPodes::BDF, SimTK::CPodes::Newton);
    integrator1.setAccuracy(1e-4);
    integrator1.setMaximumStepSize(1e-3);
    SimTK::TimeStepper timestepper1(model.getSystem(), integrator1);
    timestepper1.initialize(state);

    StatesTrajectory states_trj1;
    states_trj1.append(state);

    for (int i = 0; i <= nSteps; ++i) {
        double t = (i + 1) * dt;

        timestepper1.stepTo(t);
        state = timestepper1.getState();

        // Record Result
        model.realizeReport(state);
        states_trj1.append(state);
    }
    model.print("test.osim");
    STOFileAdapter().write(states_trj1.exportToTable(model), "test_states1.sto");

    double settle_depth1 = ty.getValue(state);

    SimTK::Vec3 reported_casting_force = contact->getOutputValue<SimTK::Vec3>(
        state, "casting_total_contact_force");

    double weight = indenter->getMass() * gravity_acc;
    double KE2 = model.calcKineticEnergy(state);


    ASSERT_EQUAL(reported_casting_force.norm(), weight,
        1e-3, __FILE__, __LINE__,
        "Expected the settled indenter contact force to be equal to the "
        "weight of the indenter.");

    // TEST 3 - Ray intersection tests
    // ========================================================================
    // half sphere radius 10 cm, center at (0,0,0), top half (+y) deleted

    Smith2018ContactMesh mesh = 
        Smith2018ContactMesh("half_sphere","half_sphere_10cm_radius.stl");

    mesh.finalizeFromProperties();

    SimTK::Vec3 origin(0, -0.2, 0);
    SimTK::UnitVec3 ray(0, 0.5, 0);
    double min_proximity = 0.0;
    double max_proximity = 0.1;

    int tri_index;
    SimTK::Vec3 intersect_pt;
    double distance;
    
    bool ray_hit = mesh.rayIntersectMesh(origin, ray, 
            min_proximity, max_proximity, tri_index, intersect_pt, distance);
    
    ASSERT(ray_hit, __FILE__, __LINE__,
        "Expected the ray to intersect the half sphere. ");

    double expected_distance = 0.1; // origin y - sphere radius
    ASSERT_EQUAL(distance, expected_distance,
        1e-3, __FILE__, __LINE__,
        "The distance along the ray to contact with sphere to be equal to the "
        "ray origin minus the sphere radius.");

    // Flip ray
    ray = SimTK::UnitVec3(0,-0.5,0);

    ray_hit = mesh.rayIntersectMesh(origin, ray,
        min_proximity, max_proximity, tri_index, intersect_pt, distance);
    
    ASSERT(!ray_hit, __FILE__, __LINE__,
        "Expected the ray pointed away from half sphere to fail the "
        "rayIntersectMesh test. ");

    // move origin
    origin = SimTK::Vec3(0, 0, 0);
    ray_hit = mesh.rayIntersectMesh(origin, ray,
        min_proximity, max_proximity, tri_index, intersect_pt, distance);
    
    ASSERT(!ray_hit, __FILE__, __LINE__,
        "Expected the ray pointed from inside from half sphere to hit "
        "a back face and fail the rayIntersectMesh test. ");
        
    // Test 4 - test constructor from vertices and tri

    SimTK::PolygonalMesh ply_mesh;
    ply_mesh.loadFile("half_sphere_10cm_radius.stl");

    SimTK::Vector_<SimTK::Vec3> vertices(ply_mesh.getNumVertices());
    for (int v = 0; v < ply_mesh.getNumVertices(); ++v) {
        vertices.set(v, ply_mesh.getVertexPosition(v));
    }
    
    //SimTK::Vector_<SimTK::Vec3> triangles;
    std::vector<std::vector<int>> triangles;
    for (int t = 0; t < ply_mesh.getNumFaces(); ++t) {
        std::vector<int> face_vertex(3);
        for (int j = 0; j < 3; j++) {
             face_vertex[j] = ply_mesh.getFaceVertex(t, j);
        }
        triangles.push_back(face_vertex);
    }

    Smith2018ContactMesh t4_mesh = Smith2018ContactMesh("half_sphere",vertices,triangles);
    
    t4_mesh.finalizeFromProperties();

    origin = SimTK::Vec3 (0, -0.2, 0);
    ray = SimTK::UnitVec3 (0, 0.5, 0);

    ray_hit = t4_mesh.rayIntersectMesh(origin, ray,
        min_proximity, max_proximity, tri_index, intersect_pt, distance);
    
    ASSERT(ray_hit, __FILE__, __LINE__,
        "Expected the ray to intersect the half sphere. ");
}


TEST_CASE("testSmith2018ArticularContactForce", "[.][smith2018_legacy]") {
    testSmith2018ArticularContactForce();
}
