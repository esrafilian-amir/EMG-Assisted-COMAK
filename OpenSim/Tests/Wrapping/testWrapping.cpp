/* -------------------------------------------------------------------------- *
 *                         OpenSim:  testWrapping.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

#include <OpenSim/OpenSim.h>
#include <OpenSim/Actuators/ModelOperators.h>
#include <SimTKcommon.h>

#define CATCH_CONFIG_MAIN
#include <OpenSim/Auxiliary/catch.hpp>

using namespace OpenSim;
using namespace SimTK;
using namespace std;
using OpenSim::TimeSeriesTable;

// HELPER FUNCTIONS
namespace {
    // Simulate a model from an initial time to a final time given an initial
    // state.
    void simulate(Model& model, State& state, double initialTime, 
            double finalTime)
    {
        // Create the Manager for the simulation.
        const double accuracy = 1.0e-4;
        Manager manager(model);
        manager.setIntegratorAccuracy(accuracy);
    
        // Integrate from initial time to final time.
        state.setTime(initialTime);
        cout << "\nIntegrating from " << initialTime << " to " 
             << finalTime << endl;
    
        const double start = SimTK::realTime();
        manager.initialize(state);
        manager.integrate(finalTime);
        cout << "Simulation time = " << SimTK::realTime() - start
             << " seconds (wallclock time)\n"
             << endl;
    
        auto& integrator = manager.getIntegrator();
        cout << "Integrator iterations = " 
             << integrator.getNumStepsTaken() << endl;
    
        // Save the simulation results.
        Storage states(manager.getStateStorage());
        states.print(model.getName() + "_states.sto");
        model.updSimbodyEngine().convertRadiansToDegrees(states);
        states.setWriteSIMMHeader(true);
        states.print(model.getName() + "_states_degrees.mot");
    }

    // Simulate a model with muscles at a constant activation to a given final
    // time.
    void simulateModelWithMuscles(
            const string& modelFile, double finalTime, double activation = 0.5) 
    {
        // Create a new OpenSim model.
        Model model(modelFile);
        
        // Create a PrescribedController that simply applies a function of the
        // force.
        PrescribedController actuController;
        actuController.setActuators(model.updActuators());
        for (int i = 0; i < actuController.getActuatorSet().getSize(); i++) {
            actuController.prescribeControlForActuator(
                    i, new Constant(activation));
        }
    
        // Add the controller to the model. We need to call disownAllComponents
        // because PrescribedController is on the stack.
        model.addController(&actuController);
        model.disownAllComponents();
        model.finalizeFromProperties();
        model.printBasicInfo();
    
        // Initialize the system and get the state representing the state system.
        SimTK::State& state = model.initSystem();
    
        // Apply the activations to the muscles and equilibrate.
        const Set<Muscle>& muscles = model.getMuscles();
        for (int i = 0; i < muscles.getSize(); i++) {
            muscles[i].setActivation(state, activation);
        }
        model.equilibrateMuscles(state);
    
        // Simulate.
        simulate(model, state, 0.0, finalTime);
    }
    
    // Given a TimeSeriesTable of coordinate values, create a TimeSeriesTable of
    // muscle lengths.
    TimeSeriesTable createMuscleLengthsTable(const OpenSim::Model& model,
            const TimeSeriesTable& coordinates) 
    {
        auto statesTraj = OpenSim::StatesTrajectory::createFromStatesTable(
                model, coordinates, true, true, true);
        TimeSeriesTable lengths;
        const auto& muscleSet = model.getMuscles();
        for (const auto& state : statesTraj) {
            model.realizePosition(state);
            SimTK::RowVector lengthsRow(muscleSet.getSize());
            for (int imuscle = 0; imuscle < muscleSet.getSize(); ++imuscle) {
                const auto& muscle = muscleSet.get(imuscle);
                const auto& geometryPath = muscle.getGeometryPath();
                lengthsRow[imuscle] = geometryPath.getLength(state);
            }
            lengths.appendRow(state.getTime(), lengthsRow);
        }
        std::vector<std::string> labels;
        for (int imuscle = 0; imuscle < muscleSet.getSize(); ++imuscle) {
            const auto& muscle = muscleSet.get(imuscle);
            const auto& path = muscle.getAbsolutePathString();
            labels.push_back(path + "|length");
        }
        lengths.setColumnLabels(labels);
        
        return lengths;
    }
}

TEST_CASE("testWrapCylinder") {
    const double r = 0.25;
    const double off = sqrt(2)*r-0.05;
    Model model;
    model.setName("testWrapCylinder");

    auto& ground = model.updGround();
    auto body = new OpenSim::Body("body", 1, Vec3(0), Inertia(0.1, 0.1, 0.01));
    model.addComponent(body);

    auto bodyOffset = new PhysicalOffsetFrame(
            "bToj", *body, Transform(Vec3(-off, 0, 0)));
    model.addComponent(bodyOffset);
    
    auto joint = new PinJoint("pin", ground, *bodyOffset);
    model.addComponent(joint);

    WrapCylinder* pulley1 = new WrapCylinder();
    pulley1->setName("pulley1");
    pulley1->set_radius(r);
    pulley1->set_length(0.05);

    // Add the wrap object to the body, which takes ownership of it
    ground.addWrapObject(pulley1);

    // One spring has wrap cylinder with respect to ground origin
    PathSpring* spring1 =
        new PathSpring("spring1", 1.0, 0.1, 0.01);
    spring1->updGeometryPath().
        appendNewPathPoint("origin", ground, Vec3(-off, 0, 0));
    spring1->updGeometryPath().
        appendNewPathPoint("insert", *body, Vec3(0));
    spring1->updGeometryPath().addPathWrap(*pulley1);

    model.addComponent(spring1);

    WrapCylinder* pulley2 = new WrapCylinder();
    pulley2->setName("pulley2");
    pulley2->set_radius(r);
    pulley2->set_length(0.05);

    // Add the wrap object to the body, which takes ownership of it
    bodyOffset->addWrapObject(pulley2);

    // Second spring has wrap cylinder with respect to bodyOffse origin
    PathSpring* spring2 =
        new PathSpring("spring2", 1.0, 0.1, 0.01);
    spring2->updGeometryPath().
        appendNewPathPoint("origin", ground, Vec3(-off, 0, 0));
    spring2->updGeometryPath().
        appendNewPathPoint("insert", *body, Vec3(0));
    spring2->updGeometryPath().addPathWrap(*pulley2);
    spring2->updGeometryPath().setDefaultColor(Vec3(0, 0.8, 0));

    model.addComponent(spring2);
    
    SimTK::State& s = model.initSystem();
    auto& coord = joint->updCoordinate();

    int nsteps = 10;
    for (int i = 0; i < nsteps; ++i) {
        
        coord.setValue(s, i*SimTK::Pi/(2*nsteps));
        model.realizeVelocity(s);
        
        double ma1 = spring1->computeMomentArm(s, coord);
        double ma2 = spring2->computeMomentArm(s, coord);

        CHECK_THAT(r - ma1, Catch::WithinAbs(0.0, SimTK::Eps));
        CHECK_THAT(r - ma2, Catch::WithinAbs(0.0, SimTK::Eps));

        double len1 = spring1->getLength(s);
        double len2 = spring2->getLength(s);

        CHECK_THAT(len1 - len2, Catch::WithinAbs(0.0, SimTK::Eps));
    }
}

TEST_CASE("testShoulderWrapping") {
    // Test the performance of multiple paths with wrapping in the 
    // upper-extremity.
    simulateModelWithMuscles("TestShoulderWrapping.osim", 0.1, 0.5);
}

TEST_CASE("testWrapObjectUpdateFromXMLNode30515") {
    // In XMLDocument version 30515, we converted VisibleObject, color and
    // display_preference properties to Appearance properties.
    XMLDocument doc("testWrapObject_updateFromXMLNode30515.osim");

    // Make sure this test is not weakened by the model in the repository being
    // updated.
    SimTK_TEST(doc.getDocumentVersion() == 20302);
    Model model("testWrapObject_updateFromXMLNode30515.osim");
    model.print("testWrapObject_updateFromXMLNode30515_updated.osim");
    const auto& wrapObjSet = model.getGround().getWrapObjectSet();

    // WrapSphere has:
    //   display_preference = 1
    //   color = default
    //   VisibleObject display_preference = 0
    {
        const auto& sphere = wrapObjSet.get("wrapsphere");
        SimTK_TEST(!sphere.get_Appearance().get_visible());
        SimTK_TEST_EQ(sphere.get_Appearance().get_color(), SimTK::Cyan);
        SimTK_TEST(sphere.get_Appearance().get_representation() == 
                VisualRepresentation::DrawPoints /* == 1 */);
    }

    // WrapCylinder has:
    //   display_preference = 0
    //   color = default
    //   VisibleObject display_preference = 4 
    {
        const auto& cyl = wrapObjSet.get("wrapcylinder");
        // The outer display_preference overrides the inner one.
        SimTK_TEST(!cyl.get_Appearance().get_visible());
        SimTK_TEST_EQ(cyl.get_Appearance().get_color(), SimTK::Cyan);
        SimTK_TEST(cyl.get_Appearance().get_representation() == 
                VisualRepresentation::DrawSurface /* == 3 */);
    }

    // WrapEllipsoid has:
    //   display_preference = 2
    //   color = 1 0.5 0
    //   VisibleObject display_preference = 3
    {
        const auto& ellipsoid = wrapObjSet.get("wrapellipsoid");
        SimTK_TEST(ellipsoid.get_Appearance().get_visible());
        SimTK_TEST_EQ(ellipsoid.get_Appearance().get_color(), Vec3(1, 0.5, 0));
        // Outer display_preference overrides inner one.
        SimTK_TEST(ellipsoid.get_Appearance().get_representation() == 
                VisualRepresentation::DrawWireframe /* == 2 */);
    }
}

TEST_CASE("testWrapObjectScaleWithNoFrameDoesNotSegfault") {
    // reproduction for #3465
    //
    // effectively, if code tries to use a `WrapObject` outside
    // of a `PhysicalFrame` then the code in this test will segfault
    // without the fix because `WrapObject` will contain nullptrs
    // that are usually "fixed" by the parent `PhysicalFrame`
    //
    // the "proper" fix for this is to (e.g.) use the socket API but
    // this was avoided in #3465, which just focuses on downgrading
    // the segfault

    try {
        OpenSim::Model m;
        m.addComponent(new OpenSim::WrapCylinder{});
        m.buildSystem();
        SimTK::State& state = m.initializeState();
        m.scale(state, OpenSim::ScaleSet{}, true);
    } catch (const OpenSim::Exception&) {
        // the fix in #3465 only ensures no runtime segfaults may
        // occur - it does not guarantee that `WrapObject`s are
        // usable outside of their usual usage (i.e. as children
        // of `PhysicalFrame`s)
    }
}

TEST_CASE("testMuscleLengths") {
    
    SECTION("Rajagopal2016, 18 muscles") {
        Model model("subject_walk_armless_18musc.osim");
        model.initSystem();
    
        TableProcessor tableProcessor =
                TableProcessor("subject_walk_armless_coordinates.mot") |
                TabOpUseAbsoluteStateNames();
        TimeSeriesTable coordinates =
                tableProcessor.processAndConvertToRadians(model);
        
        TimeSeriesTable lengths = createMuscleLengthsTable(model, coordinates);

        TimeSeriesTable stdLengths(
                "std_testMuscleLengths_subject_walk_armless.sto");
        CHECK(SimTK::Test::numericallyEqual(
                lengths.getMatrix(), stdLengths.getMatrix(),
                static_cast<int>(lengths.getNumColumns()), 1e-6));
    }
    
    SECTION("gait10dof18musc") {
        Model model("walk_gait1018_subject01.osim");
        model.initSystem();

        TableProcessor tableProcessor =
                TableProcessor("walk_gait1018_state_reference.mot") |
                TabOpUseAbsoluteStateNames();
        TimeSeriesTable coordinates =
                tableProcessor.processAndConvertToRadians(model);

        TimeSeriesTable lengths = createMuscleLengthsTable(model, coordinates);

        TimeSeriesTable stdLengths("std_testMuscleLengths_walk_gait1018.sto");
        CHECK(SimTK::Test::numericallyEqual(lengths.getMatrix(),
                stdLengths.getMatrix(),
                static_cast<int>(lengths.getNumColumns()), 1e-6));
    }
}
