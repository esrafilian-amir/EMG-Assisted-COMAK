/* -------------------------------------------------------------------------- *
 *                 OpenSim:  testSmith2018NonlinearContact.cpp                *
 * -------------------------------------------------------------------------- *
 * Tests of the nonlinear elastic foundation of Smith2018ArticularContactForce:
 * the pressure against an independent solution of the depth-pressure
 * equation, the energy against the pressure (dE/dd = A P), and finite,
 * smooth values for overlaps beyond the layer thickness.
 * -------------------------------------------------------------------------- */

#include <cmath>
#include <OpenSim/Simulation/osimSimulation.h>
#include <catch2/catch_all.hpp>

using namespace OpenSim;

namespace {

// stiffness (1-v)E/((1+v)(1-2v))
double stiffness(double E, double v) {
    return (1 - v) * E / ((1 + v) * (1 - 2 * v));
}

// Independent solution of d = h1(1-exp(-P/k1)) + h2(1-exp(-P/k2)) by bisection
// for d below the strain cap, and the linear continuation above it.
// Single layer if h2 = 0.
double referencePressure(double d, double h1, double k1, double h2, double k2,
        double max_strain) {
    auto depth = [&](double P) {
        return h1 * (1 - std::exp(-P / k1)) + h2 * (1 - std::exp(-P / k2));
    };
    double P_cap = -k1 * std::log(1 - max_strain);
    if (h2 > 0) { P_cap = std::min(P_cap, -k2 * std::log(1 - max_strain)); }
    const double d_cap = depth(P_cap);
    if (d > d_cap) {
        const double slope = h1 / k1 * std::exp(-P_cap / k1) +
            h2 / k2 * std::exp(-P_cap / k2);
        return P_cap + (d - d_cap) / slope;
    }
    double lo = 0, hi = P_cap;
    for (int i = 0; i < 200; ++i) {
        const double P = 0.5 * (lo + hi);
        if (depth(P) < d) { lo = P; } else { hi = P; }
    }
    return 0.5 * (lo + hi);
}

struct DiskOnPlane {
    Model model;
    Smith2018ArticularContactForce* contact;
    Coordinate* ty;
    SimTK::State state;
    double h_plane, h_disk, E, v;

    DiskOnPlane(bool lumped, const std::string& formulation) {
        h_plane = 0.003;
        h_disk = 0.0025;
        E = 5e6;
        v = 0.45;

        model.setGravity(SimTK::Vec3(0, 0, 0));
        auto* plane = new OpenSim::Body("plane", 0.0, SimTK::Vec3(0),
            SimTK::Inertia(0));
        auto* disk = new OpenSim::Body("disk", 1.0, SimTK::Vec3(0),
            SimTK::Inertia::brick(0.05, 0.05, 0.05));
        model.addBody(plane);
        model.addBody(disk);

        auto* ground_plane =
            new WeldJoint("ground_plane", model.getGround(), *plane);
        // slider along y: the joint x axis is rotated by 90 degrees about z
        const SimTK::Vec3 rotation(0, 0, SimTK::Pi / 2);
        auto* slider = new SliderJoint("plane_disk", *plane, SimTK::Vec3(0),
            rotation, *disk, SimTK::Vec3(0), rotation);
        slider->updCoordinate().setName("ty");
        model.addJoint(ground_plane);
        model.addJoint(slider);

        auto* plane_mesh = new Smith2018ContactMesh(
            "plane", "x_z_plane.stl", *plane);
        auto* disk_mesh = new Smith2018ContactMesh(
            "disk", "x_z_disk_10cm_radius.stl", *disk);
        plane_mesh->set_elastic_modulus(E);
        plane_mesh->set_poissons_ratio(v);
        plane_mesh->set_thickness(h_plane);
        disk_mesh->set_elastic_modulus(E);
        disk_mesh->set_poissons_ratio(v);
        disk_mesh->set_thickness(h_disk);
        model.addContactGeometry(plane_mesh);
        model.addContactGeometry(disk_mesh);

        contact = new Smith2018ArticularContactForce(
            "contact", *plane_mesh, *disk_mesh);
        contact->set_max_proximity(0.1);
        contact->set_elastic_foundation_formulation(formulation);
        contact->set_use_lumped_contact_model(lumped);
        model.addForce(contact);

        state = model.initSystem();
        ty = &model.updCoordinateSet().get("ty");
    }

    // sets the overlap (m) and returns max pressure, energy and area
    void evaluate(double depth, double& pressure, double& energy,
            double& area) {
        ty->setValue(state, -depth);
        model.realizeReport(state);
        pressure = contact->getOutputValue<double>(
            state, "casting_total_max_pressure");
        energy = contact->getOutputValue<double>(state, "potential_energy");
        area = contact->getOutputValue<double>(
            state, "casting_total_contact_area");
    }
};

} // namespace

TEST_CASE("testSmith2018NonlinearContact") {
    const double max_strain = 0.8;
    // overlaps below and above the strain cap (about 4.4 mm for these layers)
    const std::vector<double> depths = {0.0005, 0.001, 0.002, 0.003, 0.004,
        0.005, 0.006, 0.008, 0.01};

    for (bool lumped : {true, false}) {
        DiskOnPlane test(lumped, "nonlinear");
        const double k = stiffness(test.E, test.v);

        for (double depth : depths) {
            double P, energy, area;
            test.evaluate(depth, P, energy, area);

            // pressure against the independent solution
            const double expected = lumped ?
                referencePressure(depth, test.h_plane + test.h_disk, k, 0, 1,
                    max_strain) :
                referencePressure(depth, test.h_plane, k, test.h_disk, k,
                    max_strain);
            REQUIRE(std::isfinite(P));
            REQUIRE(std::isfinite(energy));
            CHECK(P == Catch::Approx(expected).epsilon(1e-6));

            // the energy is the integral of the pressure: dE/dd = A P
            double Pp, Ep, Ap, Pm, Em, Am;
            const double delta = 1e-7;
            test.evaluate(depth + delta, Pp, Ep, Ap);
            test.evaluate(depth - delta, Pm, Em, Am);
            const double dEdd = (Ep - Em) / (2 * delta);
            CHECK(dEdd == Catch::Approx(area * P).epsilon(1e-4));
        }

        // pressure and its slope are continuous at the strain cap (4.4 mm)
        double P1, E, A;
        double previous_P = 0.0, previous_slope = 0.0;
        const double step = 1e-5;
        for (double d = 0.0042; d <= 0.0047; d += step) {
            test.evaluate(d, P1, E, A);
            if (previous_P > 0) {
                REQUIRE(P1 > previous_P);
                const double slope = (P1 - previous_P) / step;
                if (previous_slope > 0) {
                    const double ratio = slope / previous_slope;
                    CHECK(ratio > 0.99);
                    CHECK(ratio < 1.02);
                }
                previous_slope = slope;
            }
            previous_P = P1;
        }
    }

    SECTION("variable model with identical layers equals the lumped model") {
        DiskOnPlane lumped(true, "nonlinear");
        DiskOnPlane variable(false, "nonlinear");
        // layers with the same material in series are the same as one layer
        // with the sum of the thicknesses
        double Pl, Pv, El, Ev, A;
        for (double depth : {0.001, 0.003, 0.006}) {
            lumped.evaluate(depth, Pl, El, A);
            variable.evaluate(depth, Pv, Ev, A);
            CHECK(Pl == Catch::Approx(Pv).epsilon(1e-6));
            CHECK(El == Catch::Approx(Ev).epsilon(1e-6));
        }
    }

    SECTION("linear formulation is unchanged and finite for deep overlaps") {
        DiskOnPlane linear(false, "linear");
        const double k = stiffness(linear.E, linear.v);
        double P, E, A;
        for (double depth : {0.001, 0.005, 0.01}) {
            linear.evaluate(depth, P, E, A);
            const double expected = depth /
                (linear.h_plane / k + linear.h_disk / k);
            CHECK(P == Catch::Approx(expected).epsilon(1e-6));
            CHECK(E == Catch::Approx(0.5 * A * expected * depth).epsilon(1e-6));
        }
    }

    SECTION("invalid properties are rejected") {
        DiskOnPlane test(true, "nonlinear");
        test.contact->set_elastic_foundation_formulation("nonlinaer");
        CHECK_THROWS(test.model.finalizeFromProperties());
        test.contact->set_elastic_foundation_formulation("nonlinear");
        test.contact->set_max_strain(1.0);
        CHECK_THROWS(test.model.finalizeFromProperties());
    }
}
