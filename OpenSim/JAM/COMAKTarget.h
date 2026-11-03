#ifndef OPENSIM_COMAK_TARGET_H_
#define OPENSIM_COMAK_TARGET_H_
/* -------------------------------------------------------------------------- *
 *                              COMAKTarget.h                                 *
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
//#include "osimAnalysesDLL.h"
#include "OpenSim/Common/Array.h"
#include "COMAKTool.h"
#include <simmath/Optimizer.h>

//=============================================================================
//=============================================================================
namespace OpenSim { 

/**

 */
class ComakTarget : public SimTK::OptimizerSystem
{


//=============================================================================
// METHODS
//=============================================================================
public:
    ComakTarget(SimTK::State s, Model* aModel,
        const SimTK::Vector& observed_udot,
        const SimTK::Vector& init_parameters,
        double dt, double unit_udot_epsilon, 
        Array<std::string> primary_coords, Array<std::string> secondary_coords,
        Array<std::string> muscle_path,
        Array<std::string> non_muscle_actuator_path,
        const SimTK::Vector& max_change);

    //-------------------------------------------------------------------------
    // REQUIRED OPTIMIZATION TARGET METHODS
    //-------------------------------------------------------------------------
    
    /**
     * Compute objective function value (peformance) from the optimization 
     * parameters
     *
     * @param parameters Vector of optimization parameters.
     * @param performance Value of the performance criterion.
     * @return Status (normal termination = 0, error < 0).
     */
    int objectiveFunc(const SimTK::Vector &parameters, bool new_parameters, 
        SimTK::Real& performance) const override;

    
    /**
     * Compute the gradient of performance given parameters.
     *
     * @param parameters Vector of optimization parameters.
     * @param gradient Derivatives of performance with respect to the parameters.
     * @return Status (normal termination = 0, error < 0).
     */
    int gradientFunc(const SimTK::Vector &parameters, bool new_parameters,
        SimTK::Vector &gradient) const override;

    /**
     * Compute the optimization constraints given parameters.
     *
     * @param parameters Vector of optimization parameters.
     * @param constraints Vector of optimization constraints.
     * @return Status (normal termination = 0, error < 0).
     */
    int constraintFunc(const SimTK::Vector &parameters, bool new_parameters, 
        SimTK::Vector &constraints) const override;

    /**
     * Compute the gradient of constraints given parameters.
     *
     * @param parameters Vector of parameters.
     * @param jac Derivative of constraint with respect to the parameters.
     * @return Status (normal termination = 0, error < 0).
     */
    int constraintJacobian(const SimTK::Vector &parameters, 
        bool new_parameters, SimTK::Matrix &jac) const override;

    //Helper
    void initialize();
    void realizeAccelerationFromParameters(
        SimTK::State& s, const SimTK::Vector &parameters);
    void precomputeConstraintMatrix();
    void setParameterBounds(double scale);
    void printPerformance(SimTK::Vector parameters);

    //Set
   void setModel(Model& aModel){
        _model = &aModel;
    }

    void setCostFunctionWeight(SimTK::Vector weight) {
        _muscle_weight = weight;
    }

    void setDesiredActivation(SimTK::Vector desired_act) {
        _desired_act = desired_act;
    }

    void setMaxChange(SimTK::Vector max_change) {
        _max_change = max_change;
    }

    void setUnitUdotEpsilon(double unit_udot_epsilon) {
        _unit_udot_epsilon = unit_udot_epsilon;
    };

    void setDT(double dt) {
        _dt = dt;
    };

    void setOptimalForces(SimTK::Vector optimal_force) {
         _optimal_force = optimal_force;
    }

    void setMuscleVolumes(SimTK::Vector msl_volumes) {
        _muscle_volumes = msl_volumes;
    }

    void setContactEnergyWeight(double contact_energy_weight) {
        _contact_energy_weight = contact_energy_weight;
    }

    void setNonMuscleActuatorWeight(double non_muscle_actuator_weight) {
        _non_muscle_actuator_weight = non_muscle_actuator_weight;
    }

    void setActivationExponent(double activation_exponent) {
        _activation_exponent = activation_exponent;
    }

    void setScaleDeltaCoordinates(double scale) {
        _scale_delta_coord = scale;
    }

    void setParameterNames(const Array<std::string>& parameter_names) {
        _parameter_names = parameter_names;
    }
    /** Options for the constraint Jacobian: central instead of forward
    differences for the secondary coordinate columns, no speed term for the
    secondary coordinates, and normalization of each constraint row by its
    largest entry. Call before initialize(). */
    void setJacobianOptions(bool central_difference, bool ignore_speed_term,
            bool normalize_rows) {
        _central_difference = central_difference;
        _ignore_secondary_speed_term = ignore_speed_term;
        _normalize_rows = normalize_rows;
    }

    /** Values of the secondary coordinates at the start of the time step: the
    total change of a secondary coordinate stays within its maximum change
    from this value. Call before initialize(). */
    void setReferenceValues(const SimTK::Vector& reference_values) {
        _reference_values = reference_values;
    }

    /** Penalty on the change of the muscle activations from the previous time
    step: weight * sum(muscle cost coefficient * (activation - previous)^2),
    with the muscle cost coefficient of the activation cost (weight 1 is as
    costly as the activation itself). Call before initialize(). */
    void setActivationChangePenalty(double weight,
            const SimTK::Vector& previous_activations) {
        _act_change_weight = weight;
        _previous_activations = previous_activations;
    }

    /** Relax the acceleration constraints with a penalized slack variable per
    constraint row, so the optimization always has a solution: row + slack =
    0 (in the units of the row), and the squared slack times the penalty is
    added to the cost. The optimization then
    has getNumVariables() = number of parameters + number of constraints
    variables, with the slack variables last. Call before initialize(). */
    void setRelaxConstraints(bool relax, double slack_penalty) {
        _relax = relax;
        _slack_penalty = slack_penalty;
    }

    /** The number of optimization variables: the parameters, and the slack
    variables if the constraints are relaxed. */
    int getNumVariables() const { return _nParameters + _nSlack; }

    /** Keep the secondary coordinates within their range [range min, range
    max] in the optimization. Call before initialize(). */
    void setEnforceCoordinateRange(bool enforce) {
        _enforce_coordinate_range = enforce;
    }

    /** Penalty weight * sum((unit_scale * change of secondary coordinate)^2) on
    the change of the secondary coordinates in the optimization. */
    void setSecondaryStepPenalty(double weight, const SimTK::Vector& unit_scale) {
        _step_penalty_weight = weight;
        _step_unit_scale = unit_scale;
    }

    /** Reuse the columns of the constraint Jacobian from an earlier
    linearization instead of evaluating the model for them (only the
    accelerations at the current state are evaluated). Call before
    initialize(). */
    void setReusedJacobian(const SimTK::Matrix& msl,
            const SimTK::Matrix& non_muscle, const SimTK::Matrix& coord,
            const SimTK::Matrix& speed, const SimTK::Vector& energy) {
        _reuse_jacobian = true;
        _reused_msl_unit_udot = msl;
        _reused_non_muscle_actuator_unit_udot = non_muscle;
        _reused_secondary_coord_unit_udot = coord;
        _reused_secondary_speed_unit_udot = speed;
        _reused_secondary_coord_unit_energy = energy;
    }

    /** The columns computed by initialize(), to reuse later. */
    void getJacobianColumns(SimTK::Matrix& msl, SimTK::Matrix& non_muscle,
            SimTK::Matrix& coord, SimTK::Matrix& speed,
            SimTK::Vector& energy) const {
        msl = _msl_unit_udot;
        non_muscle = _non_muscle_actuator_unit_udot;
        coord = _secondary_coord_unit_udot;
        speed = _secondary_speed_unit_udot;
        energy = _secondary_coord_unit_energy;
    }

    //Amir
    void setVerbose(int verbose) {
        _verbose = verbose;
    }
    
    void setEmgGammaWeight(SimTK::Vector weight) {
        _emg_gamma_weight = weight;
    }
    //=========================================================================
    // DATA
    //=========================================================================
public:
	bool _is_emg_assisted;

private:
    Model *_model;
    SimTK::State _state;
    SimTK::Vector _optimal_force;
    const SimTK::Vector _init_parameters;
    double _activation_exponent;
    SimTK::Vector _observed_udot;

    int _nSecondaryCoord;
    int _nPrimaryCoord;
    int _nCoordinates;
    int _nActuators;

    int _nConstraints;
    Array<std::string> _constraint_names;

    int _nParameters;
    bool _central_difference = false;
    bool _ignore_secondary_speed_term = false;
    bool _normalize_rows = false;
    bool _enforce_coordinate_range = false;
    bool _relax = false;
    double _slack_penalty = 1e4;
    int _nSlack = 0;
    SimTK::Vector _row_scale;
    double _act_change_weight = 0.0;
    SimTK::Vector _previous_activations;
    SimTK::Vector _reference_values;
    double _step_penalty_weight = 0.0;
    SimTK::Vector _step_unit_scale;
    Array<std::string> _parameter_names;

    int _nMuscles;
    Array<std::string> _muscle_path;
    SimTK::Vector _muscle_volumes;
    SimTK::Vector _muscle_weight;
    SimTK::Vector _desired_act;

    int _nNonMuscleActuators;
    Array<std::string> _non_muscle_actuator_path;

    SimTK::Matrix _msl_unit_udot;
    SimTK::Matrix _non_muscle_actuator_unit_udot;
    SimTK::Matrix _secondary_coord_unit_udot;
    SimTK::Matrix _secondary_speed_unit_udot;
    SimTK::Vector _secondary_coord_unit_energy;

    bool _reuse_jacobian = false;
    SimTK::Matrix _reused_msl_unit_udot;
    SimTK::Matrix _reused_non_muscle_actuator_unit_udot;
    SimTK::Matrix _reused_secondary_coord_unit_udot;
    SimTK::Matrix _reused_secondary_speed_unit_udot;
    SimTK::Vector _reused_secondary_coord_unit_energy;

    Array<std::string> _primary_coords;
    Array<std::string> _secondary_coords;
    
    int _verbose; //Amir
    SimTK::Vector _emg_gamma_weight; //Amir

    SimTK::Vector _max_change;
    double _unit_udot_epsilon;
    double _dt;
    double _initial_contact_energy;
    double _contact_energy_weight;
    double _non_muscle_actuator_weight;
    
    SimTK::Vector _initial_udot;
    SimTK::Vector _constraint_initial_udot;
    SimTK::Vector _constraint_desired_udot;

    double _scale_delta_coord;

    SimTK::Matrix _constraint_matrix;
protected:
};

}; //namespace

#endif // OPENSIM_COMAK_TARGET_H_
