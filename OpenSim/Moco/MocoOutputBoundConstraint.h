#ifndef OPENSIM_MOCOOUTPUTBOUNDCONSTRAINT_H
#define OPENSIM_MOCOOUTPUTBOUNDCONSTRAINT_H
//
// Created by Allison John on 7/24/24.
// Make an output bound constraint that can bound one or an arithmetic
// combination of two outputs between two functions
//

#include "MocoConstraint.h"

namespace OpenSim {

class MocoOutputBoundConstraint : MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoOutputBoundConstraint, MocoPathConstraint);

public:
    MocoOutputBoundConstraint() { constructProperties(); }

    /** Set the absolute path to the output in the model to be used in this path
    constraint. The format is "/path/to/component|output_name". */
    void setOutputPath(std::string path) { set_output_path(std::move(path)); }
    const std::string& getOutputPath() const { return get_output_path(); }

    /** Set the absolute path to the optional second Output in the model to be
    used in this path constraint. The format is "/path/to/component|output_name".
    This Output should have the same  type as the first Output. If providing a
    second Output, the user must also provide an operation via `setOperation()`.
    */
    void setSecondOutputPath(std::string path) {
        set_second_output_path(std::move(path));
    }
    const std::string& getSecondOutputPath() const {
        return get_second_output_path();
    }

    /** Set the exponent applied to the output value in the constraint. This
    exponent is applied when minimizing the norm of a vector type output. */
    void setExponent(int exponent) { set_exponent(exponent); }
    int getExponent() const { return get_exponent(); }

    /** Set the index to the value to be constrained when a vector type Output
    is specified. For SpatialVec Outputs, indices 0, 1, and 2 refer to the
    rotational components and indices 3, 4, and 5 refer to the translational
    components. A value of -1 indicates to constrain the vector norm (which is
    the default setting). If an index for a type double Output is provided, an
    exception is thrown. */
    void setOutputIndex(int index) { set_output_index(index); }
    int getOutputIndex() const { return get_output_index(); }


    void setLowerBound(const Function& f) { set_lower_bound(f); }
    void clearLowerBound() { updProperty_lower_bound().clear(); }
    bool hasLowerBound() const { return !getProperty_lower_bound().empty(); }
    const Function& getLowerBound() const { return get_lower_bound(); }

    void setUpperBound(const Function& f) { set_upper_bound(f); }
    void clearUpperBound() { updProperty_upper_bound().clear(); }
    bool hasUpperBound() const { return !getProperty_upper_bound().empty(); }
    const Function& getUpperBound() const { return get_upper_bound(); }

    /// Should the control be constrained to be equal to the lower bound (rather
    /// than an inequality constraint)? In this case, the upper bound must be
    /// unspecified.
    void setEqualityWithLower(bool v) { set_equality_with_lower(v); }
    //// @copydoc setEqualityWithLower()
    bool getEqualityWithLower() const { return get_equality_with_lower(); }

protected:
    void initializeOnModelImpl(
            const Model& model, const MocoProblemInfo&) const override;

    void calcPathConstraintErrorsImpl(
            const SimTK::State& state, SimTK::Vector& errors) const override;
    // these could be private
    /** Calculate the Output value for the provided SimTK::State. Do not
    call this function until 'initializeOnModelBase()' has been called. */
    double calcOutputValue(const SimTK::State&) const;

    /** Raise a value to the exponent set via 'setExponent()'. Do not call this
    function until 'initializeOnModelBase()' has been called. */
    double setValueToExponent(double value) const {
        return m_power_function(value);
    }

    /** Get the "depends-on stage", or the SimTK::Stage we need to realize the
    system to in order to calculate the Output value. */
    const SimTK::Stage& getDependsOnStage() const {
        return m_dependsOnStage;
    }

private:
    OpenSim_DECLARE_PROPERTY(output_path, std::string,
            "The absolute path to the Output in the model to be used in this "
            "path constraint (i.e., '/path/to/component|output_name').");
    OpenSim_DECLARE_PROPERTY(second_output_path, std::string,
            "The absolute path to the optional second Output in the model to be used"
            " in this path constraint (i.e., '/path/to/component|output_name').");
    OpenSim_DECLARE_PROPERTY(operation, std::string, "The operation to combine "
            "the two outputs: 'addition', 'subtraction', 'multiplication', or "
            "'divison'.");
    OpenSim_DECLARE_PROPERTY(exponent, int,
            "The exponent applied to the output value in the constraint "
            "(default: 1).");
    OpenSim_DECLARE_PROPERTY(output_index, int,
            "The index to the value to be constrained when a vector type "
            "Output is specified. For SpatialVec Outputs, indices 0, 1, "
            "and 2 refer to the rotational components and indices 3, 4, "
            "and 5 refer to the translational components. A value of -1 "
            "indicates to constrain the vector norm (default: -1).");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(
        lower_bound, Function, "Lower bound as a function of time.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(
            upper_bound, Function, "Upper bound as a function of time.");
    OpenSim_DECLARE_PROPERTY(equality_with_lower, bool,
            "The control must be equal to the lower bound; "
            "upper must be unspecified (default: false).");

    void constructProperties();

    /** Initialize additional information when there are two Outputs:
    the second Output, the operation, and the dependsOnStage. */
    void initializeComposite() const;

    /** Calculate the Output value of one Output. */
    double calcSingleOutputValue(const SimTK::State&) const;
    /** Calculate the two Output values and apply the operation. */
    double calcCompositeOutputValue(const SimTK::State&) const;

    /** Get a reference to the Output for this path constraint. */
    template <typename T>
    const Output<T>& getOutput() const {
        return static_cast<const Output<T>&>(m_output.getRef());
    }
    /** Get a reference to the second Output for this path constraint. */
    template <typename T>
    const Output<T>& getSecondOutput() const {
        return static_cast<const Output<T>&>(m_second_output.getRef());
    }

    /** Apply the operation to two double values. */
    double applyOperation(double value1, double value2) const {
        switch (m_operation) {
        case Addition       : return value1 + value2;
        case Subtraction    : return value1 - value2;
        case Multiplication : return value1 * value2;
        case Division       : return value1 / value2;
        default             : OPENSIM_THROW_FRMOBJ(Exception,
                                "Internal error: invalid operation type for "
                                "double type Outputs.");
        }
    }
    /** Apply the elementwise operation to two SimTK::Vec3 values. */
    double applyOperation(const SimTK::Vec3& value1,
                          const SimTK::Vec3& value2) const {
        switch (m_operation) {
        case Addition       : return (value1 + value2).norm();
        case Subtraction    : return (value1 - value2).norm();
        case Multiplication : return value1.elementwiseMultiply(value2).norm();
        case Division       : return value1.elementwiseDivide(value2).norm();
        default             : OPENSIM_THROW_FRMOBJ(Exception,
                                "Internal error: invalid operation type for "
                                "SimTK::Vec3 type Outputs.");
        }
    }
    /** Apply the elementwise operation to two SimTK::SpatialVec values.
    Multiplication and divison operations are not supported for SpatialVec
    Outputs without an index. */
    double applyOperation(const SimTK::SpatialVec& value1,
                          const SimTK::SpatialVec& value2) const {
        switch (m_operation) {
        case Addition       : return (value1 + value2).norm();
        case Subtraction    : return (value1 - value2).norm();
        default             : OPENSIM_THROW_FRMOBJ(Exception,
                                "Internal error: invalid operation type for "
                                "SimTK::SpatialVec type Outputs.");
        }
    }

    enum DataType {
        Type_double,
        Type_Vec3,
        Type_SpatialVec,
    };
    /** Get the string of the data type, for prints and error messages. */
    std::string getDataTypeString(DataType type) const {
        switch (type) {
        case Type_double     : return "double";
        case Type_Vec3       : return "SimTK::Vec3";
        case Type_SpatialVec : return "SimTK::SpatialVec";
        default              : return "empty Datatype";
        }
    }

    mutable DataType m_data_type;
    mutable SimTK::ReferencePtr<const AbstractOutput> m_output;
    mutable SimTK::ReferencePtr<const AbstractOutput> m_second_output;
    enum OperationType { Addition, Subtraction, Multiplication, Division };
    mutable OperationType m_operation;
    mutable bool m_useCompositeOutputValue;
    mutable std::function<double(const double&)> m_power_function;
    mutable int m_index1;
    mutable int m_index2;
    mutable bool m_boundVectorNorm;
    mutable SimTK::Stage m_dependsOnStage = SimTK::Stage::Acceleration;
    mutable bool m_hasLower;
    mutable bool m_hasUpper;

};

} // namespace OpenSim

#endif // OPENSIM_MOCOOUTPUTBOUNDCONSTRAINT_H

