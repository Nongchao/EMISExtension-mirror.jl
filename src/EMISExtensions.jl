module EMISExtensions

#################################################################################
# Exports
# OperationProblem
# Service Formulations
export QuadraticCostRampReserve
export InertiaReserve
export CleanEnergyReserve

#################################################################################
# Imports
import PowerSystems
import PowerSimulations
import JuMP
# import JuMP: DenseAxisArray
import DataFrames
import ParameterJuMP
import InfrastructureSystems
import Dates
import MathOptInterface
import TimeSeries

const IS = InfrastructureSystems
const MOI = MathOptInterface
const PSY = PowerSystems
const PSI = PowerSimulations
const PM = PowerSimulations.PM
const PJ = ParameterJuMP

#################################################################################
# Includes
include("core/constraints.jl")
include("core/formulations.jl")
include("core/optimization_container.jl")

include("service_models/services_constructor.jl")
include("service_models/reserves.jl")

include("parameters/update_parameters.jl")
end # module
