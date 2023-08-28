function get_H_constant(d::PSY.Component)
    if haskey(PSY.get_ext(d), "H_constant")
        return PSY.get_ext(d)["H_constant"]
    else
        @info(
            "Inertia data (h-constant) is missing in Generator $(PSY.get_name(d)), h-constant will be set to 0."
        )
        return 0.0
    end
end

### Constraints

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{PSI.RequirementConstraint},
    service::SR,
    ::U,
    ::PSI.ServiceModel{SR, QuadraticCostRampReserve},
) where {
    SR <: PSY.ReserveDemandCurve,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
} where {D <: PSY.Component}
    time_steps = PSI.get_time_steps(container)
    service_name = PSY.get_name(service)
    constraint = PSI.add_constraints_container!(
        container,
        T(),
        SR,
        [service_name],
        time_steps;
        meta=service_name,
    )
    reserve_variable =
        PSI.get_variable(container, PSI.ActivePowerReserveVariable(), SR, service_name)
    requirement_variable = PSI.get_variable(container, PSI.ServiceRequirementVariable(), SR, service_name)
    jump_model = PSI.get_jump_model(container)
    for t in time_steps
        constraint[service_name, t] = JuMP.@constraint(
            jump_model,
            sum(reserve_variable[:, t]) == requirement_variable[service_name, t]
            # double check with Sourabh: line 210 of reserves.jl in PSI
        )
    end

    return
end


function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{InertiaConstraint},
    service::S,
    device::U,
    model::PSI.ServiceModel{S, InertiaReserve},
) where {
    U <: PSY.ThermalGen,
    S <: PSY.Reserve
}
    ## This is complete example
    time_steps = PSI.get_time_steps(container)
    service_name = PSY.get_name(service)
    name = PSY.get_name(device)
    base_power = PSI.get_base_power(device)
    h_const = get_H_constant(device)
    reserve_variable =
        PSI.get_variable(container, PSI.ActivePowerReserveVariable(), S, service_name)
    con = PSI.get_constraint(container, T(), S, service_name)
    jump_model = PSI.get_jump_model(container)
    if PSI.has_container_key(container, PSI.OnStatusParameter, U)
        bin = PSI.get_parameter(container, PSI.OnStatusParameter(), U).parameter_array[name, :]
    elseif PSI.has_container_key(container, PSI.OnVariable, U)
        bin = PSI.get_variable(container, PSI.OnVariable(), U)[name, :]
    else
        #TODO: @Sourabh maybe error out here? 
        error("Model has no OnStatusParameter or OnVariable")
        # bin = ones(length(time_steps))
    end   
    for t in time_steps
        #TODO: what's this constraint correspond to?
        con[name, t] = JuMP.@constraint(jump_model, reserve_variable[name, t] == bin[t] * base_power * h_const)
    end
end

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{InertiaConstraint},
    service::S,
    device::U,
    model::PSI.ServiceModel{S, InertiaReserve},
) where {
    U <: PSY.HydroGen,
    S <: PSY.Reserve
}
    time_steps = PSI.get_time_steps(container)
    service_name = PSY.get_name(service)
    name = PSY.get_name(device)
    base_power = PSI.get_base_power(device)
    h_const = get_H_constant(device)
    reserve_variable =
        PSI.get_variable(container, PSI.ActivePowerReserveVariable(), S, service_name)
    con = PSI.get_constraint(container, T(), S, service_name)
    jump_model = PSI.get_jump_model(container)
    #TODO: Need to check with old code if this is correct
    p_var = PSI.get_variable(container, PSI.ActivePowerVariable(), U)[name, :]
    # Ask Sourabh, what is this?
    # expr = PSI.get_expression(container, PSI.ReserveRangeExpressionUB, U)[name, :] # P + R1_up + R2_up 
    for t in time_steps
        con[name, t] = JuMP.@constraint(jump_model, reserve_variable[name, t] == p_var[t] * base_power * h_const) # not sure i
    end
end

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{InertiaConstraint},
    service::S,
    device::U,
    model::PSI.ServiceModel{S, InertiaReserve},
) where {
    U <: PSY.Storage,
    S <: PSY.Reserve
}
    time_steps = PSI.get_time_steps(container)
    service_name = PSY.get_name(service)
    name = PSY.get_name(device)
    base_power = PSI.get_base_power(device)
    h_const = get_H_constant(device)
    reserve_variable =
        PSI.get_variable(container, PSI.ActivePowerReserveVariable(), S, service_name)
    con = PSI.get_constraint(container, T(), S, service_name)
    con_energy = PSI.get_constraint(container, T(), S, service_name*"_energy_constraint")
    jump_model = PSI.get_jump_model(container)
    # p_var = PSI.get_variable(container, PSI.ActivePowerOutVariable(), U)
    e_var = PSI.get_variable(container, PSI.EnergyVariable(), U)
    expr = PSI.get_expression(container, PSI.ReserveRangeExpressionUB, U)[name, :]
    #TODO: Adding the constraint
    for t in time_steps
        con[name, t] = JuMP.@constraint(jump_model, reserve_variable[name, t] == h_const * (base_power - expr[t])) # not sure i
        con_energy[name, t] = JuMP.@constraint(jump_model, reserve_variable[name, t] <= e_var[t] * PSI.SECONDS_IN_HOUR) # not sure i
    end
end


function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{InertiaConstraint},
    service::S,
    device::U,
    model::PSI.ServiceModel{S, InertiaReserve},
) where {
    U <: PSY.RenewableGen,
    S <: PSY.Reserve
}
    time_steps = PSI.get_time_steps(container)
    service_name = PSY.get_name(service)
    name = PSY.get_name(device)
    base_power = PSI.get_base_power(device)
    h_const = get_H_constant(device)
    reserve_variable =
        PSI.get_variable(container, PSI.ActivePowerReserveVariable(), S, service_name)
    con = PSI.get_constraint(container, T(), S, service_name)
    jump_model = PSI.get_jump_model(container)
    expr = PSI.get_expression(container, PSI.ReserveRangeExpressionUB, U)[name, :]

    parameter_container = PSI.get_parameter(container, PSI.ActivePowerTimeSeriesParameter, U)
    parameter = PSI.get_parameter_array(parameter_container)
    multiplier = PSI.get_multiplier_array(parameter_container)
    #TODO: Adding the constraint
    for t in time_steps
        con[name, t] = JuMP.@constraint(jump_model, reserve_variable[name, t] == h_const * (parameter[name, t] * multiplier[name, t] - expr[t])) # not sure i
    end
end


function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{InertiaConstraint},
    service::SR,
    contributing_devices::Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
    model::PSI.ServiceModel{SR, InertiaReserve},
) where {
    SR <: PSY.Reserve,
    U <: Union{Vector{D}, IS.FlattenIteratorWrapper{D}},
} where {D <: PSY.Component}
    time_steps = PSI.get_time_steps(container)
    service_name = PSY.get_name(service)
    PSI.add_constraints_container!(
        container,
        T(),
        SR,
        PSY.get_name.(contributing_devices),
        time_steps;
        meta=service_name,
    )
    storage_devices = []
    for d in contributing_devices
        if typeof(d) <: PSY.Storage
            push!(storage_devices, d)
        end
    end
    PSI.add_constraints_container!(
        container,
        T(),
        SR,
        PSY.get_name.(storage_devices),
        time_steps;
        meta=service_name*"_energy_constraint",
    )
    for d in contributing_devices
        PSI.add_constraints!(
            container,
            T,
            service,
            d,
            model,
        )
    end

    return
end

#### Objective Function

function _get_pwl_cost_expression_custom(
    container::PSI.OptimizationContainer,
    component::T,
    time_period::Int,
    cost_data::Vector{NTuple{2, Float64}},
    multiplier::Float64,
) where {T <: PSY.Component}
    name = PSY.get_name(component)
    pwl_var_container = PSI.get_variable(container, PSI.PieceWiseLinearCostVariable(), T)
    gen_cost = JuMP.QuadExpr(0.0)
    slopes = PSY.get_slopes(cost_data)
    upb = PSY.get_breakpoint_upperbounds(cost_data)
    for i in 1:length(cost_data)
        slope =
            abs(slopes[i]) != Inf ? slopes[i] : 0.0
        JuMP.add_to_expression!(
            gen_cost,
            ((1 / 2) * slope) * (multiplier * pwl_var_container[(name, i, time_period)]) .^ 2
        )
    end
    return gen_cost
end

####### added function below to address "ServiceRequirementVariable__ReserveDemandCurve__ReserveUp is not stored" bug ####### 
function _add_pwl_constraint!(
    container::PSI.OptimizationContainer,
    component::T,
    ::U,
    break_points::Vector{Float64},
    sos_status::PSI.SOSStatusVariable,
    period::Int,
) where {T <: PSY.Component, U <: PSI.VariableType}
    service_name = PSY.get_name(component)
    variables = PSI.get_variable(container, U(), T, service_name)
    const_container = PSI.lazy_container_addition!(
        container,
        PSI.PieceWiseLinearCostConstraint(),
        T,
        axes(variables)...,
    )
    len_cost_data = length(break_points)
    jump_model = PSI.get_jump_model(container)
    pwl_vars = PSI.get_variable(container, PSI.PieceWiseLinearCostVariable(), T)
    name = PSY.get_name(component)
    const_container[name, period] = JuMP.@constraint(
        jump_model,
        variables[name, period] ==
        sum(pwl_vars[name, ix, period] * break_points[ix] for ix in 1:len_cost_data)
    )

    if sos_status == PSI.SOSStatusVariable.NO_VARIABLE
        bin = 1.0
        @debug "Using Piecewise Linear cost function but no variable/parameter ref for ON status is passed. Default status will be set to online (1.0)" _group =
            PSI.LOG_GROUP_COST_FUNCTIONS

    elseif sos_status == PSI.SOSStatusVariable.PARAMETER
        bin = PSI.get_parameter(container, PSI.OnStatusParameter(), T).parameter_array[name, period]
        @debug "Using Piecewise Linear cost function with parameter OnStatusParameter, $T" _group =
            PSI.LOG_GROUP_COST_FUNCTIONS
    elseif sos_status == PSI.SOSStatusVariable.VARIABLE
        bin = get_variable(container, PSI.OnVariable(), T)[name, period]
        @debug "Using Piecewise Linear cost function with variable OnVariable $T" _group =
            PSI.LOG_GROUP_COST_FUNCTIONS
    else
        @assert false
    end

    JuMP.@constraint(
        jump_model,
        sum(pwl_vars[name, i, period] for i in 1:len_cost_data) == bin
    )
    return
end

function _add_pwl_term!(
    container::PSI.OptimizationContainer,
    component::T,
    cost_data::Matrix{PSY.VariableCost{Vector{Tuple{Float64, Float64}}}},
    ::U,
    ::V,
) where {T <: PSY.Component, U <: PSI.VariableType, V <: PSI.AbstractServiceFormulation}
    multiplier = PSI.objective_function_multiplier(U(), V())
    resolution = PSI.get_resolution(container)
    dt = Dates.value(Dates.Second(resolution)) / PSI.SECONDS_IN_HOUR
    base_power = PSI.get_base_power(container)
    # Re-scale breakpoints by Basepower
    name = PSY.get_name(component)
    time_steps = PSI.get_time_steps(container)
    pwl_cost_expressions = Vector{JuMP.AffExpr}(undef, time_steps[end])
    sos_val = PSI._get_sos_value(container, V, component)
    for t in time_steps
        data = PSY.get_cost(cost_data[t])
        slopes = PSY.get_slopes(data)
        # Shouldn't be passed for convexity check
        is_convex = false
        break_points = map(x -> last(x), data) ./ base_power
        PSI._add_pwl_variables!(container, T, name, t, data)
        _add_pwl_constraint!(container, component, U(), break_points, sos_val, t)
        if !is_convex
            PSI._add_pwl_sos_constraint!(container, component, U(), break_points, sos_val, t)
        end
        pwl_cost = _get_pwl_cost_expression_custom(container, component, t, data, multiplier * dt)

        pwl_cost_expressions[t] = pwl_cost
    end
    return pwl_cost_expressions
end

PSI.sos_status(::PSY.ReserveDemandCurve, ::QuadraticCostRampReserve)=PSI.SOSStatusVariable.NO_VARIABLE
PSI.uses_compact_power(::PSY.ReserveDemandCurve, ::QuadraticCostRampReserve)=false
PSI.objective_function_multiplier(::PSI.VariableType, ::QuadraticCostRampReserve)=PSI.OBJECTIVE_FUNCTION_NEGATIVE

####### added function below to address "ServiceRequirementVariable__ReserveDemandCurve__ReserveUp is not stored" bug ####### 
function _get_cost_function_parameter_container(
    container::PSI.OptimizationContainer,
    ::S,
    component::T,
    ::U,
    ::V,
    cost_type::DataType,
) where {
    S <: PSI.ObjectiveFunctionParameter,
    T <: PSY.Component,
    U <: PSI.VariableType,
    V <: QuadraticCostRampReserve,
}
    service_name = PSY.get_name(component)
    if PSI.has_container_key(container, S, T)
        return PSI.get_parameter(container, S(), T)
    else
        # this was the function that errored out.
        container_axes = axes(PSI.get_variable(container, U(), T, service_name))
        if PSI.has_container_key(container, PSI.OnStatusParameter, T)
            sos_val = PSI.SOSStatusVariable.PARAMETER
        else
            sos_val = PSI.sos_status(component, V())
        end
        return PSI.add_param_container!(
            container,
            S(),
            T,
            sos_val,
            U,
            PSI.uses_compact_power(component, V()),
            PSI._get_cost_function_data_type(cost_type),
            container_axes...,
        )
    end
end


function _add_variable_cost_to_objective!(
    container::PSI.OptimizationContainer,
    ::T,
    component::PSY.Reserve,
    ::U,
) where {T <: PSI.VariableType, U <: QuadraticCostRampReserve}
    component_name = PSY.get_name(component)
    @debug "PWL Variable Cost" _group = PSI.LOG_GROUP_COST_FUNCTIONS component_name
    # If array is full of tuples with zeros return 0.0
    time_steps = PSI.get_time_steps(container)
    initial_time = PSI.get_initial_time(container)
    variable_cost_forecast = PSI.get_time_series(container, component, "variable_cost")
    variable_cost_forecast_values = TimeSeries.values(variable_cost_forecast)
    variable_cost_forecast_values = map(PSY.VariableCost, variable_cost_forecast_values)
    parameter_container = _get_cost_function_parameter_container(
        container,
        PSI.CostFunctionParameter(),
        component,
        T(),
        U(),
        eltype(variable_cost_forecast_values),
    )
    pwl_cost_expressions =
        _add_pwl_term!(container, component, variable_cost_forecast_values, T(), U())
    jump_model = PSI.get_jump_model(container)
    for t in time_steps
        PSI.set_parameter!(
            parameter_container,
            jump_model,
            PSY.get_cost(variable_cost_forecast_values[t]),
            # Using 1.0 here since we want to reuse the existing code that adds the mulitpler
            #  of base power times the time delta.
            1.0,
            component_name,
            t,
        )
        PSI.add_to_objective_variant_expression!(container, pwl_cost_expressions[t])
    end
    return
end


function PSI.objective_function!(
    container::PSI.OptimizationContainer,
    service::PSY.ReserveDemandCurve{T},
    ::PSI.ServiceModel{PSY.ReserveDemandCurve{T}, SR},
) where {T <: PSY.ReserveDirection, SR <: QuadraticCostRampReserve}
    add_variable_cost!(container, PSI.ServiceRequirementVariable(), service, SR())
    return
end

function add_variable_cost!(
    container::PSI.OptimizationContainer,
    ::U,
    service::T,
    ::V,
) where {T <: PSY.ReserveDemandCurve, U <: PSI.VariableType, V <: QuadraticCostRampReserve}
    _add_variable_cost_to_objective!(container, U(), service, V())
    return
end
