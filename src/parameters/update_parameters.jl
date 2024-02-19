function PSI.update_parameter_values!(
    model::PSI.OperationModel,
    key::PSI.ParameterKey{T, U},
    input::PSI.DatasetContainer{PSI.InMemoryDataset},
) where {T <: PSI.ObjectiveFunctionParameter, U <: PSY.ReserveDemandCurve}
    # Enable again for detailed debugging
    # TimerOutputs.@timeit RUN_SIMULATION_TIMER "$T $U Parameter Update" begin
    optimization_container = PSI.get_optimization_container(model)
    # Note: Do not instantite a new key here because it might not match the param keys in the container
    # if the keys have strings in the meta fields
    param_array = PSI.get_parameter_array(optimization_container, key)
    parameter_attributes = PSI.get_parameter_attributes(optimization_container, key)

    service = PSY.get_component(U, PSI.get_system(model), key.meta)
    
    PSI._update_parameter_values!(param_array, parameter_attributes, service, model, input)
    IS.@record :execution PSI.ParameterUpdateEvent(
        T,
        U,
        parameter_attributes,
        PSI.get_current_timestamp(model),
        PSI.get_name(model),
    )
    #end
    return
end


function PSI._update_parameter_values!(
    param_array,
    attributes::PSI.CostFunctionAttributes,
    service::Type{V},
    model::PSI.DecisionModel,
    ::PSI.DatasetContainer{PSI.InMemoryDataset},
) where {V <: PSY.ReserveDemandCurve}
    @show "PSI._update_parameter_values!"
    initial_forecast_time = PSI.get_current_time(model) # Function not well defined for DecisionModels
    time_steps = PSI.get_time_steps(PSI.get_optimization_container(model))
    horizon = time_steps[end]
    container = PSI.get_optimization_container(model)
    if PSI.is_synchronized(container)
        obj_func = PSI.get_objective_function(container)
        PSI.set_synchronized_status(obj_func, false)
        PSI.reset_variant_terms(obj_func)
    end

    if PSI._has_variable_cost_parameter(service)
        name = PSY.get_name(service)
        ts_vector = PSY.get_variable_cost(
            service,
            start_time=initial_forecast_time,
            len=horizon,
        )
        variable_cost_forecast_values = TimeSeries.values(ts_vector)
        for (t, value) in enumerate(variable_cost_forecast_values)
            if attributes.uses_compact_power
                value, _ = PSI._convert_variable_cost(value)
            end
            PSI._set_param_value!(param_array, PSY.get_cost(value), name, t)
            PSI.update_variable_cost!(container, param_array, attributes, service, t)
        end
    end

    # update pwl parameters
    jump_model = PSI.get_jump_model(container)
    
    for pwl_constraint in PSI.get_constraints(container)[PSI.ConstraintKey{PSI.PieceWiseLinearCostConstraint, PSY.ReserveDemandCurve{PSY.ReserveUp}}(PSY.get_name(service))]
        JuMP.delete(jump_model, pwl_constraint)
    end

    name = PSY.get_name(service)
    variables = PSI.get_variable(container, PSI.ServiceRequirementVariable(), typeof(service), name)
    pwl_vars = PSI.get_variable(container, PSI.PieceWiseLinearCostVariable(), typeof(service))
    const_container = PSI.get_constraint(
        container,
        PSI.PieceWiseLinearCostConstraint(),
        typeof(service),
        name
    )
    base_power = PSI.get_base_power(container)

    ts_vector = PSY.get_variable_cost(
        service,
        start_time=initial_forecast_time,
        len=horizon,
    )
    variable_cost_forecast_values = TimeSeries.values(ts_vector)
    for (t, value) in enumerate(variable_cost_forecast_values)
        if attributes.uses_compact_power
            value, _ = PSI._convert_variable_cost(value)
        end
        len_cost_data = length(value)
        break_point = Float64[]
        for i in 1:len_cost_data
            push!(break_point, value[i][2] / base_power)
        end
        const_container[name, t] = JuMP.@constraint(
            jump_model,
            variables[name, t] ==
            sum(pwl_vars[name, ix, t] * break_point[ix] for ix in 1:len_cost_data)
        )
    end

    return
end


function PSI.update_variable_cost!(
    container::PSI.OptimizationContainer,
    param_array::JuMP.Containers.DenseAxisArray{Vector{NTuple{2, Float64}}},
    ::PSI.CostFunctionAttributes{Vector{NTuple{2, Float64}}},
    component::T,
    time_period::Int,
) where {T <: PSY.ReserveDemandCurve}
    component_name = PSY.get_name(component)
    cost_data = param_array[component_name, time_period]
    if all(iszero.(last.(cost_data)))
        return
    end
    gen_cost =
        PSI._update_pwl_cost_expression(container, T, component_name, time_period, cost_data)
    # Attribute doesn't have multiplier
    # gen_cost = attributes.multiplier * gen_cost_
    PSI.add_to_objective_variant_expression!(container, gen_cost)
    # PSI.set_expression!(container, PSI.ProductionCostExpression, gen_cost, component, time_period)
    return
end

function PSI._update_pwl_cost_expression(
    container::PSI.OptimizationContainer,
    ::Type{T},
    component_name::String,
    time_period::Int,
    cost_data::Vector{NTuple{2, Float64}},
) where {T <: PSY.ReserveDemandCurve}
    base_power = PSI.get_base_power(container)
    pwl_var_container = PSI.get_variable(container, PSI.PieceWiseLinearCostVariable(), T)
    resolution = PSI.get_resolution(container)
    dt = Dates.value(Dates.Second(resolution)) / PSI.SECONDS_IN_HOUR
    gen_cost = JuMP.AffExpr(0.0)
    slopes = PSY.get_slopes(cost_data)
    upb = PSY.get_breakpoint_upperbounds(cost_data)
    for i in 1:length(cost_data)
        linear_exp = - cost_data[i][1] * pwl_var_container[(component_name, i, time_period)]
        JuMP.add_to_expression!(gen_cost, linear_exp)
        # JuMP.add_to_expression!(
        #     gen_cost,
        #     slopes[i] * upb[i] * dt * pwl_var_container[(component_name, i, time_period)],
        # )
    end
    return gen_cost
end


# function PSI.update_variable_cost!(
#     container::PSI.OptimizationContainer,
#     param_array::PSI.JuMPFloatArray,
#     attributes::PSI.CostFunctionAttributes{Float64},
#     component::T,
#     time_period::Int,
# ) where {T <: PSY.ReserveDemandCurve}
#     @show "PSI.update_variable_cost!"
#     resolution = PSI.get_resolution(container)
#     dt = Dates.value(Dates.Second(resolution)) / PSI.SECONDS_IN_HOUR
#     base_power = PSI.get_base_power(container)
#     component_name = PSY.get_name(component)
#     cost_data = param_array[component_name, time_period]
#     if iszero(cost_data)
#         return
#     end
#     variable = PSI.get_variable(container, get_variable_type(attributes)(), T)
#     gen_cost = variable[component_name, time_period] * cost_data * base_power * dt
#     @show gen_cost
#     # Attribute doesn't have multiplier
#     # gen_cost = attributes.multiplier * gen_cost_
#     PSI.add_to_objective_variant_expression!(container, gen_cost)
#     return
# end