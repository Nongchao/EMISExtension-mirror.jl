function PSI.construct_service!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.ServiceModel{SR, QuadraticCostRampReserve},
    devices_template::Dict{Symbol, PSI.DeviceModel},
    incompatible_device_types::Set{<:DataType},
) where {SR <: PSY.Reserve}
    name = PSI.get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    contributing_devices = PSI.get_contributing_devices(model)
    PSI.add_variable!(container, PSI.ServiceRequirementVariable(), [service], QuadraticCostRampReserve())
    PSI.add_variables!(
        container,
        PSI.ActivePowerReserveVariable,
        service,
        contributing_devices,
        QuadraticCostRampReserve(),
    )
    PSI.add_to_expression!(container, PSI.ActivePowerReserveVariable, model, devices_template)
    #TODO: check if this works/used in cost function
    # PSI.add_expressions!(container, PSI.ProductionCostExpression, [service], model)
    PSI.add_feedforward_arguments!(container, model, service)
    return
end


function PSI.construct_service!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.ServiceModel{SR, QuadraticCostRampReserve},
    devices_template::Dict{Symbol, PSI.DeviceModel},
    incompatible_device_types::Set{<:DataType},
) where {SR <: PSY.Reserve}
    name = PSI.get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    contributing_devices = PSI.get_contributing_devices(model)
    # Custom implemention
    PSI.add_constraints!(container, PSI.RequirementConstraint, service, contributing_devices, model)
    # Provided by PSI 
    PSI.add_constraints!(container, PSI.RampConstraint, service, contributing_devices, model)
    PSI.objective_function!(container, service, model)

    PSI.add_feedforward_constraints!(container, model, service)

    PSI.add_constraint_dual!(container, sys, model)

    return
end


function PSI.construct_service!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.ServiceModel{SR, InertiaReserve},
    devices_template::Dict{Symbol, PSI.DeviceModel},
    incompatible_device_types::Set{<:DataType},
) where {SR <: PSY.Reserve}
    name = PSI.get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    PSI.add_parameters!(container, PSI.RequirementTimeSeriesParameter, service, model)
    contributing_devices = PSI.get_contributing_devices(model)
    PSI.add_variables!(
        container,
        PSI.ActivePowerReserveVariable,
        service,
        contributing_devices,
        InertiaReserve(),
    )
    PSI.add_feedforward_arguments!(container, model, service)
    return
end


function PSI.construct_service!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.ServiceModel{SR, InertiaReserve},
    devices_template::Dict{Symbol, PSI.DeviceModel},
    incompatible_device_types::Set{<:DataType},
) where {SR <: PSY.Reserve}
    name = PSI.get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    contributing_devices = PSI.get_contributing_devices(model)
    # implemented in PSI
    PSI.add_constraints!(container, PSI.RequirementConstraint, service, contributing_devices, model)
    #TODO: need to implement this InertiaConstraint
    PSI.add_constraints!(container, InertiaConstraint, service, contributing_devices, model)
    PSI.objective_function!(container, service, model)

    PSI.add_feedforward_constraints!(container, model, service)

    PSI.add_constraint_dual!(container, sys, model)

    return
end


function PSI.construct_service!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ArgumentConstructStage,
    model::PSI.ServiceModel{SR, CleanEnergyReserve},
    devices_template::Dict{Symbol, PSI.DeviceModel},
    incompatible_device_types::Set{<:DataType},
) where {SR <: PSY.Reserve}
    name = PSI.get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    PSI.add_parameters!(container, PSI.RequirementTimeSeriesParameter, service, model)
    contributing_devices = PSI.get_contributing_devices(model)
    PSI.add_variables!(container, PSI.ServiceRequirementVariable, [service], CleanEnergyReserve())
    PSI.add_variables!(
        container,
        PSI.ActivePowerReserveVariable,
        service,
        contributing_devices,
        CleanEnergyReserve(),
    )
    # PSI.add_to_expression!(container, PSI.ActivePowerReserveVariable, model, devices_template)
    PSI.add_feedforward_arguments!(container, model, service)
    return
end


function PSI.construct_service!(
    container::PSI.OptimizationContainer,
    sys::PSY.System,
    ::PSI.ModelConstructStage,
    model::PSI.ServiceModel{SR, CleanEnergyReserve},
    devices_template::Dict{Symbol, PSI.DeviceModel},
    incompatible_device_types::Set{<:DataType},
) where {SR <: PSY.Reserve}
    name = PSI.get_service_name(model)
    service = PSY.get_component(SR, sys, name)
    contributing_devices = PSI.get_contributing_devices(model)
    # implemented in PSI
    PSI.add_constraints!(container, PSI.RequirementConstraint, service, contributing_devices, model)
    PSI.add_constraints!(container, CleanEnergyConstraint, service, contributing_devices, model)

    PSI.objective_function!(container, service, model)

    PSI.add_feedforward_constraints!(container, model, service)

    PSI.add_constraint_dual!(container, sys, model)

    return
end

#= old Constructor for CleanEnergyReserve

function PSI.construct_service!(
    optimization_container::PSI.PSI.OptimizationContainer,
    services::Vector{SR},
    sys::PSY.System,
    model::PSI.ServiceModel{SR, CleanEnergyReserve},
    devices_template::Dict{Symbol, PSI.DeviceModel},
    incompatible_device_types::Vector{<:DataType},
) where {SR <: PSY.Reserve}
    services_mapping = PSY.get_contributing_device_mapping(sys)
    time_steps = PSI.model_time_steps(optimization_container)
    names = [PSY.get_name(s) for s in services]

    if PSI.model_has_parameters(optimization_container)
        container = PSI.add_param_container!(
            optimization_container,
            PSI.UpdateRef{SR}("service_requirement", "requirement"),
            names,
            time_steps,
        )
    end

    PSI.add_cons_container!(
        optimization_container,
        PSI.make_constraint_name(PSI.REQUIREMENT, SR),
        names,
    )

    for service in services
        contributing_devices =
            services_mapping[(
                type = typeof(service),
                name = PSY.get_name(service),
            )].contributing_devices
        if !isempty(incompatible_device_types)
            contributing_devices =
                [d for d in contributing_devices if typeof(d) ∉ incompatible_device_types]
        end
        # Variables
        PSI.add_variables!(
            optimization_container,
            PSI.ActiveServiceVariable,
            service,
            contributing_devices,
            EnergyRequirementReserve(),
        )
        # Constraints
        PSI.service_requirement_constraint!(optimization_container, service, model)
        PSI.modify_device_model!(devices_template, model, contributing_devices)
        # Cost Function
        PSI.cost_function!(optimization_container, service, model)
    end
    return
end
=#
