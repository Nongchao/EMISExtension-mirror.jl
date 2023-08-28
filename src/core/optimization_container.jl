function PSI.lazy_container_addition!(
    container::PSI.OptimizationContainer,
    constraint::T,
    ::Type{U},
    axs...;
    meta=PSI.CONTAINER_KEY_EMPTY_META,
) where {T <: PSI.ConstraintType, U <: Union{PSY.Component, PSY.System}}
    if !PSI.has_container_key(container, T, U, meta)
        cons_container = PSI.add_constraints_container!(container, constraint, U, axs...; meta)
    else
        cons_container = PSI.get_constraint(container, constraint, U, meta)
    end
    return cons_container
end