Sequential Impulse
[x] Constraint struct
[x] Store constraints in a list (for now create the list each frame)
[x] Finish Presolve (position error / Baumgarte bias)
[x] Implement Solve + Apply Impulse + Compute JV
[ ] (Wire up body references) Store the bodies in a map for the constraint solver to access
    - `std::unordered_map<int, std::variant<physics::Box*, physics::Sphere*>>` in Match
    - Solver looks up bodies by ID from ContactConstraint::body_a_id / body_b_id
    - Use `std::visit` with generic lambda to apply impulses uniformly (works once Sphere has angular_velocity)
[x] Add angular_velocity and inertia_tensor_inv to Sphere
