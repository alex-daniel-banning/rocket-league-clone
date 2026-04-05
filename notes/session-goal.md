[ ] Redo HandleCollision tests for constraint solver (see notes/handle-collision-test-scenarios.md)
[ ] Jacobian is unpacked in ApplyImpulse and ComputeJV, consider a helper method
[ ] add restitution to bias
[ ] BUG: bodies map passed by value in Solve/ComputeJV/ApplyImpulse/GenerateConstraintsFromContact — velocity changes thrown away. Pass by & or const&.
[ ] BUG: contact.body_a_id / body_b_id never set on Contact before passing to GenerateConstraintsFromContact (uninitialized)
[ ] body_a_id / body_b_id type mismatch: ContactConstraint uses long, Contact and body map use int. Make consistent (int).
[ ] Stale include in match.cpp: "engine/physics/constraint_solver.hpp" should be "engine/physics/contact_constraint_solver.hpp"
[ ] Delete or convert damping block comment to a TODO
