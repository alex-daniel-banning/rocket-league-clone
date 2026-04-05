# HandleCollision Test Scenarios Reference

Scenarios from tests that used `Collisions::HandleCollision`, preserved as
reference for writing equivalent constraint-solver tests.

---

## Sphere vs Box (`sphere_vs_box_test.cpp`)

### Rotation
- **OffCenterImpact_ProducesAngularVelocity** — Sphere hits above center of box face; box should rotate counter-clockwise (+Z angular velocity).
- **HeavierBox_RotatesLess** — Same impact, heavier box rotates less.
- **HeavierSphere_ImpartsMoreRotation** — Heavier sphere imparts more angular velocity to box.
- **LargerLeverArm_CausesMoreRotation** — Impact farther from box center causes proportionally more rotation (~4x at 0.4 vs 0.1 offset).
- **TangentialImpact_CausesMoreRotation** — More tangential velocity direction causes more rotation than direct head-on.
- **HeadOnImpact_ProducesMinimalRotation** — Center-to-center collision produces negligible angular velocity.

### Resolution
- **ZeroRelativeVelocity_OnlyAdjustsPosition** — Matched velocities: no velocity change, but positions corrected to resolve penetration.

### Immovable Box (mass=0)
- **SphereBouncesOffImmovableBox** — Sphere bounces back; box velocity, position, and angular velocity unchanged.
- **OffCenterHit_ImmovableBoxUnchanged** — Off-center hit: box still unchanged, sphere deflected.
- **PenetrationCorrection_OnlySphereMovesOut** — Only sphere displaced to resolve penetration.
- **KineticEnergyConserved_ElasticBounce** — Sphere KE conserved when bouncing off immovable box.

---

## Conservation Laws (`conservation_laws_test.cpp`)

Parameterized across three scenarios (Approaching, SameDirection, OffCenter):
- **LinearMomentum** — Total linear momentum conserved.
- **AngularMomentum** — Total angular momentum (orbital + spin) conserved.
- **TotalKineticEnergy** — Total KE (linear + rotational) conserved (elastic collision).

---

## Box vs Box (`box_vs_box/resolution_test.cpp`)

### Basic Resolution
- **HeadOnCollision_ProducesNoRotation** — Head-on produces no angular velocity; velocities exchange correctly.
- **LighterBoxMovesFaster** — 1kg vs 3kg: lighter box gets more velocity.
- **SameDirectionCollision** — Faster box catches slower box: faster slows down, slower speeds up.
- **ZeroRelativeVelocity_ShouldCreateNoImpulse** — Matched velocities produce no impulse.
- **OffCenterImpact_ProducesAngularVelocity** — Off-center hit produces rotation in both boxes.
- **RotationOnlyCollision** — Spinning box hits stationary box: rotation transfers, linear velocity induced.

### Symmetry
- **OrientationIndependence** — Same collision rotated by arbitrary quaternion produces same speed magnitudes.
- **ContactTypeEquivalence** — Face-face, edge-face, and corner-face produce similar impulses for symmetric head-on.
- **Symmetry_SwappedArguments** — HandleCollision(a,b) == HandleCollision(b,a).
- **Symmetry_EqualMassHeadOn_VelocitiesExchange** — Equal mass head-on: velocities swap exactly.

### Coefficient of Restitution
- **CoefficientOfRestitution_Elastic** (e=1) — Total KE conserved.
- **CoefficientOfRestitution_PerfectlyInelastic** (e=0) — Relative contact-point velocity along normal is zero after collision.
- **CoefficientOfRestitution_Partial** (e=0.5) — Momentum conserved, KE decreases.

### Penetration Correction
- **PenetrationCorrection_FullySeparated** — Bodies no longer overlap after resolution.
- **PenetrationCorrection_ProportionalToInverseMass** — Lighter body displaced proportionally more.

### Edge Cases
- **EdgeCase_LargeMassRatio** — 1:1000 mass ratio: heavy box barely changes, light box flung away.
- **EdgeCase_GlancingCollision** — Small normal component: small impulse, tangential velocity mostly retained.
- **EdgeCase_MultipleContactPoints** — Face-face multi-point: momentum still conserved.

### Immovable Box (mass=0)
- **MovableBouncesOffImmovable** — Movable bounces; immovable unchanged.
- **PenetrationCorrection_OnlyMovableDisplaced** — Only movable displaced.
- **KineticEnergyConserved_ElasticBounce** — KE conserved.
- **Symmetry_ArgumentOrder** — Argument order doesn't matter.
- **TwoImmovables_ShouldThrowError** — Two immovable boxes colliding throws logic_error.

### No Contact
- **NoContact_NothingHappens** — Non-overlapping boxes: no velocity change.

---

## Friction (`friction_test.cpp`)

### Sphere vs Box Friction
- **ZeroFriction_TangentialVelocityUnchanged** — mu=0: tangential velocity unchanged.
- **NonZeroFriction_TangentialVelocityReduced** — mu=0.5: tangential velocity reduced but stays positive.
- **HigherFriction_MoreTangentialReduction** — mu=0.9 reduces tangential velocity more than mu=0.1.
- **Friction_GeneratesBoxAngularVelocity** — Sliding sphere torques movable box about expected axis.
- **CoulombClamp_LimitsFrictionImpulse** — Large tangential vs small normal velocity: Coulomb clamp prevents over-reduction.

### Box vs Box Friction
- **ZeroFriction_TangentialVelocityUnchanged** — mu=0: tangential velocity unchanged for both boxes.
- **NonZeroFriction_TangentialVelocityReduced** — mu=0.5: tangential velocity reduced/transferred.
