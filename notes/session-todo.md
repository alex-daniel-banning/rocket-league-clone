[x] DETECTION - Implement similar set of unit tests to sphere v box, for box v box
[ ] Implement box v box resolution
    [ ] account for multiple contact points
    [ ] account for all types of collisions
        [ ] FACE_FACE
        [ ] EDGE_EDGE
        [ ] CORNER_FACE?
[ ] RESOLUTION - Implement similar set of unit tests to sphere v box, for box v box
    [ ] non-rotation tests
    [ ] rotation tests
[ ] Verify with demo
[ ] Implement box v plane collisions


TEST(SphereBoxRotation, OffCenterImpact_ProducesAngularVelocity) {
TEST(SphereBoxRotation, HeavierBox_RotatesLess) {
TEST(SphereBoxRotation, HeavierSphere_ImpartsMoreRotation) {
TEST(SphereBoxRotation, LargerLeverArm_CausesMoreRotation) {
TEST(SphereBoxRotation, TangentialImpact_CausesMoreRotation) {
TEST(SphereBoxRotation, HeadOnImpact_ProducesMinimalRotation) {
TEST(SphereBoxResolution, ZeroRelativeVelocity_OnlyAdjustsPosition) {
