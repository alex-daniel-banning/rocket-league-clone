[x] DETECTION - Implement similar set of unit tests to sphere v box, for box v box
[ ] Implement box v box resolution
    [x] account for multiple contact points
    [ ] write tests for multiple contact points
    [ ] account for all types of collisions
        [ ] FACE_FACE
        [ ] EDGE_EDGE
        [ ] CORNER_FACE?
[ ] RESOLUTION - Implement similar set of unit tests to sphere v box, for box v box
    [ ] non-rotation tests
    [ ] rotation tests
[ ] Verify with demo
[ ] Implement box v plane collisions

Random
    [ ] Disable plain language suggestions, or disable "Enter" selecting the first one
    [ ] Add binding to add functions in current file to quick list
    [ ] Make quicklist shrink to fix small number of results
    [ ] Enfore clean argument listing format
    [ ] How to git add . from inside nvim? Worth it?


box_a (incident)    box_b (reference)   Approach
face                face                4 verts, clip against 4 planes
edge                face                2 verts, clip against 4 planes
face                edge                swap roles, then 2 verts, clip against 4 planes
edge                edge                edge-edge closest points


Reference vs Incident object:
    reference = wall/surface
    incident = the thing hitting the wall

    Penetration axis should point from reference to incident (it's the direction you'd push the incident object to get it out of the wall)
    reference -> defines the clippling planes
    incident -> it's vertices get clipped
