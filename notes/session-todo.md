[x] DETECTION - Implement similar set of unit tests to sphere v box, for box v box
[ ] Implement box v box resolution
    [x] account for multiple contact points
        [x] See if there are other places where we compare against zero, and we could use an epsilon value
        [x] flipped axes via rotation don't mess things up (tested with rotation at 90 and 180 degrees)
    [x] write tests for multiple contact points
    [x] account for all types of collisions
        [x] FACE_FACE
        [x] EDGE_EDGE
        [x] CORNER_FACE?
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
Backlog
    [ ] Figure out what I want epsilon to be across code base
        [ ] Possibly make it relative. Research this...

Reference vs Incident object:
    reference = wall/surface
    incident = the thing hitting the wall

    Penetration axis should point from reference to incident (it's the direction you'd push the incident object to get it out of the wall)
    reference -> defines the clippling planes
    incident -> it's vertices get clipped
