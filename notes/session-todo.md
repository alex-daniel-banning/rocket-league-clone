[ ] Figure out what to do about failing sphere_v_plane_collision_demo executable
[ ] Tick accumulator death spiral fix? Is it an issue?
[ ] Implement box v plane collisions

Random
    [ ] Disable plain language suggestions, or disable "Enter" selecting the first one
    [ ] Add binding to add functions in current file to quick list
    [ ] Make quicklist shrink to fix small number of results
    [ ] Enfore clean argument listing format
    [ ] How to git add . from inside nvim? Worth it?
    [ ] Rethink clang format (it doesn't allow flexibity of builder pattern formatting)
Backlog
    [ ] Figure out what I want epsilon to be across code base
        [ ] Possibly make it relative. Research this...
    [ ] Error handling when objects spawn inside each other

Reference vs Incident object:
    reference = wall/surface
    incident = the thing hitting the wall

    Penetration axis should point from reference to incident (it's the direction you'd push the incident object to get it out of the wall)
    reference -> defines the clippling planes
    incident -> it's vertices get clipped
