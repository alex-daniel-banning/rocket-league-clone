[ ] Verify with demo (box_v_box_resolution_demo)
    [x] New file: src/demos/collisions/box-v-box/box_v_box_resolution_demo.cpp
    [x] Add to src/demos/CMakeLists.txt
    [ ] CLI arg selects scenario, --list prints available
    [ ] Auto-reset after a few seconds (like OBB demo)
    [ ] Scale up sizes/positions ~3x for visibility
    [ ] Scenarios:
        [ ] head_on — equal mass, no rotation
        [ ] mass_ratio — 1:3 mass difference
        [ ] off_center — produces angular velocity
        [ ] rotation_only — spin transfer
        [ ] edge_face — 45deg rotated box
        [ ] corner_face — arbitrarily rotated box
        [ ] large_mass — 1:1000 ratio
        [ ] glancing — mostly tangential
        [ ] velocity_exchange — classic equal-mass billiard
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

Reference vs Incident object:
    reference = wall/surface
    incident = the thing hitting the wall

    Penetration axis should point from reference to incident (it's the direction you'd push the incident object to get it out of the wall)
    reference -> defines the clippling planes
    incident -> it's vertices get clipped
