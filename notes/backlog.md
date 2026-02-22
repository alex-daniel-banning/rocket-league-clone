# Project Backlog

## Doing (WIP limit: 3)
- [x] Build scene/match that will be useful for performance profiling
- [ ] Performance profiling

## On Deck
- [ ] FPS counter
- [ ] Code coverage reports (see code-coverage.md)

## Awaiting classification
- [ ] Code Coverage

## Core Features
- [ ] Collisions
    - [ ] Optimization
        - [ ] Broad phase (spatial partitioning, AABB)
        - [ ] Sleeping (objects that have settled down into a slow velocity/acceleration get flagged as "sleeping" and skip collisions entirely until something disturbs them)
    - [ ] Tunneling resolution
    - [ ] Friction with collisions
    - [ ] Standard/Shared/Relative epsilon values
    - [ ] Stress testing
- [ ] Car system
    - [ ] Steering/wheel system
    - [ ] Render hitbox and model
    - [ ] Jumping
    - [ ] Rocket boost
    - [ ] Dodging
    - [ ] Air roll/steering
- [ ] Gravity
- [ ] Menu system
    - [ ] Pause Menu
    - [ ] Main Menu
    - [ ] Settings
        - [ ] Camera Settings
- [ ] Camera system
### Rendering
- [ ] Translucent walls (alpha blending/render order/depth testing add complexity to this)

## Defects/Tech Debt
- [ ] Update clang-format: add IncludeBlocks (Regroup), IncludeIsMainRegex, and fix include group priorities
  IncludeBlocks: Regroup
  IncludeIsMainRegex: '([-_](test|unittest))?$'
  IncludeCategories:
    - Regex: '.*glad\.h'
      Priority: 1
    - Regex: '<GLFW/glfw3.h>'
      Priority: 2
    - Regex: '<.*>'
      Priority: 3
    - Regex: '".*"'
      Priority: 4
- [ ] Use double quotes for project #include directives instead of angle brackets
- [ ] Use BoxBuilder's for sphere/box collision tests (also put BoxBuilder in the same file as Box, like how Match::Builder is)
- [ ] Initializing match with objects inside each other might be causing issues (general match initialization error handling)
- [ ] Tick accumulator death spiral fix? Is it an issue?
- [ ] Move functions in implementation files to anonymous namespaces
- [ ] Clean up plane class. It shouldn't use getters and setters because especially because the fields should be const

## Misc (env, tooling, etc.)
- [ ] Consider refactoring main function/file
- [ ] What to do with stale demo files
- [ ] Find dead/unused code (static analysis or link-time optimization flags)
- [ ] Add logging system (including OpenGL API call tracing)
- [ ] Make quicklist shrink to fix small number of results

## Brainstorm
