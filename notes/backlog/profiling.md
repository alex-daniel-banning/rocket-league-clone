# Performance Profiling

## Build

Profile against a `RelWithDebInfo` build — optimizations on so hot paths are
realistic, debug symbols on so `perf` can resolve function names.

```bash
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
```

## Permissions

Linux restricts perf by default. Check `/proc/sys/kernel/perf_event_paranoid`
— if it's above `1`, lower it:

```bash
# Temporary (resets on reboot)
sudo sysctl kernel.perf_event_paranoid=1

# Permanent
echo 'kernel.perf_event_paranoid=1' | sudo tee -a /etc/sysctl.conf
```

`1` is sufficient for user-space profiling.

## Run with perf

```bash
# Record. -g captures call graphs. Exit the demo with Escape when done.
perf record -g ./build/bin/perf_demo

# View results
perf report
or
perf report --dsos perf_demo
```

In `perf report`: arrow keys to navigate, `+` to expand call chains.

## Alternative: Callgrind (GUI)

Instruments every instruction — accurate but ~50× slower, so simulation
timing won't be realistic. Use for call-count analysis, not timing.

```bash
valgrind --tool=callgrind ./build/bin/perf_demo
kcachegrind callgrind.out.*
```

## Demo scene

`src/demos/perf/perf_demo.cpp` — 1 ball, 6 boxes, fully enclosed room
(ground + ceiling + 4 walls). No auto-reset; runs indefinitely.
