# Live Data Stream

Arches can stream a TRaX simulation's progress over TCP to an external visualizer (e.g. RusticArches) while it runs: the architecture it built, counters for every logging interval, framebuffer snapshots, and an end-of-run summary. This document covers the seven commits on the `live-stream` branch (all 2026-09-28) that add the stream and fix the simulator bugs it exposed.

If no visualizer is listening, the simulator prints one line and runs exactly as before.

## Summary of changes

| Commit | Change |
| --- | --- |
| `6ad974e` | Add `LiveStream`: TCP client, background sender, HELLO / STATS / FRAME / DONE messages, `--live-*` params |
| `88e1919` | Add `--print-intervals=0` to skip the per-interval console report |
| `630d8c2` | Stream the final partial logging interval so streamed intervals sum to the end-of-run totals |
| `98ff356` | Fix L2 stall, occupancy and peak bandwidth divisors (one L2 per 8 TMs since Assignment 4) |
| `765db63` | Stream per-unit counters for heatmaps (`units`, `tp.issued_per_tp`, `tms_per_l2`) |
| `fef6e8d` | Never issue from a halted thread (TP issue counts and early retirement bug) |
| `30db119` | Add `Topology`: describe the simulated architecture in HELLO for the visualizer's block diagram |

Files touched:

| File | What changed |
| --- | --- |
| `src/arches-v2/util/live-stream.hpp`, `.cpp` | New: `LiveStream`, `Topology`, `JsonWriter` |
| `src/arches-v2/main.cpp` | `run_sim_trax`: topology recording, HELLO / STATS / FRAME / DONE, tail interval, L2 divisor fixes |
| `src/arches-v2/shared-utils.hpp` | New params `print-intervals`, `live-host`, `live-port`, `live-frame-interval` |
| `src/arches-v2/units/unit-tp.cpp` | Halted-thread fix in `UnitTP::clock_fall` |
| `src/arches-v2/CMakeLists.txt` | Link `ws2_32` on Windows |

Only `run_sim_trax` streams; it is the only simulation entry point in `main.cpp`.

## Usage

| Param | Default | Meaning |
| --- | --- | --- |
| `--live-host` | `127.0.0.1` | Visualizer address (IPv4 literal only, no hostnames) |
| `--live-port` | `7878` | Visualizer port; `0` disables the stream entirely |
| `--live-frame-interval` | `10` | Send a FRAME every N logging intervals; `0` = final frame only |
| `--print-intervals` | `1` | `0` skips the per-interval console report; counters are still collected |
| `--logging-interval` | `10000` | Existing param; one STATS message per interval, in core cycles |

Start the visualizer first, listening on the port, then run Arches. On startup Arches attempts a non-blocking connect with a 250 ms timeout and prints one of:

- `Live stream: connected to 127.0.0.1:7878`
- `Live stream: no visualizer at 127.0.0.1:7878, continuing without it`

If the visualizer disconnects mid-run, Arches prints `Live stream: visualizer disconnected` and keeps simulating with the stream off.

## How data flows

```
run_sim_trax (simulator thread)                 LiveStream worker thread         Visualizer
-------------------------------                 ------------------------         ----------
build units, record Topology
HELLO (config + topology)          --enqueue-->  queue (64 MB cap)  --TCP-->     block diagram
every logging interval:
  snapshot_units() (before delta_log resets)
  delta_log(...) per unit type
  STATS                            --enqueue-->                     --TCP-->     charts, heatmaps
  every N intervals: FRAME (read from DRAM)  -->  dropped if queue full
after execute(): tail STATS (partial interval)
after final report: FRAME + DONE, finish()  -->  drain queue, close socket
```

Key design points:

- **The simulator never blocks on the socket.** `send()` serializes the message and pushes it onto a queue; a worker thread does the blocking `send()` calls with `TCP_NODELAY` set.
- **Frames are dropped first.** If queuing a FRAME would push the queue past 64 MB, the frame is discarded. HELLO, STATS and DONE are always queued, so the counter series has no gaps.
- **Framebuffer reads bypass the memory system.** `read_framebuffer` reads the framebuffer's address range straight out of each DRAM partition with `direct_read`, one `partition_stride` chunk at a time, so mid-run frames show the image as it is being rendered without generating simulated traffic.
- **Per-unit counters are read before they are reset.** `snapshot_units()` runs at the top of the interval callback, before `delta_log` folds each unit's log into the running total and clears it.
- **TP counters are diffed, not reset.** TP logs are never reset (the final report and profile need them), so `send_live_stats` keeps the previous running totals and sends the difference.
- **`finish()` drains the queue.** The run ends with the final FRAME and DONE, then `finish()` waits for the worker to send everything before closing.

## Wire format

Every message is a length-prefixed frame; all integers are little-endian:

```
[u32 length][u8 type][payload: length - 1 bytes]
```

| Type | Id | Payload | When |
| --- | --- | --- | --- |
| HELLO | 1 | JSON: static configuration and topology | Once, before the simulation starts |
| STATS | 2 | JSON: counters for one logging interval | Every interval, plus one partial tail interval |
| FRAME | 3 | Binary: `u64 cycle`, `u32 width`, `u32 height`, then `width * height` RGBA8 pixels, bottom row first | Every `live-frame-interval` intervals, and once at the end |
| DONE | 4 | JSON: end-of-run summary | Once, after the final report |

### HELLO

| Field | Meaning |
| --- | --- |
| `version` | Protocol version, currently `1` |
| `arch`, `scene` | `arch-name` and `scene-name` params |
| `core_clock_hz`, `dram_clock_hz` | Clock rates |
| `logging_interval` | Cycles per STATS interval |
| `num_tms`, `num_tps`, `num_threads`, `num_partitions` | Unit counts (`num_tps` is per TM, `num_threads` per TP) |
| `framebuffer_width`, `framebuffer_height` | Frame size in pixels |
| `has_rt_core` | `1` if RT cores were built |
| `tms_per_l2` | TMs sharing each L2, for laying out TMs by L2 |
| `dram` | `units`, `peak_bytes_per_cycle` |
| `l3`, `l2`, `l1d` | `units`, `size`, `associativity`, `slices`, `banks`, `mshrs`, `latency`, `peak_bytes_per_cycle` |
| `instr_types` | Instruction type names; indexes the `tp` counter arrays in STATS |
| `topology` | Architecture description (see below) |

### Topology

`topology` is `{"groups": [...], "links": [...]}`. Each group is recorded in `run_sim_trax` right where its units are constructed, so the diagram always matches what was built.

A **group** is a set of identical units:

| Field | Meaning |
| --- | --- |
| `id`, `label` | Key and display name, e.g. `l1d` / `L1d$` |
| `count` | Total number of units |
| `parent` | Containing group; units are split evenly among the parent's units |
| `stats` | Key of this group's per-unit counters in STATS `units` (`l1d`, `l2`, `l3`, `dram`, `rtc`) or `tp`; empty if none |
| `props` | `[name, value]` string pairs shown in the diagram (cache size, associativity, latency, which instruction types a unit serves, ...) |

Groups recorded: `dram`, `l3`, `xbar`, `l2`, `atomic`, and inside each `tm`: `tp`, `sched`, `sfu`, `tu`, `rtc` (only with RT cores), `l1d`.

A **link** connects two groups:

| `map` | Meaning |
| --- | --- |
| `one` | Unit i to unit i |
| `group` | Consecutive blocks of from-units share one to-unit (block size = from count / to count) |
| `all` | Every unit to every unit (a crossbar) |
| `local` | Units within the same parent unit |

A link's `traffic` names the per-unit counters whose `bytes_read` measures data crossing it. Links recorded: `l3→dram` (one), `xbar→l3` and `l2→xbar` (all), `l1d→l2` (group), `sched→atomic` (all), and within each TM `tp→sched`, `tp→sfu`, `tp→tu`, `tp→l1d`, `tu→l1d`, plus `tp→rtc` and `rtc→l1d` with RT cores.

### STATS

All counters are deltas for the interval, not running totals.

| Field | Meaning |
| --- | --- |
| `cycle` | Simulator cycle at the end of the interval |
| `cycles` | Interval length; equals `logging_interval` except for the tail interval |
| `wall_time_s` | Wall-clock seconds since the simulation started |
| `threads_launched` | Running value of the atomic thread fetch counter |
| `dram` | `loads`, `stores`, `bytes_read`, `bytes_written`, `unique_loads`, `unique_rows` |
| `l3`, `l2`, `l1d` | Summed over all units: `hits`, `misses`, `half_misses`, `uncached_requests`, `mshr_stalls`, `bytes_read`, `tag_array_access`, `data_array_reads`, `data_array_writes` |
| `tu` | `texels`, `loads` |
| `rtc` | `rays`, `nodes`, `strips`, `tris`, `restarts`, `hits_returned` |
| `tp.issued`, `tp.resource_stalls`, `tp.data_stalls` | Arrays indexed by `instr_types`, summed over all TPs |
| `tp.issued_per_tp` | Instructions issued per TP, TM-major: index = `tm * num_tps + tp` |
| `units` | Per-unit counters for heatmaps (below) |

`units` holds one array entry per unit:

| Key | Arrays |
| --- | --- |
| `l1d` (one per TM), `l2`, `l3` (one per partition) | `hits`, `half_misses`, `misses`, `mshr_stalls`, `bytes_read` |
| `dram` (one per partition) | `bytes_read`, `loads`, `stores` |
| `rtc` (one per TM, empty without RT cores) | `rays` |

### DONE

| Field | Meaning |
| --- | --- |
| `cycles` | Frame cycles |
| `frame_time_s` | Simulated frame time |
| `wall_time_s` | Wall-clock simulation time |
| `power_w`, `energy_j` | Total power and energy from the final report |
| `rays` | Rays traced by RT cores, or framebuffer pixel count without RT cores |

## Simulator fixes made along the way

Streaming every interval exposed three problems in the existing reporting and simulation.

**Final partial interval (`630d8c2`).** `simulator.execute` only calls the logging callback on full intervals, so whatever ran after the last one never reached the stream and streamed totals fell short of the final report. After `execute` returns, a tail STATS covering `current_cycle % logging_interval` cycles is sent. The STATS building moved into a `send_live_stats` lambda shared by the interval callback and the tail.

**L2 unit counts (`98ff356`).** Since Assignment 4 there is one L2 per 8 TMs, but the per-interval report still divided by the old counts:

| Metric | Before | After |
| --- | --- | --- |
| L2 stalls | `/ num_tms / num_banks` (8x too low) | `/ l2s.size() / num_slices / num_banks` |
| L2 occupancy | `/ num_partitions / num_slices / num_banks` | `/ l2s.size() / num_slices / num_banks` |
| Peak L2 bandwidth | `num_partitions * num_banks * MAX_SIZE` | `l2s.size() * num_banks * MAX_SIZE` |

The old occupancy and bandwidth figures were only right when `num_tms / 8 == num_partitions`. The stall fix also adds the `num_slices` divisor, matching the occupancy formula.

**Halted threads issuing (`fef6e8d`).** When no thread was ready, `UnitTP::clock_fall` fell back to `_last_thread_id` even if that thread had halted, re-issuing its stale final `ret` every cycle. The per-TP heatmap made this visible:

- About 90% of TP issues in a 256x256 TRaX run were these phantom JUMPs.
- The halt logic re-ran each time, so `_num_halted_threads` could reach `_num_threads` while other threads were still running. The TP retired early, and a 128x128 run ended at 2251 cycles with no rays traced.

The fix attributes the cycle to the first live thread (`pc != 0`) instead, or returns idle once all threads have halted. This changes simulation results, not just the stream: issue counts and cycle counts from runs before this commit are affected.

## Known limitations

- **IPv4 literals only.** `live-host` goes through `inet_pton`, so `localhost` is rejected as an invalid host.
- **Winsock cleanup on failed connects.** `WSAStartup` runs before the connect attempt, but `WSACleanup` only runs in `_close()` after a successful connect, so a failed connect leaves Winsock initialized for the rest of the process.
- **No reconnect.** A visualizer that starts after Arches, or drops mid-run, gets nothing more from that run.
