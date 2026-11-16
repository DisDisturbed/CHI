# Verilator simulation workspace

Run everything from this `sim/` directory.

```bash
make run  TB=random SEED=3     # build if needed, simulate, write an FST
make wave TB=random SEED=3     # open that run in GTKWave
make sim  TB=directed          # run, then open the waveform
make run  TB=rnf SEED=3        # RN-F dcache testbench
```

## Two PCs: Linux at home, Windows at work

The same `make` commands work on both. The Makefile picks the tools per PC:

| | Linux (home) | Windows (work) |
| --- | --- | --- |
| Shell | any terminal | PowerShell, cmd, Git Bash or an MSYS2 terminal (recipes always run in MSYS2's bash) |
| Verilator, g++ | `VERILATOR_ROOT` (default `/home/yusuf/verilator`) | MSYS2 UCRT64 packages, found in `%LOCALAPPDATA%\msys64` or `C:\msys64` |
| Binary | `build/.../VTb_HNF_Random` | `build/.../VTb_HNF_Random.exe` |

Anything that is not in the usual place goes in `sim/local.mk` (not in git, start
from `local.mk.example`). `make doctor` prints what this PC uses.

RTL and TB changes move between the PCs through GitHub:

```bash
make sync                      # commit this PC's changes, pull the other PC's, push
make sync M='fix TSHR retry'   # same, with your own commit message
make pull                      # only bring in the other PC's changes
```

Both PCs work on `main`; `make sync` always pushes to `main`.
Run `make pull` when you sit down at a PC and `make sync` before you leave it.
If both PCs changed the same lines, `make sync` stops and lists the files to fix.
Simulation logs in `out/` never stop a sync: this PC's log is kept. Build
directories and waveforms stay on each PC.

Files are stored with LF line endings on both PCs (`.gitattributes`).

| Command | What it does |
| --- | --- |
| `make build` | Compile only |
| `make run` | Compile if sources changed, then simulate |
| `make wave` | Open the matching `wave.fst` in GTKWave |
| `make sim` | `run`, then `wave` |
| `make lint` | Show all Verilator `-Wall` warnings without building |
| `make regress SEEDS='1 2 3'` | Run several seeds without waveforms and print one result line each |
| `make stress SEEDS='1 2 3'` | `regress` once per configuration in `STRESS_CFGS` (trackers 2/8, resets, long runs, `LOCK_ALL=1`, early Comp off) |
| `make clean` | Delete `build/` and `out/` |

| Option | Default | Meaning |
| --- | --- | --- |
| `TB` | `random` | `random` (top `Tb_HNF_Random`), `directed` (top `Tb_HNF`) or `rnf` (top `Tb_RNF`, the RN-F dcache) |
| `SEED` | `1` | Seed for `TB=random` and `TB=rnf` |
| `G` | empty | Other top-level parameters, e.g. `G='NUM_ROUNDS=2 RUN_SCENARIOS=0'` |
| `WAVES` | `1` | `WAVES=0` skips the waveform |
| `JOBS` | `8` | Parallel Verilator and C++ compile jobs |
| `ARGS` | empty | Extra runtime plusargs, e.g. `ARGS=+deadlock_cycles=50` |

## Source folders

```
common/RTL/    pgk_file.sv: CHI packages and channel interfaces, used by both nodes
common/TB/      chi_tb_pkg.sv (scoreboard), rnf_model.sv, snf_model.sv, tshr_deadlock_mon.sv
HN-F/RTL/       the HN-F
HN-F/TB/        Randomized/ (Tb_HNF_Random) and Directed/ (Tb_HNF)
RN-F/RTL/       the RN-F dcache: rnf_pkg.sv (core-side package + interface), rnf_dcache.sv,
                rnf_tag_ram.sv, rnf_data_ram.sv, rnf_link.sv (RX FIFO, TX port)
RN-F/TB/        TB_rnf.sv (Tb_RNF), rnf_core_driver.sv, rnf_chi_checker.sv, rnf_tb_pkg.sv (scenarios)
```

## RN-F dcache testbench (`TB=rnf`)

`Tb_RNF` puts the RTL dcache (RN-F 0, core side driven by `rnf_core_driver`) and one
behavioural `rnf_model` (RN-F 1) on the HN-F RTL, with the behavioural SN-F. Both RN-Fs
work on the same lines, so they snoop each other through the HN-F.

```bash
make run     TB=rnf SEED=3
make regress TB=rnf SEEDS='1 2 3 4' G='SETS=4 WAYS=2'
```

### What runs

1. **Directed scenarios** (`RN-F/TB/rnf_tb_pkg.sv`, `RS_*`). The core driver runs them and
   orders the other RN-F's ops one by one through `sb.rop_q` (`CMD_REMOTE` in `rnf_model`),
   so the two sides can be lined up against each other.

   | Scenario | What it does |
   | --- | --- |
   | `PINGPONG` | One line moves back and forth between the two RN-Fs |
   | `SNOOP_MATRIX` | Every snoop type onto a Shared Clean and onto a Unique Dirty dcache line |
   | `WB_SNOOP_RACE` | A dirty line is replaced while the other RN-F asks for every line of that set |
   | `EVICT_SNOOP_RACE` | Same with clean lines (Evict) |
   | `UPGRADE_RACE` | Store on a Shared line while the other RN-F goes for the same line |
   | `SET_THRASH` | Back-to-back walk over more lines than ways of one set |
   | `SNOOP_IN_WAIT` | Slow memory: snoops arrive while the dcache waits for its own request |
   | `NC_MIX` | Non-cacheable accesses of both RN-Fs on the same lines |
   | `PRIV_BURST` | Store / load bursts on the private lines |
   | `SUBWORD_MERGE` | 8/16/32-byte windows (or scattered sub-words) built from smaller stores that are all in flight together, from a line not held / Shared / Unique Dirty / held dirty by the other RN-F, read back wide and narrow |
   | `SUBWORD_LANES` | Every aligned position of every width stored, then read back at every width |
   | `NC_SUBWORD` | Non-cacheable sub-word stores to disjoint bytes, in flight together, read back |
   | `OOO_BURST` | Misses and slow non-cacheable accesses first, hits behind them, full in-flight window |
   | `B2B_SAME_LINE` | Back-to-back loads / stores to one line (merges and hazards) while the other RN-F takes the line away |
   | `B2B_SET_CONFLICT` | Every line of one set in flight at once: misses that need a victim while others are open |
   | `HAZARD_CHAIN` | RAW / WAR / WAW chains on the same bytes with independent accesses between them |
   | `NC_ERR` | One non-cacheable line answered with an error by the SN-F, mixed into bursts |

2. **Random rounds.** The behavioural RN-F runs its random ops; the core driver mixes single
   blocking accesses with idle gaps, pipelined bursts (any line, one line, one set), store
   then load of the same bytes, sub-word merge windows, misses with hits queued behind them
   and drain points. Per round the top re-rolls the HN-F/SN-F knobs, the behavioural RN-F's
   request mix (TxnID overlap, zero byte enables, several non-coherent ops per line,
   unaligned NC/partial writes, partial states) and the core's knobs (in-flight window 1..16,
   store / NC / private mix, width weights, gaps).

   **The core model** (`rnf_core_driver`) is a pipelined, out-of-order core: up to
   `CORE_MAX_OUT` requests in flight with their own `idx`, the next request on the pins right
   after the previous one is accepted, responses matched by `idx` in any order. It holds a
   request back only for a byte hazard with one in flight (RAW / WAR / WAW); requests to other
   bytes of the same line go out together. Accesses are 1/2/4/8 bytes, always aligned to
   their size (an unaligned access traps in the core and never reaches the dcache). How many
   requests the dcache takes at once is measured, never relied on.
3. **Flush.** The dcache has no flush of its own: the behavioural RN-F sends CleanInvalid for
   every line, which pulls the dcache's dirty lines out through snoops.

### What is checked

- **Scoreboard** (`chi_tb_pkg`): every load, SWMR on the dcache's line states (mirrored from
  its tag writes), the memory image after the flush.
- **Private lines.** 12 lines only the dcache touches, in the same sets as the shared ones.
  The driver keeps its own byte-exact copy: a load must return exactly the bytes stored last.
  It also remembers which store wrote each byte, so a load that returns the bytes of several
  sub-word stores (also ones that were in flight together) counts as a merge check.
- **Core side**: every response answers an accepted request with that `idx`, echoes its
  `op` / `src`, an `idx` is never reused while open, `chr_state` is I for non-cacheable
  accesses, `err` only on lines the SN-F was told to fail.
- **`rnf_chi_checker`**: looks only at the dcache's pins and keeps its own view of each line.
  Request fields and legal start state, CompAck exactly once with the right TxnID, write data
  format and its Resp against the line's state, every snoop answered once, in time, legally for
  its type and for the state the line had, dirty data never dropped, a store only completing
  on a Unique line, link credits.
- **Coverage** (keys `dc ...` and `chk ...` in the summary): snoop type x line state, core op x
  lookup result, which state the request FSM was in when a snoop was served, write-back lost
  to a snoop, store upgrade that lost its copy, and so on. With the full scenario range every
  cell listed in `check_coverage()` must have been reached or the run fails with `COVERAGE`.
  Only cells this HN-F can produce are required: it grants SC to ReadShared and UC to
  ReadUnique, so the dcache never holds a line as SD, and UC only between fill and store.
- **Black-box coverage** (keys `bb core ...`, `core ...`, `drv ...`, `merge ...`), from the
  pins and the reference models only, also required: every width x cacheable/NC x load/store,
  every aligned lane of every width, core op x the line's bus state at accept, hit / miss /
  victim write-back / Evict seen on the bus, snoops on the line of an open core request,
  RAW / WAR / WAW stalls, sub-word stores to one line in flight together, byte-exact merge
  checks, NC error responses. `REQUIRE_CORE_OOO=1` also requires what only a dcache that
  takes several requests at once can produce (2+ accepted, responses out of order).
- At the end the dcache must be idle and empty, with nothing open on the bus.

| `G` parameter | Default | Meaning |
| --- | --- | --- |
| `RUN_SCENARIOS` | `1` | Run the directed scenarios |
| `FIRST_SCENARIO` / `LAST_SCENARIO` | `1` / `17` | Scenario range (the coverage matrix needs the full range) |
| `CORE_MAX_OUT` | `16` | Requests the core keeps in flight at most (at most 2**ReqIdxWidth) |
| `REQUIRE_CORE_OOO` | `0` | Also require several requests accepted at once and out-of-order responses |
| `CHECK_COVERAGE` | `1` | Fail on unreached coverage cells |
| `NUM_ROUNDS` / `OPS_PER_ROUND` | `8` / `50` | Random rounds, behavioural RN-F ops per round |
| `CORE_OPS_MULT` | `4` | Core accesses per behavioural RN-F op |
| `NC_PCT` / `PRIV_PCT` | `10` / `25` | Percent of core accesses that are non-cacheable / to private lines |
| `SETS` / `WAYS` | `16` / `4` | dcache geometry (powers of two, at least 2) |
| `RX_FIFO_DEPTH` | `4` | dcache RX FIFO depth per channel (2..15) |
| `MID_RESETS` | `0` | Rounds that are reset in the middle of traffic |
| `NUM_TRACKERS` | `4` | HN-F trackers |

The replacement scenarios need more lines in a set than the dcache has ways (9 or 10 lines
per set here). With `SETS=64 WAYS=8` the cell "write-back: line lost to a snoop" is not
reached on every seed, and bigger caches never replace a line at all; both show up as
`COVERAGE` failures.

Not tested here: the dcache's retry path. The HN-F in this repo never sends RetryAck to an
RN-F.

Only a Surfer signal list exists for this testbench (`waves/rnf.sucl`).

## Where files go

```
filelists/     source lists: rtl.f (HN-F RTL), random.f, directed.f, rnf.f
support/       sim_dump.sv, which enables FST dumping
waves/         GTKWave views, one per testbench (File > Write Save File updates them)
build/<tb>/<params>/   compiled model and verilator.log (not in git)
out/<tb>/<params>/     sim.log and wave.fst (not in git)
```

For example, `make run TB=random SEED=3` writes `out/random/SEED=3/wave.fst`.

## Surfer

```bash
make wave VIEWER=surfer TB=random SEED=3
```

Leave the window open: when `make run` finishes for the same options, Surfer
refreshes by itself and keeps the signal list and cursor. Zoom may reset.

| Action | Key |
| --- | --- |
| Zoom in / out / fit | `Ctrl+=` / `Ctrl+-` / `Ctrl+0` |
| Next / previous transition | Select a signal, then `Right` / `Left` |
| Zoom at the mouse | `Ctrl+wheel` |
| Command prompt (add signals by name, and more) | `Space` |
| Keyboard help | `show_controls` in the command prompt |

- Starter signal lists are `waves/<tb>.sucl`, settings are `.surfer/config.toml`.
- A mistake in `config.toml` stops Surfer from starting, and `make wave` hides the
  message. Run `surfer` from `sim/` in a terminal to see it.
- Arrays of vectors sit one level deeper than they look:
  `dut.trk_txnid.trk_txnid`, with rows named `[0] [11:0]`. A `divider_add` line
  above them keeps track of which array they are.
- A red `ERROR (00000000)` on an enum means the value is not in the enum. For
  `opcode` this is just the idle value 0.

## GTKWave for xsim users

| In xsim | Here |
| --- | --- |
| Relaunch simulation | `make run` in the terminal, then `F5` in GTKWave. Signals and zoom stay. |
| `.wcfg` waveform config | `waves/<tb>.gtkw`, saved with `Ctrl+S` |
| Scope and Objects panes | SST panel: click a scope, select signals, press Append |
| Zoom in / out / fit | `Ctrl+=` / `Ctrl+-` / `Ctrl+0` |
| Next / previous transition | Select a signal, then `Ctrl+Right` / `Ctrl+Left` |
| Mouse wheel | Scrolls signals. `Ctrl+wheel` zooms at the cursor. |
| Value column | Click in the waves: values appear beside the names at the marker |
| Radix and enum names | `Alt+X` hex, `Alt+D` decimal, `Alt+B` binary, `Alt+E` enum names |
| Menu bar | The menu is the `☰` button at the top left |

Settings live in `support/gtkwaverc`.

**There is no red X.** Verilator is a 2-state simulator, so unknown, uninitialized
and undriven values appear as 0 or 1. A clean-looking signal can be one xsim would
show as X, so check reset and initialization in xsim if they are in doubt.

Unpacked arrays and interfaces appear as flat lists of signals rather than xsim's
expandable tree.

## Random TB switches

All are `G=` parameters, default in brackets. Each feature switch turns on a
spec-legal behaviour of the RN-F / SN-F models, so a failure can be pinned to
one feature by turning it off: `make run SEED=3 G='SNF_COMP_EARLY=0'`.

| Parameter | Meaning |
| --- | --- |
| `NUM_TRACKERS` [4] | HN-F trackers. 1 starves (see TSHR_FIXES 11) |
| `LOCK_ALL` [0] | 1: one op per line in the whole system, hides every same-line race |
| `TXNID_COLLISION_STRESS` [1] | both RN-Fs use the same TxnIDs in random rounds |
| `MID_RESETS` [4] | random rounds cut by a reset with traffic in flight |
| `SNF_EARLY_COMP` [1] | 0 turns off both of: |
| `SNF_COMP_FIRST` [1] | SN-F sends a separate Comp before DBIDResp |
| `SNF_COMP_EARLY` [1] | SN-F sends a separate Comp after DBIDResp, before the write data |
| `SNF_PCRD_EARLY` [1] | SN-F sends PCrdGrant before RetryAck, PCrdType 0..15 |
| `COMPACK_OPT` [1] | ExpCompAck=1 on ReadOnce / ReadNoSnp / WriteUnique / WriteNoSnp |
| `SNP_OPT_DATA` [1] | a clean line answers SnpUnique (RetToSrc=0) with data |
| `ZERO_BE` [1] | partial writes with every byte enable 0 |
| `NC_MULTI` [1] | one RN-F has several ReadNoSnp / WriteNoSnp to one line in flight |
| `UNALIGNED` [1] | ReadNoSnp / WriteNoSnpPtl / WriteUniquePtl with an address not aligned to Size |
| `CLEAN_UNIQUE` [1] | CleanUnique: SC -> UC, SD -> UD, I -> UCE (with `PARTIAL_STATES`) |
| `MAKE_UNIQUE` [1] | MakeUnique followed by a full-line store (-> UD) |
| `EVICT` [1] | Evict for clean lines (SC / UC / UCE), besides silent eviction |
| `READ_NSD` [1] | ReadNotSharedDirty (CompData SC / UC / UD_PD, never SD) |
| `PARTIAL_STATES` [1] | UCE / UDP: partial stores, WriteBackPtl, SnpRespDataPtl |
| `CHECK_COVERAGE` [1] | fail the run if a stress it should reach was never reached |
| `ISOLATE_FAILURES` [1] | reset after a command that failed, so one bug does not cascade |

At the end of every run the TB also checks that every tracker is idle, the
P-credit pool is empty, no hazard-order bit is left set, and every HN-F link
credit counter agrees with the agent on the other end of that link.

## Things to know

- **`SEED` and `G` are compile-time parameters.** Each new combination gets its
  own build (about 10 s and 190 MB). `make clean` removes them all.
- **A finished run is not a passing run.** The testbenches call `$finish` even
  after failures, so `make` still succeeds. Read the summary at the end of the
  log.
- **Waveforms need no testbench changes.** `sim_dump.sv` is bound into the top
  module, and a new top only needs a block in the `Makefile` and a filelist.
- **Deadlock monitor.** `common/TB/tshr_deadlock_mon.sv` is bound into every
  `TSHR` in both testbenches. It prints `DEADLOCK:` when a tracker stays in the
  same non-IDLE state for 100 cycles, with what that state waits for and the
  request header. It also prints `RELEASED` or
  `CLEARED BY RESET` when that tracker moves on, and a `DEADLOCK AT END OF SIM`
  line for every tracker still stuck. Use `+deadlock_cycles=N` to change the
  threshold and `+no_deadlock_mon` to turn it off, passed through `ARGS`:
  `make run TB=random SEED=1 ARGS=+deadlock_cycles=50`. The SNF_SLOW scenario
  has SN-F latencies above 100 cycles, so a `DEADLOCK:` followed by `RELEASED`
  there is only a slow memory.
- **`HN-F/TB/Randomized/chi_scoreboard.sv` is not compiled.** It is unused: the active
  scoreboard class is inside `chi_tb_pkg.sv`.
- **Do not put "Verilator" first in a `/* */` or `//` comment line.** Verilator
  reads it as a pragma and fails with `BADVLTPRAGMA`.

## Changes `common/TB/rnf_model.sv` needs for Verilator

Verilator inlines every task call. The many calls to `do_op` and `run_*` grow the
design tree from 2 MB to 1.5 GB, and elaboration never finishes. Two changes keep
it working, and both are plain SystemVerilog or comments to other simulators:

1. `run_read`, `run_write`, `run_cmo`, `do_op` and `do_op_f` carry a
   `/*verilator no_inline_task*/` comment. The Makefile's `-Wno-IMPURE` allows it.
2. The TX tasks no longer write interface pins directly. They set `*_d_*`
   variables and fire an event, and `always @(req_drive)` style blocks drive the
   pins.

**Keep rule 2 when editing that file.** Verilator does not schedule logic that
depends on signals written inside a `no_inline_task` task. A direct write compiles
without complaint, but the DUT sees stale values and you get false failures.

## TB pins change 1 ns after the clock edge

Every model (random and directed) drives its pins with `<= #1`, and the directed
models sample DUT outputs on the falling edge. Without the delay, Verilator
sometimes lets a TB write at a rising edge reach the DUT before the DUT's flops
sample that same edge, and sometimes not. A one-cycle flit is then seen twice
or not at all (a lost last data beat looks like a deadlock). **Keep the `#1`
when adding a new driver.**
