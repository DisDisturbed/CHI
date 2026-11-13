# Verilator simulation workspace

Run everything from this `sim/` directory.

```bash
make run  TB=random SEED=3     # build if needed, simulate, write an FST
make wave TB=random SEED=3     # open that run in GTKWave
make sim  TB=directed          # run, then open the waveform
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
| `TB` | `random` | `random` (top `Tb_HNF_Random`) or `directed` (top `Tb_HNF`) |
| `SEED` | `1` | Seed for `TB=random` |
| `G` | empty | Other top-level parameters, e.g. `G='NUM_ROUNDS=2 RUN_SCENARIOS=0'` |
| `WAVES` | `1` | `WAVES=0` skips the waveform |
| `JOBS` | `8` | Parallel Verilator and C++ compile jobs |
| `ARGS` | empty | Extra runtime plusargs, e.g. `ARGS=+deadlock_cycles=50` |

## Where files go

```
filelists/     source lists: rtl.f, random.f, directed.f
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
- **Deadlock monitor.** `TB/Common/tshr_deadlock_mon.sv` is bound into every
  `TSHR` in both testbenches. It prints `DEADLOCK:` when a tracker stays in the
  same non-IDLE state for 100 cycles, with what that state waits for and the
  request header. It also prints `RELEASED` or
  `CLEARED BY RESET` when that tracker moves on, and a `DEADLOCK AT END OF SIM`
  line for every tracker still stuck. Use `+deadlock_cycles=N` to change the
  threshold and `+no_deadlock_mon` to turn it off, passed through `ARGS`:
  `make run TB=random SEED=1 ARGS=+deadlock_cycles=50`. The SNF_SLOW scenario
  has SN-F latencies above 100 cycles, so a `DEADLOCK:` followed by `RELEASED`
  there is only a slow memory.
- **`TB/Randomized/chi_scoreboard.sv` is not compiled.** It is unused: the active
  scoreboard class is inside `chi_tb_pkg.sv`.
- **Do not put "Verilator" first in a `/* */` or `//` comment line.** Verilator
  reads it as a pragma and fails with `BADVLTPRAGMA`.

## Changes `TB/Randomized/rnf_model.sv` needs for Verilator

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
