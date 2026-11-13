SHELL := /bin/bash
# Verilator workspace for the CHI HN-F. See sim/README.md.
#   make run  TB=random SEED=3     build + simulate
#   make wave TB=random SEED=3     open that run in GTKWave

# ---- per-PC tool setup ------------------------------------------------------
# The same commands work on the Linux home PC and on the Windows work PC. Tool
# locations are found here; anything machine-specific that is not found goes in
# local.mk (not in git, copy local.mk.example). `make doctor` shows what is used.
-include local.mk

# MSYS2 make reports MAKE_HOST x86_64-pc-msys; native Windows make reports Windows32.
MSYS_MAKE := $(findstring msys,$(MAKE_HOST))$(findstring cygwin,$(MAKE_HOST))

ifneq ($(filter Windows_NT,$(OS))$(MSYS_MAKE),)
  # Windows: PowerShell, cmd, Git Bash or an MSYS2 terminal; tools from MSYS2 UCRT64.
  HOST       := windows
  EXE        := .exe
  MSYS2_ENV  ?= ucrt64
  # First of: this MSYS2 terminal's own root, %LOCALAPPDATA%/msys64, C:/msys64, D:/msys64
  # that has Verilator. (Git Bash's root has no Verilator, so it is skipped.)
  MSYS2_ROOT ?= $(patsubst %/$(MSYS2_ENV)/bin/verilator_bin.exe,%,$(firstword $(wildcard \
                  $(addsuffix /$(MSYS2_ENV)/bin/verilator_bin.exe, \
                    $(if $(MSYS_MAKE),$(patsubst %/,%,$(shell cygpath -m /))) \
                    $(subst \,/,$(LOCALAPPDATA))/msys64 C:/msys64 D:/msys64))))
  ifeq ($(wildcard $(MSYS2_ROOT)/$(MSYS2_ENV)/bin/verilator_bin.exe),)
    $(error Verilator not found under MSYS2_ROOT='$(MSYS2_ROOT)'. Install it with 'pacman -S mingw-w64-ucrt-x86_64-verilator' or set MSYS2_ROOT in sim/local.mk)
  endif
  # g++, make and the wave viewers come from the same place as Verilator.
  ifneq ($(MSYS_MAKE),)
    # make from an MSYS2 terminal: POSIX paths, /bin/bash exists.
    export PATH := $(shell cygpath -u '$(MSYS2_ROOT)/$(MSYS2_ENV)/bin'):$(PATH)
  else
    # Native Windows make (PowerShell, cmd, Scoop, Git Bash): run every recipe
    # in MSYS2's bash, with MSYS2's tools first on the Windows-style PATH.
    SHELL := $(MSYS2_ROOT)/usr/bin/bash.exe
    .SHELLFLAGS := -c
    export PATH := $(subst /,\,$(MSYS2_ROOT)/$(MSYS2_ENV)/bin;$(MSYS2_ROOT)/usr/bin);$(PATH)
  endif
  VERILATOR_ROOT ?= $(MSYS2_ROOT)/$(MSYS2_ENV)/share/verilator
  VERILATOR      ?= verilator_bin
else
  HOST           := linux
  EXE            :=
  VERILATOR_ROOT ?= /home/yusuf/verilator
  VERILATOR      ?= $(VERILATOR_ROOT)/bin/verilator
endif
export VERILATOR_ROOT
GTKWAVE   ?= gtkwave
SURFER    ?= surfer
VIEWER    ?= surfer

TB    ?= random
SEED  ?= 1
SEEDS ?= 1 2 3 4
G     ?=
WAVES ?= 1
JOBS  ?= 8
ARGS  ?=

ifeq ($(TB),random)
  TOP      := Tb_HNF_Random
  ALL_G    := SEED=$(SEED) $(G)
else ifeq ($(TB),directed)
  TOP      := Tb_HNF
  ALL_G    := $(G)
else
  $(error Unknown TB '$(TB)'. Use TB=random or TB=directed)
endif

# Parameters are fixed at compile time, so every distinct set gets its own
# build directory and output directory, named after the parameters.
TAG   := $(if $(strip $(ALL_G)),$(subst $(eval) ,_,$(strip $(ALL_G))),default)
BUILD := build/$(TB)/$(TAG)
OUT   := out/$(TB)/$(TAG)
BIN   := $(BUILD)/V$(TOP)$(EXE)
FST   := $(OUT)/wave.fst
GTKW  := waves/$(TB).gtkw

SRCS  := $(wildcard ../RTL/*.sv ../TB/Randomized/*.sv ../TB/Directed/*.sv ../TB/Common/*.sv) \
         $(wildcard filelists/*.f) support/sim_dump.sv Makefile

# IMPURE: rnf_model.sv uses no_inline_task on tasks that reach sb.*, see README.
VFLAGS := --binary --timing -sv --top-module $(TOP) \
          --trace-fst --trace-structs --trace-params \
          -Wno-fatal -Wno-IMPURE \
          +define+SIM_TOP=$(TOP) \
          -j $(JOBS) --Mdir $(BUILD) -o V$(TOP) \
          $(addprefix -G,$(ALL_G)) \
          -f filelists/$(TB).f support/sim_dump.sv

# Waves go to a temp file and are copied over wave.fst at the end, so a viewer
# watching wave.fst does not read a half-written file. The copy is in place
# because Surfer stops noticing changes if the file is replaced by a rename.
PLUS := $(if $(filter 1,$(WAVES)),+wave=$(FST).tmp,+no_wave)

.PHONY: help build run wave sim lint regress stress clean doctor sync pull
.DEFAULT_GOAL := help

help:
	@echo "make sync    [M='message']  pull the other PC's changes, commit and push this PC's"
	@echo "make pull    only pull the other PC's changes"
	@echo "make doctor  show which tools this PC uses"
	@echo "make build   TB=random|directed [SEED=n] [G='NAME=val ...']"
	@echo "make run     same options, plus WAVES=0 to skip the waveform"
	@echo "make wave    open the run with the same options, VIEWER=gtkwave|surfer"
	@echo "make sim     run, then wave"
	@echo "make lint    all Verilator warnings, no build"
	@echo "make regress TB=random SEEDS='1 2 3' (no waveforms, prints a summary)"
	@echo "make stress  SEEDS='1 2 3' (regress over every configuration in STRESS_CFGS)"
	@echo "make clean   delete build/ and out/"

$(BIN): $(SRCS)
	@mkdir -p $(BUILD)
	$(VERILATOR) $(VFLAGS) > $(BUILD)/verilator.new 2>&1 || { cat $(BUILD)/verilator.new; exit 1; }
	@# Verilator skips up-to-date work and prints nothing: keep the old log, mark the binary current.
	@if [ -s $(BUILD)/verilator.new ]; then mv $(BUILD)/verilator.new $(BUILD)/verilator.log; else rm -f $(BUILD)/verilator.new; fi
	@touch $(BIN)
	@echo "built $(BIN) ($$(grep -c '^%Warning' $(BUILD)/verilator.log) warnings, see $(BUILD)/verilator.log)"

build: $(BIN)

run: $(BIN)
	@mkdir -p $(OUT)
	@./$(BIN) $(PLUS) $(ARGS) 2>&1 | tee $(OUT)/sim.log; test $${PIPESTATUS[0]} -eq 0
	@if [ "$(WAVES)" = 1 ]; then cat $(FST).tmp > $(FST) && rm -f $(FST).tmp; fi
	@echo "log:  $(OUT)/sim.log"
	@if [ "$(WAVES)" = 1 ]; then echo "wave: $(FST)  (make wave TB=$(TB) $(if $(filter random,$(TB)),SEED=$(SEED)) $(if $(G),G='$(G)'))"; fi

wave:
	@test -f $(FST) || { echo "No $(FST). Run 'make run' with the same options first."; exit 1; }
ifeq ($(VIEWER),surfer)
	$(SURFER) $(FST) -c waves/$(TB).sucl >/dev/null 2>&1 &
else
	$(GTKWAVE) $(FST) $(GTKW) support/gtkwaverc >/dev/null 2>&1 &
endif

sim: run wave

lint:
	@mkdir -p build/lint
	$(VERILATOR) --lint-only --timing -sv --top-module $(TOP) -Wall -Wno-fatal -Wno-IMPURE \
	  --Mdir build/lint -f filelists/$(TB).f

regress:
	@test $(TB) = random || { echo 'regress needs TB=random: only that testbench has a SEED'; exit 1; }
	@for s in $(SEEDS); do \
	  $(MAKE) --no-print-directory run TB=$(TB) SEED=$$s WAVES=0 G='$(G)' > /dev/null 2>&1; \
	  echo "SEED=$$s: rc=$$? $$(grep -a 'HN-F REGRESSION' out/$(TB)/$$(echo SEED=$$s $(G) | xargs | tr ' ' '_')/sim.log | tail -1)"; \
	done

# One regress per configuration. "" is the default build (every feature on).
STRESS_CFGS ?= "" "NUM_TRACKERS=2" "NUM_TRACKERS=8" "MID_RESETS=16 RANDOM_RESETS=1" \
               "NUM_ROUNDS=40" "LOCK_ALL=1" "SNF_EARLY_COMP=0"

stress:
	@for g in $(STRESS_CFGS); do \
	  echo "== G='$$g'"; \
	  $(MAKE) --no-print-directory regress TB=random SEEDS='$(SEEDS)' G="$$g"; \
	done

clean:
	rm -rf build out

# ---- two-PC workflow ----------------------------------------------------------
sync:
	@bash ../tools/sync.sh $(if $(M),"$(M)")

pull:
	@bash ../tools/sync.sh --pull-only

doctor:
	@echo "host:           $(HOST)"
	@echo "VERILATOR_ROOT: $(VERILATOR_ROOT)"
	@echo "verilator:      $$(command -v $(VERILATOR) || echo MISSING: $(VERILATOR))"
	@$(VERILATOR) --version 2>/dev/null || true
	@echo "g++:            $$(command -v g++ || echo MISSING)"
	@echo "surfer:         $$(command -v $(SURFER) || echo 'not found (set SURFER in local.mk)')"
	@echo "gtkwave:        $$(command -v $(GTKWAVE) || echo 'not found (set GTKWAVE in local.mk)')"
	@echo "viewer:         $(VIEWER)"
	@echo "local.mk:       $(if $(wildcard local.mk),used,none)"
	@echo "git branch:     $$(git rev-parse --abbrev-ref HEAD) ($$(git status --porcelain -- ../RTL ../TB . | wc -l) changed files)"
