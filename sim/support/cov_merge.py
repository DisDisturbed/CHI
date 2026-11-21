#!/usr/bin/env python3
"""Merge the coverage of many seeds and list the bins nothing reached.

    cov_merge.py <out dir> [--json out.json]

<out dir> is sim/out (or a copy): every out/<tb>/SEED=<n>/sim.log of TB=rnf,
random and hni is read. A testbench prints its coverage as one line per bin
that was reached ("  <name>  <count>"), so a bin that was never reached is
simply missing from every log. The universe of bins therefore comes from the
testbench sources:

  * every string literal passed to cov_hit() / hit() / need() (also both arms
    of a ?: of literals);
  * every parametrized bin ($sformatf / {..} concatenations) expanded over
    the domain of its arguments (the enums and helper functions they come
    from: see PATTERNS below).

A source file's literal bins count for a testbench only if that testbench
reached at least one bin of the file (so a model a testbench does not
instantiate does not add false zeros).
"""
import itertools
import json
import os
import re
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
CHI = os.path.abspath(os.path.join(HERE, "..", ".."))

SOURCES = {
    "rnf":    ["common/TB/chi_tb_pkg.sv", "common/TB/rnf_model.sv", "common/TB/snf_model.sv",
               "RN-F/TB/rnf_core_driver.sv", "RN-F/TB/rnf_fetch_driver.sv", "RN-F/TB/rnf_chi_checker.sv",
               "RN-F/TB/TB_rnf.sv"],
    "random": ["common/TB/chi_tb_pkg.sv", "common/TB/rnf_model.sv", "common/TB/snf_model.sv",
               "HN-F/TB/Randomized/TB_hnf_random.sv", "HN-F/TB/Randomized/tshr_cov.sv"],
    "hni":    ["HN-I/TB/TB_hni.sv"],
}

# ---- domains (from the RTL / TB enums and helper functions) -------------------
CS = ["CS_I", "CS_SC", "CS_UC", "CS_UD", "CS_SD", "CS_UCE", "CS_UDP"]          # chi_pkg::cache_state_e
SNP = ["SNP_SHARED", "SNP_CLEAN", "SNP_ONCE", "SNP_NOT_SHARED_DIRTY", "SNP_UNIQUE", "SNP_CLEAN_SHARED",
       "SNP_CLEAN_INVALID", "SNP_MAKE_INVALID", "SNP_DVM_OP", "SNP_FWD_SHARED", "SNP_FWD_CLEAN",
       "SNP_FWD_ONCE", "SNP_FWD_NOT_SHARED_DIRTY", "SNP_FWD_UNIQUE"]            # chi_pkg::snp_opcode_e
RSTATE = ["R_LOOKUP", "R_LOAD", "R_TAG_RD", "R_REQ", "R_WAIT", "R_RETRY", "R_FILL_TAG", "R_FILL_SEL",
          "R_FILL", "R_FILL_DONE", "R_WB_TAG", "R_WB_CMP", "R_WB_RD", "R_WDAT", "R_WCOMP", "R_ACK",
          "R_RESP", "R_DEP"]                                                    # rnf_dcache rstate_e (not R_IDLE)
TXN = ["T_READ", "T_EVICT", "T_WB", "T_NCRD", "T_NCWR"]                         # rnf_dcache txn_e
OP = ["load", "store"]
NB = ["0", "1", "2-3", "4+"]                                                    # rnf_core_driver nb()
LST = ["not held", "Shared Clean", "Unique", "Shared Dirty"]                    # rnf_chi_checker lst_str()
START = ["not held", "Shared", "Unique Dirty"]                                  # rnf_core_driver s_start()
TSHR = ["ST_IDLE", "ST_ALLOC", "ST_SNOOP_SEND", "ST_SNOOP_WAIT", "ST_DBIDRESP_SEND", "ST_WDATA_WAIT",
        "ST_SN_REQ_SEND", "ST_SN_RSP_WAIT", "ST_SN_DATA_SEND", "ST_RDATA_SEND", "ST_COMP_SEND",
        "ST_COMPACK_WAIT", "ST_DONE", "ST_READNO_SNOOP", "ST_DIRTY_WB_REQ_SEND", "ST_DIRTY_WB_RSP_WAIT",
        "ST_DIRTY_WB_DATA_SEND", "ST_SN_COMP_WAIT", "ST_SN_RETRY_WAIT", "ST_HAZARD_WAIT", "ST_SF_LOOKUP",
        "ST_SNOOP_PICK", "ST_SNOOP_PAR"]                                         # HN-F TSHR state_e
HNI_SZ = ["1", "2", "4", "8", "16", "32", "64"]
HNI_ERR = ["", " SLVERR", " DECERR"]


def cross(fmt, *doms):
    return [fmt.format(*c) for c in itertools.product(*doms)]


req_states = [f"{r} {t}" for r in ("R_REQ", "R_WAIT", "R_WDAT", "R_WCOMP") for t in TXN] + \
             [r for r in RSTATE if r not in ("R_REQ", "R_WAIT", "R_WDAT", "R_WCOMP")]

PATTERNS = {
    "rnf": (
        cross("core: {} {}B {}", ["nc", "cached"], ["1", "2", "4", "8"], OP)
        + cross("core: rsp.err on {} {}", ["nc", "cached"], OP)
        + cross("core: no rsp.err on an error line ({})", OP)
        + cross("core: cached {} answered with chr_state {}", OP, CS)
        + cross("core: request accepted with {} other request(s) outstanding", NB)
        + cross("drv: request presented with {} other request(s) in flight", NB)
        + cross("merge: sub-word stores onto a line that starts {}", START)
        + cross("bb core: {} completed after a line read (miss)", OP)
        + cross("bb core: {} completed without a line read (hit)", OP)
        + cross("bb core: {} whose line was snooped while it was open", OP)
        + cross("bb core: {} on a line {}", OP, LST)
        + cross("bb core: sub-word store (<8B) on a line {}", LST)
        + cross("dc snoop: {} on {}", SNP, CS)
        + cross("dc snoop while the request FSM is in {}", req_states)
        + cross("dc core: {} hit on {}", OP, CS)
        + cross("dc core: {} miss, free way", OP)
        + cross("dc core: miss replaces a {} line", CS)
        + cross("dc write-back: line sent as {}", CS)
    ),
    "random": cross("tshr: enter {}", TSHR),
    "hni": (cross("op: ReadNoSnp {} B{}", HNI_SZ, HNI_ERR) + cross("op: WriteNoSnpPtl {} B{}", HNI_SZ, HNI_ERR)
            + cross("op: WriteNoSnpFull {} B{}", ["64"], HNI_ERR)),
}

CALL = re.compile(r"\b(?:cov_hit|hit|need)\s*\(", re.S)


def literal_bins(path):
    """string literals given to cov_hit/hit/need, skipping formatted ones"""
    text = open(path, errors="replace").read()
    text = re.sub(r"//[^\n]*", "", text)
    out = set()
    for m in CALL.finditer(text):
        depth, i = 1, m.end()
        while i < len(text) and depth:
            depth += {"(": 1, ")": -1}.get(text[i], 0)
            i += 1
        arg = text[m.end():i - 1]
        if "sformatf" in arg or "{" in arg:
            continue
        for s in re.findall(r'"([^"]*)"', arg):
            if s and "%" not in s and ":" in s:
                out.add(s)
    # need [$] / string arrays of required bins (TB_hni)
    for m in re.finditer(r"string\s+need\s*\[\$\]\s*=\s*'\{(.*?)\};", text, re.S):
        out.update(re.findall(r'"([^"]+)"', m.group(1)))
    return out


def read_cov(log, tb):
    lines = open(log, errors="replace").read().replace("\x1b", "").splitlines()
    out = {}
    start = None
    for i, l in enumerate(lines):
        if (tb != "hni" and l.strip() == "---- coverage ----") or (tb == "hni" and l.startswith("---- HN-I TB")):
            start = i + 1
    if start is None:
        return None
    for l in lines[start:]:
        if not l.startswith("  ") or l.startswith("  ----"):
            break
        m = re.match(r"\s+(.*\S)\s+(\d+)$", l)
        if m:
            out[m.group(1)] = int(m.group(2))
    return out


def main():
    od = sys.argv[1]
    res = {}
    for tb, srcs in SOURCES.items():
        hits = defaultdict(int)
        seeds_hit = defaultdict(int)
        seeds = []
        base = os.path.join(od, tb)
        if not os.path.isdir(base):
            continue
        for d in sorted(os.listdir(base), key=lambda s: int(re.sub(r"\D", "", s) or 0)):
            m = re.fullmatch(r"SEED=(\d+)", d)
            if not m:
                continue
            cov = read_cov(os.path.join(base, d, "sim.log"), tb)
            if cov is None:
                continue
            seeds.append(int(m.group(1)))
            for k, v in cov.items():
                hits[k] += v
                seeds_hit[k] += 1
        universe = set(PATTERNS.get(tb, []))
        per_file = {}
        for s in srcs:
            lits = literal_bins(os.path.join(CHI, s))
            per_file[s] = lits
            if any(b in hits for b in lits):
                universe |= lits
        universe |= set(hits)
        zero = sorted(b for b in universe if hits.get(b, 0) == 0)
        rare = sorted((b for b in hits if 0 < seeds_hit[b] < len(seeds)), key=lambda b: seeds_hit[b])
        res[tb] = {"seeds": seeds, "bins": len(universe), "hit": len(universe) - len(zero),
                   "zero": zero, "counts": dict(hits),
                   "rare": [(b, seeds_hit[b], hits[b]) for b in rare]}
        print(f"== TB={tb}: {len(seeds)} seeds, {len(universe)} bins, {len(universe) - len(zero)} reached, {len(zero)} never")
        for b in zero:
            print(f"   0  {b}")
    if "--json" in sys.argv:
        json.dump(res, open(sys.argv[sys.argv.index("--json") + 1], "w"), indent=1)


if __name__ == "__main__":
    main()
