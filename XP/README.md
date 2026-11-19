# XP: crosspoint between the RN-Fs and the home nodes

`RTL/xp.sv`: one CHI link per RN-F and per home node (HN-F, HN-I); flits are
routed by TgtID. The SN-F keeps its direct link to the HN-F.

```
 RN-F 0 ──┐            ┌── HN-F (0x40) ── SN-F (0x00, direct)
 RN-F 1 ──┤    XP      │
   ...  ──┤ (TgtID)    └── HN-I (0x50) ── AXI4
 RN-F n ──┘
```

| Direction | Channel | Sender queue | Routed by |
|---|---|---|---|
| RN-F → home | REQ, RSP (CompAck, SnpResp), DAT (write data, SnpRespData) | per RN-F | TgtID vs `HN_NID` |
| home → RN-F | SNP, RSP (Comp, DBIDResp, ...), DAT (CompData) | per home | TgtID vs `RN_NID` |

Design decisions:
* **Input queue per sender and channel** (`REQ_DEPTH`, `RSP_DEPTH`, `SNP_DEPTH` = 2,
  `DAT_DEPTH` = 4). The XP hands the sender one link credit per free entry, so it
  never refuses a flit.
  4 DAT entries cover the credit round trip, so a 64 B line (4 beats) streams
  at one beat per cycle; 2 entries give 2 flits per 3 cycles on a busy link.
* **Zero-cycle pass-through** (`BYPASS` = 1, default): a flit that arrives at an
  empty queue is offered to its output in the same cycle and leaves the XP in
  that cycle when it wins the output (with a credit); otherwise it is queued
  and leaves one cycle later at the earliest. The price is a combinational
  path sender -> route compare -> output arbiter -> receiver. `BYPASS` = 0
  queues every flit (one cycle per hop, no path through the XP).
* **Output port per receiver:** link credits from the receiver (up to 15),
  round-robin over the senders that offer it a flit (queue head, or the
  passing flit), one flit per cycle. Flits of one sender to one receiver stay
  in order (a flit only passes an empty queue); different senders interleave.
* **Snoops:** one per cycle on the HN-F's SNP link; each carries TgtID (the
  snooped RN-F), added to the SNP flit for this.
* **HN-I ports** (`HN_COH[h] = 0`): no SNP out of it, no RSP into it.
* **No deadlock from head-of-line blocking:** RSP / DAT / SNP receivers sink
  without waiting on another channel, and a REQ waits only for a home tracker,
  which frees without any REQ moving.
* A flit whose TgtID matches no port is dropped (its credit comes back) and
  reported in simulation; so is a flit sent without a credit.

The HN-F on the XP (`HN-F/RTL/HN-F.sv`) has one link per channel and tells
RN-Fs apart by node ID (`RNF_NID[i]` = node ID of RN-F index i, the snoop
filter presence bit i): the requester's SrcID gives its index, snoop responses
are sorted by SrcID, snoops carry `TgtID = RNF_NID[target]`. Its REQ credits
on the XP link are one per free tracker that no outstanding credit already
holds, so every request that arrives gets a tracker; the XP shares them out
round-robin between the RN-Fs.

Testbenches on the XP: `TB=random` (Tb_HNF_Random: rnf_models), `TB=rnf`
(Tb_RNF: RTL RN-F + rnf_model), `TB=directed` (Tb_HNF). The random testbench
checks every link's credits on both sides (RN-F ↔ XP, XP ↔ HN-F) and that the
XP queues are empty at the end.

Status 2026-11-19 (through the XP): `TB=random` seeds 1, 2 and seed 1 with
`NUM_TRACKERS=2` (4 RN-Fs on 2 trackers) pass; `TB=directed` passes;
`TB=rnf` seed 10 passes. `TB=rnf` seed 1 misses one coverage point
("dc snoop: SNP_SHARED on CS_I"), as seeds 1 and 2 already did before the XP;
with the extra hop that race is rarer (seed 10: 1 hit, 37 before).
`TB=rnf` with the HN-I as a second home (device range): seed 10 passes.
