# HN-I: CHI home node for I/O (CHI -> AXI4)

`RTL/hni.sv`: home node for device / I/O space. Requests come in on one CHI
link (requesters told apart by SrcID, as behind an XP); the accesses go out on
an AXI4 master port with 32-bit data.

| CHI | AXI4 | Back to the requester |
|---|---|---|
| ReadNoSnp, 1..64 B | AR: <=4 B one narrow beat, >4 B INCR of size/4 words (up to 16 beats = 64 B) | CompData, Resp I, 1..4 beats |
| WriteNoSnpPtl / Full | DBIDResp first; after the write data: AW + W (WSTRB = byte enables) | Comp after the AXI B |

Design decisions:
* **Device order per requester:** a request goes to AXI only after every earlier
  request from the same SrcID has completed (CompData / Comp sent). Accesses with
  side effects (UART RX pop, PLIC claim) happen once and in program order;
  different requesters overlap.
* **AXI ID = {SrcID[SRC_IDW-1:0], tracker}:** unique per transaction in flight, so
  read data may come back interleaved / out of order across IDs; a slave can
  tell the requester from the upper ID bits (the SoC info page uses it for HART_ID).
* **No RetryAck:** a request is taken from the REQ FIFO only when a tracker is
  free, and REQ link credits are given only while the FIFO has room.
* **Errors:** SLVERR -> RespErr DERR, DECERR -> NDERR (precise access fault in the core).
* AxCACHE = 0 (Device non-bufferable), AxPROT = 0. ExpCompAck must be 0.
* `NUM_TRK` trackers (default 4); one AR and one AW/W burst issued at a time.

## Testbench

`TB/TB_hni.sv` (CHI/sim: `make regress TB=hni SEEDS='1 2'`):
* 4 sources (SrcID 1..4), 1500 random accesses each, up to 2 outstanding: reads,
  partial writes (some byte enables off), full writes, 1..64 B, each source in
  its own window (exact expected data), ~10% to SLVERR / DECERR windows.
* Random link credits on the HN-I's TX channels; AXI slave memory with random
  ready / latency, read beats of different IDs interleaved, B out of order.
* Checks: data, response fields, DataIDs, beat counts, RespErr, Comp after B,
  per-source completion order, link credits, AXI rules (INCR, size <= 4 B,
  aligned, no 4 KB crossing, WLAST, WSTRB lanes), idle at the end; coverage.

Status 2026-11-19: seeds 1 and 2 pass (all coverage points reached).

Also behind the XP in `TB=rnf` (`DEV=1`, default): the RTL RN-F's system
address map sends its device range (0xC000_0000, 512 MB) to the HN-I (home
port 1, node 0x50) with Device MemAttr; an AXI memory model behind it shares
the scoreboard's memory image, so every device read is checked and the final
memory check covers the device lines. Seed 10: pass (48 reads, 47 writes, 35
multi-beat bursts through the HN-I).
