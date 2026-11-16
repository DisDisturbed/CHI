// snoop_filter - inclusive, set-associative snoop filter for the HN-F
//
// One entry per cache line that some RN-F may hold:
//   valid, line address (the full line address is the tag), one presence bit per RN-F
//
// Precision rule: a presence bit may be set while the RN-F no longer holds the
// line (silent eviction): that only costs a snoop that gets SnpResp_I. A bit is
// never cleared while the RN-F may still hold the line, so every copy is
// always covered by an entry (inclusive).
//
// Who touches an entry
//   Trackers are ordered per line by the HN-F hazard logic, so at most one
//   tracker works on a line at a time. That tracker reads the entry once
//   (lookup), keeps the presence vector in its own registers while it snoops,
//   and writes the final vector back when it finishes (write port, ST_DONE).
//   An entry whose line is in use by a tracker is "busy" and is never chosen
//   as a victim, so nobody else changes it in between.
//
// Lookup port (one tracker per cycle, round-robin over the requesters)
//   hit            : entry found, its way and presence vector are returned
//   miss, no alloc : nobody holds the line, nothing is allocated
//   miss, alloc    : the requester will cache the line, so it needs an entry
//     - a free way          -> it is taken (presence 0)
//     - else a way that is not busy -> victim: the entry is taken over at once
//       and the victim's line address + presence are returned; the tracker
//       must back-invalidate that line (SnpCleanInvalid to every present RN-F)
//       before it goes on. The HN-F hazard logic holds every younger request
//       to the victim line until that is done.
//     - else (every way busy) -> no grant, the tracker asks again later
//
// Write port (any number of trackers per cycle: they own different entries)
//   the final presence vector; an all-zero vector frees the entry.
//
// The set index is an XOR fold of the line address, so lines that differ only
// in high address bits still spread over the sets.
module snoop_filter #(
  parameter int NUM_TRACKERS = 4,
  parameter int NUM_RNF      = 2,
  parameter int SETS         = 16,      // power of 2
  parameter int WAYS         = 4,
  parameter int AddrWidth    = 39,
  parameter int LineOffW     = 6,
  localparam int LAW         = AddrWidth - LineOffW,            // line address width
  localparam int IDXW        = (SETS <= 1) ? 1 : $clog2(SETS),
  localparam int WAYW        = (WAYS <= 1) ? 1 : $clog2(WAYS),
  localparam int TRKW        = (NUM_TRACKERS <= 1) ? 1 : $clog2(NUM_TRACKERS)
) (
  input  wire clk,
  input  wire resetn,

  // every tracker's line (for the busy check and the write-port index)
  input  logic                 trk_valid_i  [NUM_TRACKERS],
  input  logic [AddrWidth-1:0] trk_addr_i   [NUM_TRACKERS],

  // lookup
  input  logic                 lk_req_i     [NUM_TRACKERS],
  input  logic                 lk_alloc_i   [NUM_TRACKERS],  // allocate on a miss
  output logic                 lk_gnt_o     [NUM_TRACKERS],
  // result, valid in the cycle of lk_gnt_o (shared by all trackers)
  output logic                 lk_hit_o,
  output logic                 lk_entry_o,                   // the tracker owns an entry now (hit or allocated)
  output logic [WAYW-1:0]      lk_way_o,
  output logic [NUM_RNF-1:0]   lk_pres_o,
  output logic                 lk_victim_o,                  // back-invalidate lk_victim_addr_o first
  output logic [AddrWidth-1:0] lk_victim_addr_o,
  output logic [NUM_RNF-1:0]   lk_victim_pres_o,

  // write back the final presence vector
  input  logic                 wr_i         [NUM_TRACKERS],
  input  logic [WAYW-1:0]      wr_way_i     [NUM_TRACKERS],
  input  logic [NUM_RNF-1:0]   wr_pres_i    [NUM_TRACKERS]
);

  initial begin
    if (SETS < 1 || (SETS & (SETS - 1)) != 0)
      $fatal(1, "snoop_filter: SETS (%0d) must be a power of 2", SETS);
    if (WAYS < 1)
      $fatal(1, "snoop_filter: WAYS (%0d) must be at least 1", WAYS);
  end

  logic               v_q    [SETS][WAYS];
  logic [LAW-1:0]     tag_q  [SETS][WAYS];
  logic [NUM_RNF-1:0] pres_q [SETS][WAYS];
  logic [WAYW-1:0]    vic_ptr_q;              // where the victim search starts (rotates)
  logic [TRKW-1:0]    rr_q;                   // round-robin start for the lookup port

  function automatic logic [LAW-1:0] line_of(logic [AddrWidth-1:0] a);
    return a[AddrWidth-1:LineOffW];
  endfunction

  function automatic logic [IDXW-1:0] set_of(logic [AddrWidth-1:0] a);
    logic [LAW-1:0]  la;
    logic [IDXW-1:0] h;
    la = line_of(a);
    h  = '0;
    if (SETS > 1)
      for (int i = 0; i < LAW; i += IDXW)
        h ^= IDXW'(la >> i);
    return h;
  endfunction

  // ------------------------------------------------------------ lookup port
  logic             sel_v;
  logic [TRKW-1:0]  sel;
  logic [AddrWidth-1:0] sel_addr;
  logic [IDXW-1:0]  sel_set;
  logic             sel_alloc;
  logic             hit, free_v, vic_v, resolved;
  logic [WAYW-1:0]  hit_way, free_way, vic_way;
  logic             busy [WAYS];

  always_comb begin
    // pick the first requester at or after rr_q
    sel_v = 1'b0;
    sel   = '0;
    for (int k = 0; k < NUM_TRACKERS; k++) begin
      automatic int t = (int'(rr_q) + k) % NUM_TRACKERS;
      if (!sel_v && lk_req_i[t]) begin
        sel_v = 1'b1;
        sel   = TRKW'(t);
      end
    end
    sel_addr  = trk_addr_i[sel];
    sel_set   = set_of(sel_addr);
    sel_alloc = lk_alloc_i[sel];

    // a way is busy when any live tracker (or the requester) works on its line
    for (int w = 0; w < WAYS; w++) begin
      busy[w] = 1'b0;
      for (int t = 0; t < NUM_TRACKERS; t++)
        if (trk_valid_i[t] && v_q[sel_set][w] && line_of(trk_addr_i[t]) == tag_q[sel_set][w])
          busy[w] = 1'b1;
    end

    hit = 1'b0; hit_way = '0;
    free_v = 1'b0; free_way = '0;
    for (int w = 0; w < WAYS; w++) begin
      if (!hit && v_q[sel_set][w] && tag_q[sel_set][w] == line_of(sel_addr)) begin
        hit = 1'b1; hit_way = WAYW'(w);
      end
      if (!free_v && !v_q[sel_set][w]) begin
        free_v = 1'b1; free_way = WAYW'(w);
      end
    end

    vic_v = 1'b0; vic_way = '0;
    for (int k = 0; k < WAYS; k++) begin
      automatic int w = (int'(vic_ptr_q) + k) % WAYS;
      if (!vic_v && !busy[w]) begin
        vic_v = 1'b1; vic_way = WAYW'(w);
      end
    end

    resolved = sel_v && (hit || !sel_alloc || free_v || vic_v);

    for (int t = 0; t < NUM_TRACKERS; t++) lk_gnt_o[t] = resolved && (sel == TRKW'(t));

    lk_hit_o         = hit;
    lk_entry_o       = hit || (sel_alloc && (free_v || vic_v));
    lk_way_o         = hit ? hit_way : (free_v ? free_way : vic_way);
    lk_pres_o        = hit ? pres_q[sel_set][hit_way] : '0;
    lk_victim_o      = !hit && sel_alloc && !free_v && vic_v;
    lk_victim_addr_o = {tag_q[sel_set][vic_way], LineOffW'(0)};
    lk_victim_pres_o = pres_q[sel_set][vic_way];
  end

  // ------------------------------------------------------------ state
  always_ff @(posedge clk or negedge resetn) begin
    if (!resetn) begin
      for (int s = 0; s < SETS; s++)
        for (int w = 0; w < WAYS; w++) begin
          v_q[s][w]    <= 1'b0;
          tag_q[s][w]  <= '0;
          pres_q[s][w] <= '0;
        end
      vic_ptr_q <= '0;
      rr_q      <= '0;
    end else begin
      // the round-robin start moves every cycle a tracker asks, so one that
      // cannot be served (every way busy) does not block the others
      if (sel_v) rr_q <= (int'(sel) == NUM_TRACKERS - 1) ? '0 : sel + 1'b1;

      // write port: final presence vector of a finished tracker
      for (int t = 0; t < NUM_TRACKERS; t++) begin
        if (wr_i[t]) begin
          pres_q[set_of(trk_addr_i[t])][wr_way_i[t]] <= wr_pres_i[t];
          if (wr_pres_i[t] == '0) v_q[set_of(trk_addr_i[t])][wr_way_i[t]] <= 1'b0;
        end
      end

      // lookup port: a new entry (free way or victim taken over)
      if (resolved && !hit && sel_alloc) begin
        v_q[sel_set][lk_way_o]    <= 1'b1;
        tag_q[sel_set][lk_way_o]  <= line_of(sel_addr);
        pres_q[sel_set][lk_way_o] <= '0;
        if (lk_victim_o) vic_ptr_q <= (int'(vic_way) == WAYS - 1) ? '0 : vic_way + 1'b1;
      end
    end
  end

  // ------------------------------------------------------------ checks (sim)
  // synthesis translate_off
  always @(posedge clk) begin
    if (resetn) begin
      for (int a = 0; a < NUM_TRACKERS; a++) begin
        if (wr_i[a] && !v_q[set_of(trk_addr_i[a])][wr_way_i[a]])
          $error("[%0t] snoop_filter: tracker %0d writes an entry that is not valid", $time, a);
        if (wr_i[a] && v_q[set_of(trk_addr_i[a])][wr_way_i[a]] &&
            tag_q[set_of(trk_addr_i[a])][wr_way_i[a]] != line_of(trk_addr_i[a]))
          $error("[%0t] snoop_filter: tracker %0d writes an entry that belongs to another line", $time, a);
        if (resolved && !hit && sel_alloc && wr_i[a] && set_of(trk_addr_i[a]) == sel_set && wr_way_i[a] == lk_way_o)
          $error("[%0t] snoop_filter: tracker %0d writes the entry the lookup port is allocating", $time, a);
      end
      for (int s = 0; s < SETS; s++)
        for (int w1 = 0; w1 < WAYS; w1++)
          for (int w2 = w1 + 1; w2 < WAYS; w2++)
            if (v_q[s][w1] && v_q[s][w2] && tag_q[s][w1] == tag_q[s][w2])
              $error("[%0t] snoop_filter: set %0d holds line %0h twice (ways %0d and %0d)", $time, s, tag_q[s][w1], w1, w2);
    end
  end
  // synthesis translate_on

endmodule
