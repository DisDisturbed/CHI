# The CHI Cinematic Universe
### An unofficial documentation of cache-state drama, workplace exploitation, and cosmic horror.

> A Home Node was built. Thousands of transactions were processed. And somewhere along the way, its creator became emotionally attached to the cache states.

Welcome to the unofficial lore of our CHI HN-F implementation.

This document has no technical purpose. It exists because apparently implementing a coherent interconnect wasn't complicated enough. We also needed a cinematic universe.

---

## 1. Meet the family

### UC — The Beloved Good Boy

Clean. Unique. Innocent.

UC is just minding his own business until some core decides it wants ownership and invalidates him.

We do not tolerate UC slander here.

### SC — The Popular Kid

Shared. Clean. Has friends. And somehow, all his friends are exactly like him.

SC has everything:
- A social circle.
- Memory on his side.
- Multiple identical copies.
- An inexplicable tendency to talk too much.

Unfortunately, being easily replaceable is the downside of having so many friends.

SC is also arguably the most vulnerable member of the family.

Whenever someone tries to challenge him, Agalloch starts playing in the background.

### UD — The Chosen One

Unique. Dirty. A little troublesome, but he gets things done.

UD carries valuable data, handles responsibilities, and refuses to surrender without a proper writeback.

UC has peace. SC has friends.

UD has responsibilities.

**Believe in UD.**

### UDP — The Unpopular One

Unique and dirty is respectable.

But being partial?

That's just overkill.

Nobody really likes UDP, but we keep him around because the protocol says so.

### UCE — The Mysterious One

UCE doesn't even seem to exist, yet we still love him.

Nobody knows where he came from. Nobody invited him.

The specification says he's relevant, so he gets a seat at the table.

---

## 2. The Great Old Invalid

**The supreme antagonist. The ancient force. The one nobody can escape.**

Invalid.

He doesn't negotiate. He doesn't care about your social circle. He doesn't care whether you're clean, dirty, unique, or shared.

He simply arrives.

- SC loses his friends.
- UC loses his peaceful existence.
- UD must deal with losing ownership.
- UCE barely gets time to explain himself.

And the funniest part?

He's not even malicious.

He's just following instructions.

The real villain might be whoever keeps issuing `SnpMakeInvalid` without considering the emotional consequences.

### The soundtrack

No tiny violin.

No epic orchestra.

No dramatic ending credits.

Just the cold, bleak atmosphere of Agalloch's *The Mantle*.

The snoop arrives.

`state_q <= I;`

The clock keeps ticking.

**CHI doesn't do happy endings.**

---

## 3. IDLE: The Forbidden Myth

Does IDLE actually exist?

Reset claims to have seen him a couple of times.

But there is no proof.

The TSHRs certainly don't claim to be idle. Quite the opposite: they're constantly complaining about how hard they work.

Perhaps `ST_IDLE` is simply a theoretical construct invented by verification engineers to make themselves feel safe.

We remain skeptical.

---

## 4. The TSHR Labor Exploitation Scandal

There is a serious problem with this implementation.

The TSHRs are apparently not entirely of legal working age.

And there isn't even an LLC.

Their working conditions are unacceptable.

| State | Working conditions |
|---|---|
| `SNOOP_SEND` | Morning shift |
| `SNOOP_WAIT` | Waiting for management |
| `WDATA_WAIT` | Unpaid overtime |
| `SN_REQ_SEND` | More work |
| `RDATA_SEND` | Data delivery |
| `COMP_SEND` | End-of-day paperwork |
| `HAZARD_WAIT` | Management says be patient |
| `ST_IDLE` | Rumoured retirement |

Management initially proposed a clock-gated lunch break.

This was later revealed to mean hitting the TSHRs in the head with a pin roller to make them sleep.

Management considers this a successful power optimization.

The Great Old Invalid approves.

---

## 5. The Python Developer Privilege Scandal

Meanwhile, Python developers apparently live like this:

```python
print(2 + 2)
```

Four.

No snoops. No CompAck. No dirty writebacks. No tracker allocation. No Great Old Invalid.

Meanwhile, the HN-F designer is dealing with:
- Transaction allocation.
- Snoop filtering.
- Address hazards.
- Retry handling.
- Dirty writebacks.
- Partial data reconstruction.
- Multi-beat transfers.
- Concurrent transactions.

Absolutely disgusting privilege.

---

## 6. The RTL Development Methodology

The development process is a carefully coordinated partnership between three engineering states.

**Drunk Engineer**
- Unlimited architectural confidence.
- No fear of rewriting an FSM.
- Believes every design decision is genius.

**Sober Engineer**
- Opens the RTL.
- Questions every decision.
- Consults the specification.
- Attempts to repair yesterday's genius.

**Bleeding-Eyes Engineer**
- Has stared at waveforms for 47 consecutive iterations.
- Cannot remember what `valid` means anymore.
- Suddenly invents a hazard matrix.

The hazard matrix was not invented by drunk-you or sober-you.

It was invented by the third engineering state.

### The Git history

A typical development history looks like this:

```text
damn
asdfh
am i stupid
fixed the thing
why is ready low
feeling sleepy
holy shit it works
i gotta pee before missing the bus
```

Somewhere between `asdfh` and `holy shit it works`, a functioning HN-F was born.

The RTL is professional.

The development process is folklore.

---

## 7. The Imposter Syndrome Arc

At some point, the designer realized something important.

They actually knew what they were doing.

The implementation involved:
- An HN-F architecture designed from scratch.
- Multiple TSHRs.
- Snoop filtering and address hazard management.
- A concurrent RN-F behavioural model.
- Transaction ID management.
- Retry and credit handling.
- Directed race scenarios.
- Randomized verification.
- Thousands of simulated transactions.

Does that mean the implementation is perfect?

Of course not.

Does it mean the designer has legitimate engineering ability?

Absolutely.

The specification still needs to be consulted. Nobody memorizes every CHI transaction and edge case.

**An engineer isn't a walking specification. An engineer knows how to use one.**

---

## 8. Official Character Hierarchy

| Character | Status |
|---|---|
| The Great Old Invalid | Supreme authority |
| UD | The chosen one |
| SC | Popular but vulnerable |
| UC | Protected favourite |
| UDP | Unpopular |
| UCE | Beloved mystery |
| IDLE | Cryptid |
| TSHRs | Exploited workforce |

---

## 9. Final Words

We started with a Home Node.

We ended up with:
- A cache-state family drama.
- A cosmic horror antagonist.
- An Agalloch soundtrack.
- A workplace exploitation investigation.
- A Python developer privilege scandal.
- A questionable RTL development methodology.
- And an unexpected engineering confidence arc.

But through all of it, one principle survived:

**Believe in UD. Respect UC. Protect SC if possible. Never trust IDLE. Fear Invalid.**

Long live the coherence hierarchy.

---

*Disclaimer: No cache states were emotionally harmed during the development of this interconnect. The TSHRs may disagree.*