# IEEE 802.11abg MAC Packet Drop Statistics

## Overview

The IEEE 802.11abg MAC model publishes two statistic tables that count packets dropped for specific reasons:

- `UnicastPacketDropTable0` … `UnicastPacketDropTable3` (one per access category / WMM queue)
- `BroadcastPacketDropTable0` … `BroadcastPacketDropTable3`

These tables are implemented by `EMANE::Utils::CommonLayerStatistics` (`src/libemane/commonlayerstatistics.cc`).
Each row is keyed by the **source NEM** of the packet. The remaining columns are integer counters that increment each time a packet from that source is dropped for the corresponding reason.

**Important:** The column labeled **Dst MAC** is **not** a destination MAC address. It is a counter of packets dropped because the destination NEM in the MAC header did not match the receiving NEM (and promiscuous mode was disabled).

## Table Columns

| Column              | Drop Code | Meaning                                                                 | Source |
|---------------------|-----------|-------------------------------------------------------------------------|--------|
| NEM                 | —         | Source NEM identifier of the packets counted in this row               | CommonLayerStatistics key |
| SINR                | 1         | Packet dropped because the received SINR was below the PCR threshold   | `maclayer.cc` PCR check |
| Reg Id              | 2         | MAC registration ID in the common header did not match the expected IEEE 802.11abg registration ID | `processUpstreamPacket` |
| **Dst MAC**         | 3         | Destination NEM in the MAC header ≠ this NEM and promiscuous mode is off | `processUpstreamPacket` (after successful reception) |
| Queue Overflow      | 4         | Downstream queue for the access category was full; packet discarded    | `processDownstreamPacket` |
| Bad Control         | 5         | Required PHY control messages (ReceiveProperties / Frequency) missing or empty | `processUpstreamPacket` |
| Bad Spectrum Query  | 6         | Spectrum service query failed while evaluating reception               | `processUpstreamPacket` |
| Flow Control        | 7         | Flow-control token unavailable when attempting to enqueue downstream   | `processDownstreamPacket` |
| Duplicate           | 8         | Duplicate sequence number detected for this source                     | duplicate detection logic |
| Rx During Tx        | 9         | Packet arrived while this NEM was transmitting                         | `checkRxDuringTx` |
| Hidden Busy         | 10        | Hidden-node / busy-channel condition prevented reception               | `checkHiddenBusy` |

All values except the `NEM` key are **packet counts**.

## Interpreting the Example from Issue #254

```
nem 9   mac UnicastPacketDropTable0
| NEM | SINR | Reg Id | Dst MAC | Queue Overflow | Bad Control | Bad Spectrum Query | Flow Control | Duplicate | Rx During Tx | Hidden Busy |
| 5   | 0    | 0      | 4929    | 0              | 0           | 0                  | 0            | 359       | 0            | 0           |
| 6   | 0    | 0      | 1532    | 0              | 0           | 0                  | 0            | 1778      | 0            | 0           |
| 10  | 0    | 0      | 2304    | 0              | 0           | 0                  | 0            | 1484      | 0            | 0           |
| 14  | 3    | 0      | 3       | 0              | 0           | 0                  | 0            | 0         | 0            | 2           |
```

- The table is published by **NEM 9**.
- Rows for source NEMs 5, 6, 10 and 14.
- **4929, 1532, 2304** are the number of packets from those sources that were dropped because they were not addressed to NEM 9 (destination filtering). They are **not** MAC addresses.
- High “Dst MAC” counts are normal when many nodes are transmitting unicast traffic destined for other stations; they do **not** indicate poor RF quality.
- “Duplicate” counts reflect the model’s sequence-number duplicate detection.
- SINR drops (only 3 packets from NEM 14) are a different category from destination filtering.

## Obtaining the Table

Using the EMANE shell (`emanesh`):

```
show 9 mac UnicastPacketDropTable0
```

or via the Control Port API query for statistic tables.

Tables are clearable (`StatisticProperties::CLEARABLE`).

## Notes

- There is a separate table for each access category (0–3).
- Broadcast tables use the same column labels.
- These counters are model-specific to the IEEE 802.11abg MAC. Other MAC models (TDMA, RF Pipe, BentPipe, etc.) publish different drop-reason tables.
- A high drop count in one column does not by itself imply a link quality problem; total transmitted/received packet counts (available in the Accept tables and numeric statistics) are required to compute loss rates.

## Source References

- `src/models/mac/ieee80211abg/maclayer.cc` — drop code definitions and usage
- `src/libemane/commonlayerstatistics.cc` — table registration and `updateDropTable()`
