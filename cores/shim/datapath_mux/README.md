# Datapath Mux Core

The `datapath_mux` module is the per-board 2:1 select that lets the programmed-I/O (PIO) and DMA datapaths share one board's two high-rate FIFO ports -- the DAC command FIFO write side and the ADC data FIFO read side. The PIO path is the PS reaching a FIFO by direct CPU register access through its `axi_fifo_bridge` window (mapped into userspace with `mmap()`); the DMA path is the MCDMA engine. One instance sits per board, gated by that board's `datapath_mode` bit (0 = PIO, 1 = DMA; the bit comes from `axi_sys_ctrl`, resets to PIO, and is locked while the system runs).

## Why it exists

The DAC command FIFO has a single write-data port and the ADC data FIFO a single read port, but each now has two possible owners: the `axi_fifo_bridge` (PIO) and, on the DMA side, an `axis_fifo_bridge` (DAC command writes from MM2S) and the `adc_packetizer` (ADC data reads to S2MM). Two producers on one write port, or two consumers on one read port, would corrupt the FIFO if both were ever live, so exactly one must drive it at a time. The 32-bit write-data select has no stock IPI primitive, so the select lives in RTL; the same core also handles the 1-bit read-enable select and the handshake gating.

## The hard lock

The select is a hard lock, not a convention. The idle owner is gated on both its data path and its handshake, so it physically cannot touch the FIFO:

- In DMA mode the PIO side is shown `full = 1` and `empty = 1`, so the `axi_fifo_bridge`'s `wr_en` / `rd_en` fold to zero, and only the DMA side's `wr_en` / `rd_en` reach the FIFO.
- In PIO mode the DMA side is shown `full = 1` and `empty = 1` (and a zero read count), so the `axis_fifo_bridge` / `adc_packetizer` are held off, and only the PIO side reaches the FIFO.

A wrong-mode access can therefore drop data but can never corrupt the FIFO. Reporting a wrong-mode access gracefully (the `mode_viol` fault into `hw_manager`) is folded in with the DMA interrupt work in a later stage; this core is only responsible for the lock.

## Parameters

- `DATA_WIDTH` (integer): FIFO/data word width (default: `32`).
- `COUNT_WIDTH` (integer): ADC data FIFO read-side count width, the FIFO's `ADDR_WIDTH + 1` (default: `14`).

## Ports

- `datapath_mode` (input): 0 selects the PIO owner, 1 selects the DMA owner.

### DAC command FIFO write port

- `cmd_wr_data`, `cmd_wr_en` (output): to the FIFO write port.
- `cmd_full` (input): the FIFO's full flag.
- `cmd_pio_wr_data`, `cmd_pio_wr_en` (input) / `cmd_pio_full` (output): the PIO owner side.
- `cmd_dma_wr_data`, `cmd_dma_wr_en` (input) / `cmd_dma_full` (output): the DMA owner side.

### ADC data FIFO read port

- `data_rd_data`, `data_empty`, `data_count` (input) / `data_rd_en` (output): the FIFO read port.
- `data_pio_rd_data`, `data_pio_empty` (output) / `data_pio_rd_en` (input): the PIO owner side.
- `data_dma_rd_data`, `data_dma_empty`, `data_dma_count` (output) / `data_dma_rd_en` (input): the DMA owner side.
