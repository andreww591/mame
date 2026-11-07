// license:MIT
// copyright-holders:Andrew Warkentin, Michal Pleban
/******************************************************************************************

Copyright (C) 2026 Michal Pleban
Copyright (C) 2026 Andrew Warkentin

Commodore C900 Physical DMA Controller, including disks

This is a port of the PDMAC implementation from Michal Pleban's minimal C900 emulator.

The disk-controller card carries both the WD2010 hard-disk MCU and the Commodore floppy
controller. They share one PDMAC doorbell (port 0x0500) and one vectored interrupt (vector
0x80), but each owns a 16-byte SASI-style command block: the hard disk at m_hdc_cmdblk
(default 0x080000) and the floppy immediately after it at m_hdc_cmdblk+0x10. A block is
"armed" when its status byte (offset 0x0C) holds 0xFF; the controller overwrites it with a
completion status and raises the interrupt. Both media address their data by linear block
number.

To Do:
- Floppy: commands currently complete with "drive not ready"

*******************************************************************************************/

#include "emu.h"
#include "c900_pdmac.h"

#define LOG_CMD (1U << 1)
//#define VERBOSE (LOG_CMD)
#include "logmacro.h"

namespace {

constexpr uint32_t SECTOR_SIZE = 512;
constexpr uint8_t VECTOR = 0x80;

constexpr uint8_t ST_OK_NO_DATA = 0x00;
constexpr uint8_t ST_OK = 0x80;
constexpr uint8_t ST_BAD_OPCODE = 0x81;
constexpr uint8_t ST_NOT_READY = 0x92;

} // anonymous namespace

DEFINE_DEVICE_TYPE(C900PDMAC, c900_pdmac_device, "c900_pdmac", "Commodore 900 PDMAC")

c900_pdmac_device::c900_pdmac_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, C900PDMAC, tag, owner, clock)
	, device_z80daisy_interface(mconfig, *this)
	, m_ireq_cb(*this)
	, m_drive(*this, "harddisk")
	, m_dma_space(*this, finder_base::DUMMY_TAG, -1)
	, m_hdc_cmdblk(0x080000)
	, m_intrq(0)
{
}

void c900_pdmac_device::device_add_mconfig(machine_config &config)
{
	HARDDISK(config, m_drive);
}

void c900_pdmac_device::device_start()
{
	save_item(NAME(m_hdc_cmdblk));
	save_item(NAME(m_intrq));
}

void c900_pdmac_device::device_reset()
{
	m_hdc_cmdblk = 0x080000;
	set_ireq(0);
}

void c900_pdmac_device::set_ireq(int state)
{
	m_intrq = state;
	m_ireq_cb(state);
}

int c900_pdmac_device::z80daisy_irq_state()
{
	return m_intrq ? Z80_DAISY_INT : 0;
}

int c900_pdmac_device::z80daisy_irq_ack()
{
	set_ireq(0);
	return VECTOR;
}

void c900_pdmac_device::z80daisy_irq_reti()
{
}

u16 c900_pdmac_device::read(offs_t offset)
{
	return 0;
}

// Doorbell: service whichever command block is armed. Both are always examined - the floppy
// controller is a fixed part of the card, so a command issued with no drive completes with a
// "not ready" status rather than leaving the driver waiting for an interrupt.
void c900_pdmac_device::write(offs_t offset, u16 data)
{
	if (data != 0){
		process_command(m_hdc_cmdblk, false);
		process_command(m_hdc_cmdblk + 0x10, true);
	}
}

void c900_pdmac_device::process_command(uint32_t cb, bool is_floppy)
{
	address_space &space = *m_dma_space;

	if (space.read_byte(cb + 0x0c) != 0xff)
		return; // not armed for this block

	uint8_t const opcode = space.read_byte(cb + 0x00);
	uint8_t const lunhi = space.read_byte(cb + 0x01);
	uint8_t const mid = space.read_byte(cb + 0x02);
	uint8_t const low = space.read_byte(cb + 0x03);
	uint8_t const bcnt = space.read_byte(cb + 0x04);
	uint32_t const dma = (uint32_t(space.read_byte(cb + 0x06)) << 16) | (uint32_t(space.read_byte(cb + 0x07)) << 8) | space.read_byte(cb + 0x08);
	uint32_t const lba = (uint32_t(lunhi & 0x1f) << 16) | (uint32_t(mid) << 8) | low;

	bool const ready = !is_floppy && m_drive->exists();

	uint32_t sectors = 0;
	if (ready){
		hard_disk_file::info const &info = m_drive->get_info();
		sectors = info.cylinders * info.heads * info.sectors;
	}

	LOGMASKED(LOG_CMD, "%s %s cmd %02x lba %u count %u dma %06x\n", machine().describe_context(), is_floppy ? "FD" : "HD", opcode, lba, bcnt, dma);

	uint8_t status;
	switch (opcode){
		case 0x00: /* TestDriveReady */
		case 0x01: /* Restore */
		case 0x03: /* RequestStatus */
		case 0x05: /* CheckTrackFormat */
		case 0x0c: /* SetDriveParams */
			status = ready ? ST_OK_NO_DATA : ST_NOT_READY;
			break;

		case 0x04: /* FormatDisk (floppy only) - no low-level format to model */
			status = ready ? ST_OK : ST_NOT_READY;
			break;

		case 0x0f: /* ChangeCmdBlockAddr - new base in the DMA fields (hard disk) */
			if (!is_floppy)
				m_hdc_cmdblk = dma;
			status = ST_OK_NO_DATA;
			break;

		case 0x08: /* Read */
			if (!ready || (lba + bcnt > sectors)){
				status = ST_NOT_READY;
				break;
			}
			for (uint32_t s = 0; s < bcnt; s++){
				uint8_t buf[SECTOR_SIZE];
				if (!m_drive->read(lba + s, buf))
					memset(buf, 0, SECTOR_SIZE); // unwritten blocks of a partial image read as zero
				for (uint32_t i = 0; i < SECTOR_SIZE; i++)
					space.write_byte(dma + s * SECTOR_SIZE + i, buf[i]);
			}
			status = ST_OK;
			break;

		case 0x0a: /* Write */
			if (!ready || (lba + bcnt > sectors)){
				status = ST_NOT_READY;
				break;
			}
			for (uint32_t s = 0; s < bcnt; s++){
				uint8_t buf[SECTOR_SIZE];
				for (uint32_t i = 0; i < SECTOR_SIZE; i++)
					buf[i] = space.read_byte(dma + s * SECTOR_SIZE + i);
				m_drive->write(lba + s, buf);
			}
			status = ST_OK;
			break;

		default:
			logerror("%s bad opcode %02x\n", machine().describe_context(), opcode);
			status = ST_BAD_OPCODE;
			break;
	}

	space.write_byte(cb + 0x0c, status); // completion sentinel
	space.write_byte(cb + 0x0d, lunhi);
	space.write_byte(cb + 0x0e, mid);

	// raise the PDMAC disk-completion vectored interrupt
	set_ireq(1);
}
