// license:MIT
// copyright-holders:Andrew Warkentin, Michal Pleban
/******************************************************************************************

Commodore C900 Physical DMA Controller, including disks

This is a port of the PDMAC implementation from Michal Pleban's minimal C900 emulator.

*******************************************************************************************/

#ifndef MAME_COMMODORE_C900_PDMAC_H
#define MAME_COMMODORE_C900_PDMAC_H

#pragma once

#include "imagedev/harddriv.h"
#include "machine/z80daisy.h"

class c900_pdmac_device : public device_t, public device_z80daisy_interface
{
public:
	c900_pdmac_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	auto ireq_callback() { return m_ireq_cb.bind(); }
	template <typename T> void set_dma_space(T &&tag, int spacenum) { m_dma_space.set_tag(std::forward<T>(tag), spacenum); }

	u16 read(offs_t offset);
	void write(offs_t offset, u16 data);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	// device_z80daisy_interface implementation
	virtual int z80daisy_irq_state() override;
	virtual int z80daisy_irq_ack() override;
	virtual void z80daisy_irq_reti() override;

private:
	void process_command(uint32_t cb, bool is_floppy);
	void set_ireq(int state);

	devcb_write_line m_ireq_cb;
	required_device<harddisk_image_device> m_drive;
	required_address_space m_dma_space;
	uint32_t m_hdc_cmdblk;
	int m_intrq;
};

DECLARE_DEVICE_TYPE(C900PDMAC, c900_pdmac_device)

#endif // MAME_COMMODORE_C900_PDMAC_H
