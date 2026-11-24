// license:BSD-3-Clause
// copyright-holders:Angelo Salese

#ifndef MAME_COMMODORE_C900_KBD_H
#define MAME_COMMODORE_C900_KBD_H

#pragma once

#include "machine/keyboard.h"

class c900_kbd_device : public device_t
						, protected device_matrix_keyboard_interface<13>
{
public:
	c900_kbd_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	// PA7: high while a scancode is presented on PA0-PA6/PC2
	auto strobe_callback() { return m_strobe_cb.bind(); }

	u8 pa_r();                  // CIO #1 port A: scancode in bits 0-6, strobe in bit 7
	u8 pc_r();                  // CIO #1 port C: bit 2 set for a key release
	void kbdclr_w(int state);   // CIO #1 PC3 (KBDCLR): low clears the latch

	u8 read_latch(offs_t offset);
	void write_latch(offs_t offset, u8 data);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	virtual void key_make(uint8_t row, uint8_t column) override;
	virtual void key_break(uint8_t row, uint8_t column) override;
	virtual void key_repeat(uint8_t row, uint8_t column) override;

private:
	static constexpr unsigned FIFO_SIZE = 16;

	uint8_t translate(uint8_t row, uint8_t column) const;
	void key_event(uint8_t code);
	void key_make_inner(uint8_t row, uint8_t column);
	void key_break_inner(uint8_t row, uint8_t column);
	void update_strobe();

	devcb_write_line m_strobe_cb;

	uint8_t m_fifo[FIFO_SIZE];  // scancodes, bit 7 set for a release
	uint8_t m_fifo_head;
	uint8_t m_fifo_count;
	bool m_strobe;
	bool m_kbdclr;
	bool m_enabled;
};

DECLARE_DEVICE_TYPE(C900KBD, c900_kbd_device)

#endif // MAME_COMMODORE_C900_KBD_H
