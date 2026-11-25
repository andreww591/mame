// license:BSD-3-Clause
// copyright-holders:Curt Coder, Robbbert
/******************************************************************************************

Commodore C900
UNIX prototype

http://www.zimmers.net/cbmpics/c900.html
http://www.zimmers.net/cbmpics/cbm/900/c900-chips.txt

Chips: Z8001 CPU, Z8010 MMU, Z8030 SCC, 2x Z8036 CIO (U78 at I/O 0x00, U66 at I/O 0x80). Crystal: 12MHz

The Z8030 runs 2 serial ports. The Z8036 runs the IEEE interface and the speaker.

The FDC is an intelligent device that communicates with the main board via the MMU.
It has a 6508 CPU.

Disk drive is a Matsushita JA-560-012

Increasing the amount of RAM stops the error message, however it still keeps running
into the weeds (jumps to 00000). Due to lack of banking, the stack is pointing at rom.

To Do:
- Banking
- Pretty much everything
- Need schematics, technical manuals and so on.
- Eventually, will need software.
- Disassembler needs fixing

*******************************************************************************************/


#include "emu.h"
#include "cpu/z8000/z8000.h"
#include "cpu/m6502/m6510.h"
#include "machine/bankdev.h"
#include "machine/z80daisy.h"
#include "machine/z80scc.h"
#include "machine/z8010.h"
#include "bus/rs232/rs232.h"
#include "bus/centronics/ctronics.h"
#include "machine/z8536.h"
#include "c900_pdmac.h"
#include "c900_kbd.h"
#include "screen.h"
#include "sound/spkrdev.h"
#include "emupal.h"
#include "speaker.h"
#include "logmacro.h"

class c900_daisy_device : public device_t, public z80_daisy_chain_interface
{
public:
	c900_daisy_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);
	uint16_t viack_r() {
		device_z80daisy_interface *intf = daisy_get_irq_device();
		if (!intf) {
			logerror("%s: VI acknowledge with no device requesting\n", machine().describe_context());
			return 0xffff; // nobody drives the bus
		}
		return intf->z80daisy_irq_ack();
	}
protected:
	void device_start() override {}
};

DEFINE_DEVICE_TYPE(C900_DAISY, c900_daisy_device, "c900_daisy", "Commodore 900 daisy chain device")

c900_daisy_device::c900_daisy_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, C900_DAISY, tag, owner, clock)
	, z80_daisy_chain_interface(mconfig, *this) {}

namespace {

class c900_state : public driver_device
{
public:
	c900_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_mmu(*this, "mmu")
		, m_daisy(*this, "c900_daisy")
		, m_physmem(*this, "physmem")
		, m_fdcpu(*this, "fdcpu")
		, m_spkrdev(*this, "speaker")
		, m_cio1(*this, "cio1")
		, m_cio2(*this, "cio2")
		, m_centronics(*this, "centronics")
		, m_cent_data(*this, "cent_data")
		, m_kbd(*this, "kbd")
		, m_vram(*this, "vram")
		, m_mousex(*this, "MOUSEX")
		, m_mousey(*this, "MOUSEY")
	{ }

	void c900(machine_config &config, unsigned vram_start, unsigned vram_end, const char *serial_device);
	void c900nv(machine_config &config);
	void c900hr(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	// sources of the shared vectored interrupt line, in daisy chain priority order
	enum { VI_CIO1, VI_SCC, VI_CIO2, VI_PDMAC };

	void sound_pb_w(u8 data);
	void kbd_pc_w(u8 data);

	void data_map(address_map &map) ATTR_COLD;
	void io_map(address_map &map) ATTR_COLD;
	void special_io_map(address_map &map) ATTR_COLD;
	void fdc_map(address_map &map) ATTR_COLD;
	void phys_mem_map(address_map &map);
	void virt_mem_map(address_map &map);
	void virt_data_map(address_map &map);
	bool translate_addr(int spacenum, bool write, offs_t &offset);
	uint16_t virt_r(address_space &space, offs_t offset, uint16_t mask);
	void virt_w(address_space &space, offs_t offset, uint16_t data, uint16_t mask);
	uint16_t segtack_r();
	void segt_interrupt(int state);
	template <unsigned N> void vi_w(int state);
	void ns_w(int state);
	uint16_t nviack_r();
	uint8_t cio1_pc_r();
	uint8_t cio2_pb_r();
	uint8_t mmu_read(offs_t offset);
	void mmu_write(offs_t offset, uint8_t data);
	u32 screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);


	required_device<z8001_device> m_maincpu;
	required_device<z8010_device> m_mmu;
	required_device<c900_daisy_device> m_daisy;
	required_device<address_map_bank_device> m_physmem;
	required_device<cpu_device> m_fdcpu;
	required_device<speaker_sound_device> m_spkrdev;
	required_device<z8036_device> m_cio1;
	required_device<z8036_device> m_cio2;
	required_device<centronics_device> m_centronics;
	required_device<output_latch_device> m_cent_data;
	required_device<c900_kbd_device> m_kbd;
	required_shared_ptr<u16> m_vram;
	required_ioport m_mousex, m_mousey;
	bool m_dma_on = false;
	bool m_normal_mode = false;
	int m_cent_busy = 0;
	int m_cent_ack = 1;
	uint8_t m_mmu_mode = 0;  // shadow of the MMU mode register
	uint8_t m_vi_lines = 0;
	uint32_t m_vram_start;
	uint32_t m_vram_end;
};

void c900_state::machine_start()
{
	save_item(NAME(m_normal_mode));
	save_item(NAME(m_vi_lines));
	save_item(NAME(m_mmu_mode));
}

void c900_state::machine_reset()
{
	// _CS is low during reset on this board, which makes the MMU come up with its master
	// enable set and translation off, i.e. passing addresses straight through
	m_mmu->write(0x00, 0x80);
	m_mmu_mode = 0x80;
	m_normal_mode = false;
}

void c900_state::sound_pb_w(u8 data)
{
	m_spkrdev->level_w(BIT(data, 0));
}

void c900_state::kbd_pc_w(u8 data)
{
	m_kbd->kbdclr_w(BIT(data, 3)); // PC3 = KBDCLR
}

uint8_t c900_state::cio1_pc_r() {
	return (m_kbd->pc_r() & ~0x01) | m_cent_busy;
}

uint8_t c900_state::cio2_pb_r() {
	return m_cent_ack << 1;
}

void c900_state::phys_mem_map(address_map &map)
{
	// Nothing else responds: the expansion bus and the video boards float, and the boot
	// ROM relies on reading back FFFF to fall back to the serial console
	map.unmap_value_high();
	map(0x000000, 0x007fff).rom().region("roms", 0);
	map(0x080000, 0x17ffff).ram(); // 1 MB motherboard DRAM
	map(m_vram_start, m_vram_end).ram().share("vram");
}

// A7-A0 bypass the MMU, which only sees and adds to the upper byte of the offset
#define NON_MMU_MASK 0xff

bool c900_state::translate_addr(int spacenum, bool write, offs_t &offset)
{
	bool stack_access = (spacenum == z8001_device::AS_STACK);

	// virt_r/virt_w are 16-bit handlers on a 16-bit wide space, so 'offset' is a
	// word offset (byte address >> 1).  The MMU translates byte addresses (segment
	// number in bits 16-22, offset in bits 0-15), and the physical bank device's
	// read16/write16 take word offsets again, so convert here and convert back.
	offset <<= 1;

	offs_t mmu_offset = offset & ~NON_MMU_MASK;

	bool code_access = (spacenum == AS_PROGRAM);
	int st = code_access ?
				(m_maincpu->is_ifetch1() ?
					z8002_device::ST_IFETCH_1 :
					z8002_device::ST_IFETCH_N) :
				(stack_access ?
					z8002_device::ST_REQ_STACK :
					z8002_device::ST_REQ_DATA);

	LOG("%s MMU MEM REQ (space %d): %06x\n", machine().describe_context(), spacenum, offset);

	if (st == z8002_device::ST_IFETCH_1)
		m_mmu->ifetch1_observed(offset);

	mmu_offset &= 0x3f'ffff;	// Mask off seg bit 6 to disable URS checking in MMUs

	// With master enable off the MMU tri-states its address outputs; assume the board
	// then passes the CPU's segment/offset through to memory unchanged.  Unverified
	// against schematics, but OpenCoherent's restart() relies on it to drop back into
	// the boot ROM, and the reference emulator behaves the same way.
	if (!BIT(m_mmu_mode, 7)){
		offset >>= 1;
		return true;
	}

	if (!m_mmu->translate(mmu_offset, write, !m_normal_mode, m_dma_on, st))
		return false; // cycle suppressed by the MMU

	offset = (mmu_offset | (offset & NON_MMU_MASK)) >> 1;
	return true;
}


uint16_t c900_state::virt_r(address_space &space, offs_t offset, uint16_t mask)
{
    uint16_t data = 0;
    if (translate_addr(space.spacenum(), false, offset))
        data = m_physmem->read16(offset, mask);
	else if (space.spacenum() == AS_PROGRAM)
        data = 0x8d07;  // NOP instruction

    return data;
}

void c900_state::virt_w(address_space &space, offs_t offset, uint16_t data, uint16_t mask)
{
    if (translate_addr(space.spacenum(), true, offset))
        m_physmem->write16(offset, data, mask);
}

void c900_state::virt_mem_map(address_map &map)
{
	map(0x00'0000, 0x7f'ffff).rw(FUNC(c900_state::virt_r), FUNC(c900_state::virt_w));
}

void c900_state::virt_data_map(address_map &map)
{
	map(0x00'0000, 0x7f'ffff).rw(FUNC(c900_state::virt_r), FUNC(c900_state::virt_w));
}

void c900_state::io_map(address_map &map)
{
	map(0x0000, 0x007f).rw("cio1", FUNC(z8036_device::read), FUNC(z8036_device::write)).umask16(0x00ff);
	map(0x0080, 0x00ff).rw("cio2", FUNC(z8036_device::read), FUNC(z8036_device::write)).umask16(0x00ff);
	map(0x0100, 0x013f).rw("scc", FUNC(scc8030_device::zbus_r), FUNC(scc8030_device::zbus_w)).umask16(0x00ff);
	map(0x0500, 0x05ff).rw("pdmac", FUNC(c900_pdmac_device::read), FUNC(c900_pdmac_device::write));
	map(0x0204, 0x0205).rw(m_kbd, FUNC(c900_kbd_device::read_latch), FUNC(c900_kbd_device::write_latch)).umask16(0x00ff);
	// mouse: 10-bit free-running position counters, buttons in the top bits of the X port
	map(0x0400, 0x0401).lr16(NAME([this] () { return m_mousex->read(); }));
	map(0x0402, 0x0403).lr16(NAME([this] () { return m_mousey->read(); }));
	map(0x0404, 0x041f).nopw(); // video board control (0x0404 / 0x0414 bitmap enable, 0x0410-0x0418 text CRTC); TODO
}

void c900_state::special_io_map(address_map &map)
{
	map(0x0000, 0xffff).rw(FUNC(c900_state::mmu_read), FUNC(c900_state::mmu_write));
}

#define MMU1  0xfc
#define MMU12 0xf8
bool mmu_valid(offs_t offset) {
	offs_t base = (offset & 0x00ff);
	return (base == MMU1 || base == MMU12);
}

uint8_t c900_state::mmu_read(offs_t offset)
{
	if (mmu_valid(offset)) {
		return (m_mmu->read(offset >> 8));
	}else{
		return (0);
	}
}

void c900_state::mmu_write(offs_t offset, uint8_t data)
{
	if (mmu_valid(offset)) {
		if ((offset >> 8) == 0x00)
			m_mmu_mode = data;
		m_mmu->write(offset >> 8, data);
	}else{
		return ;
	}
}

void c900_state::fdc_map(address_map &map)
{
	map(0x0000, 0x01ff).noprw(); // internal
	map(0xe000, 0xffff).rom().region("fdc", 0);
}

uint16_t c900_state::segtack_r()
{
	uint16_t code = m_mmu->segtack_r();
	if (code == 0) {
		segt_interrupt(0);
	}
	return code;
}

void c900_state::segt_interrupt(int state)
{
	m_maincpu->set_input_line(z8001_device::SEGT_LINE, state ? ASSERT_LINE : CLEAR_LINE);
}

// The daisy chain picks which device answers the acknowledge. The line itself is the OR of
// the requests; IEI/IEO gating of lower priority requests while a device is under service
// isn't modelled, since the SCC doesn't report when its IUS clears.
template <unsigned N>
void c900_state::vi_w(int state)
{
	if (state)
		m_vi_lines |= 1U << N;
	else
		m_vi_lines &= ~(1U << N);

	m_maincpu->set_input_line(z8001_device::VI_LINE, m_vi_lines ? ASSERT_LINE : CLEAR_LINE);
}

void c900_state::ns_w(int state)
{
	m_normal_mode = bool(state);
}

uint16_t c900_state::nviack_r()
{
	//TODO
	return (0);
}

static INPUT_PORTS_START( c900 )
	PORT_START("MOUSEX")
	PORT_BIT(0x03ff, 0, IPT_MOUSE_X) PORT_SENSITIVITY(100) PORT_KEYDELTA(0)
	// Button names are from the OpenCoherent hr driver (DACTION/DWMENU/DSMENU); its GUI
	// treats 0x8000 as the left button, 0x4000 as the middle and 0x2000 as the right.
	PORT_BIT(0x1c00, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x2000, IP_ACTIVE_HIGH, IPT_BUTTON2) PORT_NAME("Right Button (Action)")
	PORT_BIT(0x4000, IP_ACTIVE_HIGH, IPT_BUTTON3) PORT_NAME("Middle Button (Window Menu)")
	PORT_BIT(0x8000, IP_ACTIVE_HIGH, IPT_BUTTON1) PORT_NAME("Left Button (Screen Menu)")
	PORT_START("MOUSEY")
	PORT_BIT(0x03ff, 0, IPT_MOUSE_Y) PORT_SENSITIVITY(100) PORT_KEYDELTA(0)
	PORT_BIT(0xfc00, IP_ACTIVE_HIGH, IPT_UNUSED)
INPUT_PORTS_END

/*************************************
 *
 *  Video refresh
 *
 *************************************/

#define X_RES 1024
#define Y_RES 800

u32 c900_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	for (int y = 0; y < Y_RES; y++){
		u16 *const line = &bitmap.pix(y);
		u16 const *video_ram = &m_vram[y * X_RES / 16]; // 128 bytes per scan line

		for (int x = 0; x < X_RES; x += 16){
			u16 const word = *(video_ram++);
			for (int b = 0; b < 16; b++){
				line[x + b] = BIT(word, 15 - b);
			}
		}
	}

	return 0;
}



#if 0
/* F4 Character Displayer */
static const gfx_layout charlayout =
{
	8, 16,                   /* 8 x 16 characters */
	256,                    /* 256 characters */
	1,                  /* 1 bits per pixel */
	{ 0 },                  /* no bitplanes */
	/* x offsets */
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	/* y offsets */
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8, 8*8, 9*8, 10*8, 11*8, 12*8, 13*8, 14*8, 15*8 },
	8*16                    /* every char takes 16 bytes */
};

static GFXDECODE_START( gfx_c900 )
	GFXDECODE_ENTRY( "chargen", 0x0000, charlayout, 0, 1 )
GFXDECODE_END
#endif

static const z80_daisy_config c900_daisy_chain[] =
{
	{ "cio1" },
	{ "scc" },
	{ "cio2" },
	{ "pdmac" },
	{ nullptr }
};

void c900_state::c900(machine_config &config, unsigned vram_start, unsigned vram_end, const char *serial_device)
{
	/* basic machine hardware */
	Z8001(config, m_maincpu, 12_MHz_XTAL / 2);
	m_maincpu->set_addrmap(AS_PROGRAM, &c900_state::virt_mem_map);
	m_maincpu->set_addrmap(AS_DATA, &c900_state::virt_data_map);
	m_maincpu->set_addrmap(AS_IO, &c900_state::io_map);
	m_maincpu->set_addrmap(z8001_device::AS_SIO, &c900_state::special_io_map);
	m_maincpu->segtack().set(FUNC(c900_state::segtack_r));
	m_maincpu->viack().set(m_daisy, FUNC(c900_daisy_device::viack_r));
	m_maincpu->ns().set(FUNC(c900_state::ns_w));
	m_maincpu->nviack().set(FUNC(c900_state::nviack_r));

	C900_DAISY(config, m_daisy);
	m_daisy->set_daisy_config(c900_daisy_chain);

	m_vram_start = vram_start;
	m_vram_end = vram_end;
	ADDRESS_MAP_BANK(config, m_physmem).set_map(&c900_state::phys_mem_map).set_options(ENDIANNESS_BIG, 16, 24, 0x100'0000);

	Z8010(config, m_mmu, 12_MHz_XTAL / 2);
	m_mmu->out_segt_cb().set(FUNC(c900_state::segt_interrupt));

	M6508(config, m_fdcpu, 12_MHz_XTAL / 8); // PH1/PH2 = 1.5 MHz
	m_fdcpu->set_addrmap(AS_PROGRAM, &c900_state::fdc_map);

	/*GFXDECODE(config, "gfxdecode", "palette", gfx_c900);
	PALETTE(config, "palette", palette_device::MONOCHROME);*/

	Z8036(config, m_cio1, 12_MHz_XTAL / 2); // PCLK; Coherent derives its 100 Hz tick from this
	m_cio1->irq_wr_cb().set(FUNC(c900_state::vi_w<VI_CIO1>));
	m_cio1->pc_wr_cb().set(FUNC(c900_state::kbd_pc_w));
	m_cio1->pa_rd_cb().set(m_kbd, FUNC(c900_kbd_device::pa_r));
	m_cio1->pb_wr_cb().set(m_cent_data, FUNC(output_latch_device::write));
	m_cio1->pc_rd_cb().set(FUNC(c900_state::cio1_pc_r));

	Z8036(config, m_cio2, 12_MHz_XTAL / 2); // U66, tested by the boot ROM as "CIO2"; IEEE-488 interface?
	m_cio2->pb_wr_cb().set(FUNC(c900_state::sound_pb_w));
	m_cio2->pb_rd_cb().set(FUNC(c900_state::cio2_pb_r));
	m_cio2->pc_wr_cb().set(m_centronics,
		FUNC(centronics_device::write_strobe)
	).bit(3);
	m_cio2->irq_wr_cb().set(FUNC(c900_state::vi_w<VI_CIO2>));

	CENTRONICS(config, m_centronics, centronics_devices, "printer");
	m_centronics->busy_handler().set([this] (int state) { m_cent_busy = state; });
	m_centronics->ack_handler().set([this] (int state) { m_cent_ack = state; });
	m_centronics->ack_handler().append(m_cio2, FUNC(z8036_device::pb1_w));

	OUTPUT_LATCH(config, m_cent_data);
	m_centronics->set_output_latch(*m_cent_data);

	scc8030_device &scc(SCC8030(config, "scc", 12_MHz_XTAL / 2)); // 5'850'000 is the ideal figure
	/* Port B */
	scc.out_txdb_callback().set("rs232", FUNC(rs232_port_device::write_txd));
	scc.out_dtrb_callback().set("rs232", FUNC(rs232_port_device::write_dtr));
	scc.out_rtsb_callback().set("rs232", FUNC(rs232_port_device::write_rts));
	scc.out_int_callback().set(FUNC(c900_state::vi_w<VI_SCC>));

	rs232_port_device &rs232(RS232_PORT(config, "rs232", default_rs232_devices, serial_device));
	rs232.rxd_handler().set("scc", FUNC(scc8030_device::rxb_w));
	rs232.cts_handler().set("scc", FUNC(scc8030_device::ctsb_w));

	c900_pdmac_device &pdmac(C900PDMAC(config, "pdmac", 12_MHz_XTAL / 8));
	pdmac.ireq_callback().set(FUNC(c900_state::vi_w<VI_PDMAC>));
	pdmac.set_dma_space(m_physmem, AS_PROGRAM);

	C900KBD(config, m_kbd, 12_MHz_XTAL / 8);
	m_kbd->strobe_callback().set(m_cio1, FUNC(z8036_device::pa7_w));

	SPEAKER(config, "mono").front_center();
	SPEAKER_SOUND(config, m_spkrdev).add_route(ALL_OUTPUTS, "mono", 0.05);
}

void c900_state::c900hr(machine_config &config)
{
	c900(config, 0x3e0000, 0x3fffff, nullptr);
	/* video hardware */
	screen_device &screen(SCREEN(config, "screen"));
	screen.set_raw(70'000'000, 1344, 0, 1024, 838, 0, 800);
	screen.set_screen_update(FUNC(c900_state::screen_update));
	screen.set_palette("palette");
	PALETTE(config, "palette", palette_device::MONOCHROME);
}

void c900_state::c900nv(machine_config &config)
{
	c900(config, 0x7f0000, 0x7fffff, "terminal");
}

ROM_START( c900 )
	ROM_REGION16_BE( 0x8000, "roms", 0 )
	ROM_LOAD16_BYTE( "c 900 boot-h v 1.0.bin.u17", 0x0000, 0x4000, CRC(c3aa7fc1) SHA1(ff12dd100fa7b1e7e931e9a8ef4c4f5cc056e099) )
	ROM_LOAD16_BYTE( "c 900 boot-l v 1.0.bin.u18", 0x0001, 0x4000, CRC(0aa39272) SHA1(b2c5da4586d38fc66bb33aafeae4dbda36080f1e) )

	ROM_REGION( 0x2000, "fdc", 0 )
	ROM_LOAD( "s41_6-20-85.bin", 0x0000, 0x2000, CRC(ec245721) SHA1(4cc19014b4887833a56b1236dc5fe39cc5d7b5c3) )

	ROM_REGION( 0x1000, "chargen", 0 ) // this must be for the c900 terminal as the mainframe has no video output
	ROM_LOAD( "380217-01.u2", 0x0000, 0x1000, CRC(64cb4171) SHA1(e60d796170addfd27e2c33090f9c512c7e3f99f5) )
ROM_END

} // anonymous namespace

#define rom_c900nv rom_c900
#define rom_c900hr rom_c900

/*    YEAR  NAME  PARENT  COMPAT  MACHINE  INPUT  CLASS       INIT        COMPANY      FULLNAME         FLAGS */
COMP( 1985, c900, 0,      0,      c900hr,    c900,  c900_state, empty_init, "Commodore", "Commodore 900 (high-res workstation)", MACHINE_NO_SOUND | MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )
COMP( 1985, c900nv, c900,      0,      c900nv,    c900,  c900_state, empty_init, "Commodore", "Commodore 900 (server without video)", MACHINE_NO_SOUND | MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )
