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
#include "machine/z80scc.h"
#include "machine/z8010.h"
#include "bus/rs232/rs232.h"
#include "machine/z8536.h"
#include "sound/spkrdev.h"
#include "emupal.h"
#include "speaker.h"
#include "logmacro.h"

namespace {

class c900_state : public driver_device
{
public:
	c900_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_mmu(*this, "mmu")
		, m_physmem(*this, "physmem")
		, m_fdcpu(*this, "fdcpu")
		, m_spkrdev(*this, "speaker")
	{ }

	void c900(machine_config &config);

private:
	void sound_pb_w(u8 data);

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
	uint8_t mmu_read(offs_t offset);
	void mmu_write(offs_t offset, uint8_t data);


	required_device<z8001_device> m_maincpu;
	required_device<z8010_device> m_mmu;
	required_device<address_map_bank_device> m_physmem;
	required_device<cpu_device> m_fdcpu;
	required_device<speaker_sound_device> m_spkrdev;
	bool m_dma_on = false;
};

void c900_state::sound_pb_w(u8 data)
{
	m_spkrdev->level_w(BIT(data, 0));
}

void c900_state::phys_mem_map(address_map &map)
{
	map(0x000000, 0x007fff).rom().region("roms", 0);
	map(0x080000, 0x0fffff).ram();
	//map(0x3f0000, 0x3f1fff).ram();
}

#define NON_MMU_MASK 0x1ff

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

	mmu_offset &= 0x3f'ffff;	// Mask off seg bit 6 to disable URS checking in MMUs

	if (!m_mmu->translate(mmu_offset, write, true, m_dma_on, st)){
		mmu_offset = 0;
	}
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
	map(0x0000, 0x007f).rw("cio", FUNC(z8036_device::read), FUNC(z8036_device::write)).umask16(0x00ff);
	map(0x0100, 0x013f).rw("scc", FUNC(scc8030_device::zbus_r), FUNC(scc8030_device::zbus_w)).umask16(0x00ff);
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

static INPUT_PORTS_START( c900 )
INPUT_PORTS_END

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

void c900_state::c900(machine_config &config)
{
	/* basic machine hardware */
	Z8001(config, m_maincpu, 12_MHz_XTAL / 2);
	m_maincpu->set_addrmap(AS_PROGRAM, &c900_state::virt_mem_map);
	m_maincpu->set_addrmap(AS_DATA, &c900_state::virt_data_map);
	m_maincpu->set_addrmap(AS_IO, &c900_state::io_map);
	m_maincpu->set_addrmap(z8001_device::AS_SIO, &c900_state::special_io_map);
	m_maincpu->segtack().set(FUNC(c900_state::segtack_r));

	ADDRESS_MAP_BANK(config, m_physmem).set_map(&c900_state::phys_mem_map).set_options(ENDIANNESS_BIG, 16, 23, 0x100'0000);

	Z8010(config, m_mmu, 12_MHz_XTAL / 2);
	m_mmu->out_segt_cb().set(FUNC(c900_state::segt_interrupt));

	M6508(config, m_fdcpu, 12_MHz_XTAL / 8); // PH1/PH2 = 1.5 MHz
	m_fdcpu->set_addrmap(AS_PROGRAM, &c900_state::fdc_map);

	GFXDECODE(config, "gfxdecode", "palette", gfx_c900);
	PALETTE(config, "palette", palette_device::MONOCHROME);

	z8036_device &cio(Z8036(config, "cio", 12_MHz_XTAL / 16)); // SNDCLK = 750kHz
	cio.pb_wr_cb().set(FUNC(c900_state::sound_pb_w));

	scc8030_device &scc(SCC8030(config, "scc", 12_MHz_XTAL / 2)); // 5'850'000 is the ideal figure
	/* Port B */
	scc.out_txdb_callback().set("rs232", FUNC(rs232_port_device::write_txd));
	scc.out_dtrb_callback().set("rs232", FUNC(rs232_port_device::write_dtr));
	scc.out_rtsb_callback().set("rs232", FUNC(rs232_port_device::write_rts));
	//scc.out_int_callback().set("rs232", FUNC(c900_state::scc_int));

	rs232_port_device &rs232(RS232_PORT(config, "rs232", default_rs232_devices, "terminal"));
	rs232.rxd_handler().set("scc", FUNC(scc8030_device::rxb_w));
	rs232.cts_handler().set("scc", FUNC(scc8030_device::ctsb_w));

	SPEAKER(config, "mono").front_center();
	SPEAKER_SOUND(config, m_spkrdev).add_route(ALL_OUTPUTS, "mono", 0.05);
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


/*    YEAR  NAME  PARENT  COMPAT  MACHINE  INPUT  CLASS       INIT        COMPANY      FULLNAME         FLAGS */
COMP( 1985, c900, 0,      0,      c900,    c900,  c900_state, empty_init, "Commodore", "Commodore 900", MACHINE_NO_SOUND | MACHINE_NOT_WORKING | MACHINE_SUPPORTS_SAVE )
