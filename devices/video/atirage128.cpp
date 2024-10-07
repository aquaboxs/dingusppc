/*
DingusPPC - The Experimental PowerPC Macintosh emulator
Copyright (C) 2018-23 divingkatae and maximum
                      (theweirdo)     spatium
(Contact divingkatae#1017 or powermax#2286 on Discord for more info)
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <core/bitops.h>
#include <core/timermanager.h>
#include <devices/common/hwcomponent.h>
#include <devices/common/pci/pcidevice.h>
#include <devices/deviceregistry.h>
#include <devices/video/atirage128.h>
#include <devices/video/displayid.h>
#include <endianswap.h>
#include <loguru.hpp>
#include <memaccess.h>

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>

/* Human readable ATI Rage 128 HW register names for easier debugging. */
static const std::map<uint16_t, std::string> rage128_reg_names = {
    #define one_reg_name(x) {ATI_ ## x, #x}
    one_reg_name(MM_INDEX),
    one_reg_name(MM_DATA),
    one_reg_name(CLOCK_CNTL_INDEX),
    one_reg_name(CLOCK_CNTL_DATA),
    one_reg_name(BIOS_0_SCRATCH),
    one_reg_name(BIOS_1_SCRATCH),
    one_reg_name(BIOS_2_SCRATCH),
    one_reg_name(BIOS_3_SCRATCH),
    one_reg_name(BUS_CNTL),
    one_reg_name(BUS_CNTL1),
    one_reg_name(GEN_INT_CNTL),
    one_reg_name(GEN_INT_STATUS),
    one_reg_name(CRTC_GEN_CNTL),
    one_reg_name(CRTC_EXT_CNTL),
    one_reg_name(DAC_CNTL),
    one_reg_name(CRTC_STATUS),
    one_reg_name(GPIO_MONID),
    one_reg_name(SEPROM_CNTL),
    one_reg_name(I2C_CNTL_1),
    one_reg_name(PALETTE_INDEX),
    one_reg_name(PALETTE_DATA),
    one_reg_name(CONFIG_CNTL),
    one_reg_name(CONFIG_XSTRAP),
    one_reg_name(CONFIG_BONDS),
    one_reg_name(GEN_RESET_CNTL),
    one_reg_name(GEN_STATUS),
    one_reg_name(CNFG_MEMSIZE),
    one_reg_name(CONFIG_APER_0_BASE),
    one_reg_name(CONFIG_APER_1_BASE),
    one_reg_name(CONFIG_APER_SIZE),
    one_reg_name(CONFIG_REG_1_BASE),
    one_reg_name(CONFIG_REG_APER_SIZE),
    one_reg_name(TEST_DEBUG_CNTL),
    one_reg_name(TEST_DEBUG_MUX),
    one_reg_name(HW_DEBUG),
    one_reg_name(HOST_PATH_CNTL),
    one_reg_name(MEM_CNTL),
    one_reg_name(EXT_MEM_CNTL),
    one_reg_name(MEM_ADDR_CONFIG),
    one_reg_name(MEM_INTF_CNTL),
    one_reg_name(MEM_STR_CNTL),
    one_reg_name(VIPH_CONTROL),
    one_reg_name(CRTC_H_TOTAL_DISP),
    one_reg_name(CRTC_H_SYNC_STRT_WID),
    one_reg_name(CRTC_V_TOTAL_DISP),
    one_reg_name(CRTC_V_SYNC_STRT_WID),
    one_reg_name(CRTC_VLINE_CRNT_VLINE),
    one_reg_name(CRTC_CRNT_FRAME),
    one_reg_name(CRTC_GUI_TRIG_VLINE),
    one_reg_name(CRTC_DEBUG),
    one_reg_name(CRTC_OFFSET),
    one_reg_name(CRTC_OFFSET_CNTL),
    one_reg_name(CRTC_PITCH),
    one_reg_name(OVR_CLR),
    one_reg_name(OVR_WID_LEFT_RIGHT),
    one_reg_name(OVR_WID_TOP_BOTTOM),
    one_reg_name(SNAPSHOT_VH_COUNTS),
    one_reg_name(SNAPSHOT_F_COUNT),
    one_reg_name(N_VIF_COUNT),
    one_reg_name(SNAPSHOT_VIF_COUNT),
    one_reg_name(CUR_OFFSET),
    one_reg_name(CUR_HORZ_VERT_POSN),
    one_reg_name(CUR_HORZ_VERT_OFF),
    one_reg_name(CUR_CLR0),
    one_reg_name(CUR_CLR1),
    one_reg_name(DAC_EXT_CNTL),
    one_reg_name(DAC_CRC_SIG),
    one_reg_name(PM4_BUFFER_DL_WPTR_DELAY),
    one_reg_name(DST_OFFSET),
    one_reg_name(DST_PITCH),
    one_reg_name(DST_WIDTH),
    one_reg_name(DST_HEIGHT),
    one_reg_name(SRC_X),
    one_reg_name(SRC_Y),
    one_reg_name(DST_X),
    one_reg_name(DST_Y),
    one_reg_name(SRC_PITCH_OFFSET),
    one_reg_name(DST_PITCH_OFFSET),
    one_reg_name(SRC_Y_X),
    one_reg_name(DST_Y_X),
    one_reg_name(DST_HEIGHT_WIDTH),
    one_reg_name(DST_WIDTH_X),
    one_reg_name(DST_HEIGHT_WIDTH_8),
    one_reg_name(SRC_X_Y),
    one_reg_name(DST_X_Y),
    one_reg_name(DST_WIDTH_HEIGHT),
    one_reg_name(DST_WIDTH_X_INCY),
    one_reg_name(DST_HEIGHT_Y),
    one_reg_name(DST_X_SUB),
    one_reg_name(DST_Y_SUB),
    one_reg_name(SRC_OFFSET),
    one_reg_name(SRC_PITCH),
    one_reg_name(DST_WIDTH_BW),
    one_reg_name(GUI_SCRATCH_REG0),
    one_reg_name(GUI_SCRATCH_REG1),
    one_reg_name(GUI_SCRATCH_REG2),
    one_reg_name(GUI_SCRATCH_REG3),
    one_reg_name(GUI_SCRATCH_REG4),
    one_reg_name(GUI_SCRATCH_REG5),
    one_reg_name(DST_BRES_ERR),
    one_reg_name(DST_BRES_INC),
    one_reg_name(DST_BRES_DEC),
    one_reg_name(DST_BRES_LNTH),
    one_reg_name(DST_BRES_LNTH_SUB),
    one_reg_name(SC_LEFT),
    one_reg_name(SC_RIGHT),
    one_reg_name(SC_TOP),
    one_reg_name(SC_BOTTOM),
    one_reg_name(SC_CNTL),
    one_reg_name(DEFAULT_OFFSET),
    one_reg_name(DEFAULT_PITCH),
    one_reg_name(DEFAULT_SC_BOTTOM_RIGHT),
    one_reg_name(SC_TOP_LEFT),
    one_reg_name(SC_BOTTOM_RIGHT),
    one_reg_name(SRC_SC_BOTTOM_RIGHT),
    one_reg_name(WAIT_UNTIL),
    one_reg_name(GUI_STAT),
    one_reg_name(HOST_DATA0),
    one_reg_name(HOST_DATA1),
    one_reg_name(HOST_DATA2),
    one_reg_name(HOST_DATA3),
    one_reg_name(HOST_DATA4),
    one_reg_name(HOST_DATA5),
    one_reg_name(HOST_DATA6),
    one_reg_name(HOST_DATA7),
    one_reg_name(HOST_DATA_LAST),
    one_reg_name(W_START),
    one_reg_name(CONSTANT_COLOR),
    one_reg_name(Z_VIS),
    one_reg_name(WINDOW_XY_OFFSET),
    one_reg_name(DRAW_LINE_POINT),
    one_reg_name(DST_PITCH_OFFSET_C),
    one_reg_name(SC_TOP_LEFT_C),
};

ATIRage128::ATIRage128(uint16_t dev_id) : PCIDevice("ati-rage128"), VideoCtrlBase(640, 480) {
    uint8_t asic_id;

    supports_types(HWCompType::MMIO_DEV | HWCompType::PCI_DEV);

    this->vram_size = GET_INT_PROP("gfxmem_size") << 20;    // convert MBs to bytes

    // allocate video RAM
    this->vram_ptr = std::unique_ptr<uint8_t[]>(new uint8_t[this->vram_size]);

    // ATI Rage driver needs to know ASIC ID (manufacturer's internal chip code)
    // to operate properly
    switch (dev_id) {
    case ATI_RAGE_128_GL_DEV_ID:
        asic_id = 0x00;
        break;
    default:
        asic_id = 0xDD;
        LOG_F(WARNING, "ATI Rage 128: bogus ASIC ID assigned!");
    }

    // set up PCI configuration space header
    this->vendor_id   = PCI_VENDOR_ATI;
    this->device_id   = dev_id;
    this->subsys_vndr = 0xb530;
    this->subsys_id   = 0x0408;    // adapter ID
    this->class_rev   = (0x030000 << 8) | asic_id;
    this->min_gnt     = 8;
    this->irq_pin     = 1;

    for (int i = 0; i < this->aperture_count; i++) {
        this->bars_cfg[i] = (uint32_t)(-this->aperture_size[i] | this->aperture_flag[i]);
    }

    this->finish_config_bars();

    this->pci_notify_bar_change = [this](int bar_num) { 
        this->notify_bar_change(bar_num); 
    };

    // stuff default values into chip registers
    //this->regs[ATI_CONFIG_CHIP_ID] = (asic_id << 24) | dev_id;

    // initialize display identification
    this->disp_id = std::unique_ptr<DisplayID>(new DisplayID());

    uint8_t mon_code = this->disp_id->read_monitor_sense(0, 0);

    this->regs[ATI_GPIO_MONID] = ((mon_code & 6) << 11) | ((mon_code & 1) << 8);
}

void ATIRage128::change_one_bar(
    uint32_t& aperture, uint32_t aperture_size, uint32_t aperture_new, int bar_num) {
    if (aperture != aperture_new) {
        if (aperture)
            this->host_instance->pci_unregister_mmio_region(aperture, aperture_size, this);

        aperture = aperture_new;
        if (aperture)
            this->host_instance->pci_register_mmio_region(aperture, aperture_size, this);

        LOG_F(INFO, "%s: aperture[%d] set to 0x%08X", this->name.c_str(), bar_num, aperture);
    }
}

void ATIRage128::notify_bar_change(int bar_num)
{
    switch (bar_num) {
    case 0:
        change_one_bar(this->aperture_base[bar_num],
                       this->aperture_size[bar_num] - this->vram_size,
                       this->bars[bar_num] & ~15, bar_num);
        break;
    case 2:
        change_one_bar(this->aperture_base[bar_num],
                       this->aperture_size[bar_num],
                       this->bars[bar_num] & ~15, bar_num);
        break;
    case 1:
        this->aperture_base[1] = this->bars[bar_num] & ~3;
        LOG_F(INFO, "%s: I/O space address set to 0x%08X", this->name.c_str(),
              this->aperture_base[1]);
        break;
    }
}

uint32_t ATIRage128::pci_cfg_read(uint32_t reg_offs, AccessDetails& details) {
    if (reg_offs < 128) {
        return PCIDevice::pci_cfg_read(reg_offs, details);
    }

    LOG_READ_UNIMPLEMENTED_CONFIG_REGISTER();

    return 0;
}

const char* ATIRage128::get_reg_name(uint32_t reg_offset) {
    auto iter = rage128_reg_names.find(reg_offset & ~3);
    if (iter != rage128_reg_names.end()) {
        return iter->second.c_str();
    } else {
        return "unknown Rage 128 register";
    }
}

uint32_t ATIRage128::read_reg(uint32_t reg_offset, uint32_t size) {
    uint32_t reg_num = reg_offset >> 2;
    uint32_t offset = reg_offset & 3;
    uint64_t result = this->regs[reg_num];

    switch (reg_num) {
    case ATI_GUI_STAT:
        result = this->cmd_fifo_size << 16; // HACK: tell the guest the command FIFO is empty
        break;
    }

    if (offset || size != 4) { // slow path
        if ((offset + size) > 4) {
            result |= (uint64_t)(this->regs[reg_num + 1]) << 32;
        }
        result = extract_bits<uint64_t>(result, offset * 8, size * 8);
    }

    return static_cast<uint32_t>(result);
}

void ATIRage128::write_reg(uint32_t reg_offset, uint32_t value, uint32_t size) {
    uint32_t reg_num = reg_offset >> 2;
    uint32_t offset = reg_offset & 3;
    uint32_t old_value = this->regs[reg_num];
    uint32_t new_value;

    if (offset || size != 4) { // slow path
        if ((offset + size) > 4) {
            ABORT_F("%s: unaligned DWORD writes not implemented", this->name.c_str());
        }
        uint64_t val = old_value;
        insert_bits<uint64_t>(val, value, offset * 8, size * 8);
        value = static_cast<uint32_t>(val);
    }

    switch (reg_num) {
    case ATI_CRTC_H_TOTAL_DISP:
        new_value = value;
        LOG_F(9, "%s: ATI_CRTC_H_TOTAL_DISP set to 0x%08X", this->name.c_str(), value);
        break;
    case ATI_CRTC_VLINE_CRNT_VLINE:
        new_value = old_value;
        insert_bits<uint32_t>(new_value, value, ATI_CRTC_VLINE, ATI_CRTC_VLINE_size);
        break;
    case ATI_CRTC_OFFSET:
        new_value = value;
        this->crtc_update();
        return;
    case ATI_CRTC_GEN_CNTL:
        new_value = value;
        if (bit_changed(old_value, new_value, ATI_CRTC_DISPLAY_DIS)) {
            if (bit_set(new_value, ATI_CRTC_DISPLAY_DIS)) {
                this->blank_on = true;
                this->blank_display();
            } else {
                this->blank_on = false;
            }
        }

        this->regs[reg_num] = new_value;
        if (bit_changed(old_value, new_value, ATI_CRTC_ENABLE)) {
            if (bit_set(new_value, ATI_CRTC_ENABLE) &&
                !bit_set(new_value, ATI_CRTC_DISPLAY_DIS)) {
                this->crtc_update();
            }
        }
        break;
    case ATI_CUR_CLR0:
    case ATI_CUR_CLR1:
        new_value = value;
        this->cursor_dirty = true;
        draw_fb = true;
        return;
    case ATI_CUR_OFFSET:
        new_value = value;
        if (old_value != new_value)
            this->cursor_dirty = true;
        draw_fb = true;
        return;
    case ATI_CUR_HORZ_VERT_OFF:
        new_value = value;
        if (old_value != new_value)
            this->cursor_dirty = true;
        draw_fb = true;
        return;
    case ATI_CUR_HORZ_VERT_POSN:
        new_value = value;
        draw_fb = true;
        break;
    case ATI_GPIO_MONID:
        new_value = value;
        if (offset <= 1 && offset + size > 1) {
            uint8_t gpio_levels = (new_value >> 8) & 0xFFU;
            gpio_levels = ((gpio_levels & 0x30) >> 3) | (gpio_levels & 1);
            uint8_t gpio_dirs = (new_value >> 24) & 0xFFU;
            gpio_dirs = ((gpio_dirs & 0x30) >> 3) | (gpio_dirs & 1);
            gpio_levels = this->disp_id->read_monitor_sense(gpio_levels, gpio_dirs);
            insert_bits<uint32_t>(new_value,
                                ((gpio_levels & 6) << 3) | (gpio_levels & 1), 8, 8);
        }
        break;
    default:
        new_value = value;
        break;
    }
}

bool ATIRage128::io_access_allowed(uint32_t offset) {
    if (offset >= this->aperture_base[1] && offset < (this->aperture_base[1] + 0x100)) {
        if (this->command & 1) {
            return true;
        }
        LOG_F(WARNING, "ATI I/O space disabled in the command reg");
    }
    return false;
}

bool ATIRage128::pci_io_read(uint32_t offset, uint32_t size, uint32_t* res) {
    if (!this->io_access_allowed(offset)) {
        return false;
    }

    *res = BYTESWAP_SIZED(this->read_reg(offset - this->aperture_base[1], size), size);
    return true;
}

bool ATIRage128::pci_io_write(uint32_t offset, uint32_t value, uint32_t size) {
    if (!this->io_access_allowed(offset)) {
        return false;
    }

    this->write_reg(offset - this->aperture_base[1], BYTESWAP_SIZED(value, size), size);
    return true;
}

uint32_t ATIRage128::read(uint32_t rgn_start, uint32_t offset, int size)
{
    if (rgn_start == this->aperture_base[0] && offset < this->aperture_size[0]) {
        if (offset < this->vram_size) { // little-endian VRAM region
            return read_mem(&this->vram_ptr[offset], size);
        }
        if (offset >= BE_FB_OFFSET) { // big-endian VRAM region
            return read_mem(&this->vram_ptr[offset - BE_FB_OFFSET], size);
        }
        //if (!bit_set(this->regs[ATI_BUS_CNTL], ATI_BUS_APER_REG_DIS)) {
            if (offset >= MM_REGS_0_OFF) { // memory-mapped registers, block 0
                return BYTESWAP_SIZED(this->read_reg(offset & 0x3FF, size), size);
            }
            if (offset >= MM_REGS_1_OFF
                //&& bit_set(this->regs[ATI_BUS_CNTL], ATI_BUS_EXT_REG_EN)
            ) { // memory-mapped registers, block 1
                return BYTESWAP_SIZED(this->read_reg((offset & 0x3FF) + 0x400, size), size);
            }
        //}
        LOG_F(WARNING, "%s: read  unmapped aperture[0] region %08x.%c",
              this->name.c_str(), offset, SIZE_ARG(size));
        return 0;
    }

    if (rgn_start == this->aperture_base[2] && offset < this->aperture_size[2]) {
        LOG_F(WARNING, "%s: read  unmapped aperture[2] region %08x.%c",
              this->name.c_str(), offset, SIZE_ARG(size));
        return 0;
    }

    // memory mapped expansion ROM region
    if (rgn_start == this->exp_rom_addr) {
        if (offset < this->exp_rom_size)
            return read_mem(&this->exp_rom_data[offset], size);
        LOG_F(WARNING, "%s: read  unmapped ROM region %08x.%c",
            this->name.c_str(), offset, SIZE_ARG(size));
        return 0;
    }

    LOG_F(WARNING, "%s: read  unmapped aperture region %08x.%c",
          this->name.c_str(), offset, SIZE_ARG(size));
    return 0;
}

void ATIRage128::write(uint32_t rgn_start, uint32_t offset, uint32_t value, int size)
{
    if (rgn_start == this->aperture_base[0] && offset < this->aperture_size[0]) {
        if (offset < this->vram_size) { // little-endian VRAM region
            draw_fb = true;
            return write_mem(&this->vram_ptr[offset], value, size);
        }
        if (offset >= BE_FB_OFFSET) { // big-endian VRAM region
            draw_fb = true;
            return write_mem(&this->vram_ptr[offset & (BE_FB_OFFSET - 1)], value, size);
        }
        //if (!bit_set(this->regs[ATI_BUS_CNTL], ATI_BUS_APER_REG_DIS)) {
            if (offset >= MM_REGS_0_OFF) { // memory-mapped registers, block 0
                return this->write_reg(offset & 0x3FF, BYTESWAP_SIZED(value, size), size);
            }
            if (offset >= MM_REGS_1_OFF
                //&& bit_set(this->regs[ATI_BUS_CNTL], ATI_BUS_EXT_REG_EN)
            ) { // memory-mapped registers, block 1
                return this->write_reg((offset & 0x3FF) + 0x400,
                                        BYTESWAP_SIZED(value, size), size);
            }
        //}
        LOG_F(WARNING, "%s: write unmapped aperture[0] region %08x.%c = %0*x",
              this->name.c_str(), offset, SIZE_ARG(size), size * 2, value);
        return;
    }

    if (rgn_start == this->aperture_base[2] && offset < this->aperture_size[2]) {
        LOG_F(WARNING, "%s: write unmapped aperture[2] region %08x.%c = %0*x",
              this->name.c_str(), offset, SIZE_ARG(size), size * 2, value);
        return;
    }

    LOG_F(WARNING, "%s: write unmapped aperture region %08x.%c = %0*x",
          this->name.c_str(), offset, SIZE_ARG(size), size * 2, value);
}

void ATIRage128::verbose_pixel_format(int crtc_index) {
    if (crtc_index) {
        LOG_F(ERROR, "CRTC2 not supported yet");
        return;
    }

    uint32_t pix_fmt = extract_bits<uint32_t>(this->regs[ATI_CRTC_GEN_CNTL],
                                              ATI_CRTC_PIX_WIDTH, ATI_CRTC_PIX_WIDTH_size);

    const char* what = "Pixel format:";

    switch (pix_fmt) {
    case 1:
        LOG_F(INFO, "%s 4 bpp with DAC palette", what);
        break;
    case 2:
        // check the undocumented DAC_DIRECT bit
        if (bit_set(this->regs[ATI_DAC_CNTL], ATI_DAC_DIRECT)) {
            LOG_F(INFO, "%s 8 bpp direct color (RGB332)", what);
        } else {
            LOG_F(INFO, "%s 8 bpp with DAC palette", what);
        }
        break;
    case 3:
        LOG_F(INFO, "%s 15 bpp direct color (RGB555)", what);
        break;
    case 4:
        LOG_F(INFO, "%s 16 bpp direct color (RGB565)", what);
        break;
    case 5:
        LOG_F(INFO, "%s 24 bpp direct color (RGB888)", what);
        break;
    case 6:
        LOG_F(INFO, "%s 32 bpp direct color (ARGB8888)", what);
        break;
    default:
        LOG_F(ERROR, "%s: CRTC pixel format %d not supported", this->name.c_str(), pix_fmt);
    }
}

void ATIRage128::crtc_update() {
    uint32_t new_width, new_height, new_htotal, new_vtotal;

    // check for unsupported modes and fail early
    if (!bit_set(this->regs[ATI_CRTC_GEN_CNTL], ATI_CRTC_EXT_DISP_EN))
        ABORT_F("%s: VGA not supported", this->name.c_str());

    bool need_recalc = false;

    new_width  = (extract_bits<uint32_t>(this->regs[ATI_CRTC_H_TOTAL_DISP],
                                         ATI_CRTC_H_DISP,
                                         ATI_CRTC_H_DISP_size) + 1) * 8;
    new_height = extract_bits<uint32_t>(this->regs[ATI_CRTC_V_TOTAL_DISP],
                                        ATI_CRTC_V_DISP, ATI_CRTC_V_DISP_size) + 1;

    if (new_width != this->active_width || new_height != this->active_height) {
        this->create_display_window(new_width, new_height);
        need_recalc = true;
    }

    new_htotal = (extract_bits<uint32_t>(this->regs[ATI_CRTC_H_TOTAL_DISP],
                                         ATI_CRTC_H_TOTAL,
                                         ATI_CRTC_H_TOTAL_size) + 1) * 8;
    new_vtotal = extract_bits<uint32_t>(this->regs[ATI_CRTC_V_TOTAL_DISP],
                                        ATI_CRTC_V_TOTAL, ATI_CRTC_V_TOTAL_size) + 1;

    if (new_htotal != this->hori_total || new_vtotal != this->vert_total) {
        this->hori_total = new_htotal;
        this->vert_total = new_vtotal;
        need_recalc = true;
    }

    uint32_t new_vert_blank = new_vtotal - new_height;
    if (new_vert_blank != this->vert_blank) {
        this->vert_blank = new_vert_blank;
        need_recalc = true;
    }

    int new_pixel_format = extract_bits<uint32_t>(this->regs[ATI_CRTC_GEN_CNTL],
                                                  ATI_CRTC_PIX_WIDTH,
                                                  ATI_CRTC_PIX_WIDTH_size);
    if (new_pixel_format != this->pixel_format) {
        this->pixel_format = new_pixel_format;
        need_recalc = true;
    }

    static uint8_t bits_per_pixel[8] = {0, 4, 8, 16, 16, 24, 32, 0};

    int new_fb_pitch = extract_bits<uint32_t>(this->regs[ATI_CRTC_OFF_PITCH],
        ATI_CRTC_PITCH, ATI_CRTC_PITCH_size) * bits_per_pixel[this->pixel_format];
    if (new_fb_pitch != this->fb_pitch) {
        this->fb_pitch = new_fb_pitch;
        need_recalc = true;
    }
    uint8_t* new_fb_ptr = &this->vram_ptr[extract_bits<uint32_t>(this->regs[ATI_CRTC_OFF_PITCH],
        ATI_CRTC_OFFSET, ATI_CRTC_OFFSET_size) * 8];
    if (new_fb_ptr != this->fb_ptr) {
        this->fb_ptr = new_fb_ptr;
        need_recalc = true;
    }

    if (!need_recalc)
        return;

    this->draw_fb = true;

    // calculate display refresh rate
    this->refresh_rate = pixel_clock / this->hori_total / this->vert_total;

    if (this->refresh_rate < 24 || this->refresh_rate > 120) {
        LOG_F(ERROR, "%s: Refresh rate is weird. Will try 60 Hz", this->name.c_str());
        this->refresh_rate = 60;
        this->pixel_clock = this->refresh_rate * this->hori_total / this->vert_total;
    }

    // set up frame buffer converter
    switch (this->pixel_format) {
    case 1:
        this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
            draw_fb = false;
            this->convert_frame_4bpp_indexed(dst_buf, dst_pitch);
        };
        break;
    case 2:
        if (bit_set(this->regs[ATI_DAC_CNTL], ATI_DAC_DIRECT)) {
            this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
                draw_fb = false;
                this->convert_frame_8bpp(dst_buf, dst_pitch);
            };
        }
        else {
            this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
                draw_fb = false;
                this->convert_frame_8bpp_indexed(dst_buf, dst_pitch);
            };
        }
        break;
    case 3:
        this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
            draw_fb = false;
            this->convert_frame_15bpp_BE(dst_buf, dst_pitch);
        };
        break;
    case 4:
        this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
            draw_fb = false;
            this->convert_frame_16bpp(dst_buf, dst_pitch);
        };
        break;
    case 5:
        this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
            draw_fb = false;
            this->convert_frame_24bpp(dst_buf, dst_pitch);
        };
        break;
    case 6:
        this->convert_fb_cb = [this](uint8_t *dst_buf, int dst_pitch) {
            draw_fb = false;
            this->convert_frame_32bpp_BE(dst_buf, dst_pitch);
        };
        break;
    default:
        LOG_F(ERROR, "%s: unsupported pixel format %d", this->name.c_str(), this->pixel_format);
    }

    LOG_F(INFO, "%s: primary CRT controller enabled:", this->name.c_str());
    LOG_F(INFO, "Video mode: %s",
         bit_set(this->regs[ATI_CRTC_GEN_CNTL], ATI_CRTC_EXT_DISP_EN) ? "extended" : "VGA");
    LOG_F(INFO, "Video width: %d px", this->active_width);
    LOG_F(INFO, "Video height: %d px", this->active_height);
    verbose_pixel_format(0);
    LOG_F(INFO, "Pixel (dot) clock: %f MHz", this->pixel_clock * 1e-6);
    LOG_F(INFO, "Refresh rate: %f Hz", this->refresh_rate);

    this->stop_refresh_task();
    this->start_refresh_task();

    this->crtc_on = true;
}

static const PropMap AtiRage128GL_Properties = {
    {"gfxmem_size",
        new IntProperty(  16, vector<uint32_t>({16}))},
    {"mon_id",
        new StrProperty("")},
};

static const DeviceDescription AtiRage128GL_Descriptor = {
    ATIRage128::create_gl, {}, AtiRage128GL_Properties
};

REGISTER_DEVICE(AtiRage128GL, AtiRage128GL_Descriptor);