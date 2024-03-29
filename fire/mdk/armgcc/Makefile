PROJECT_NAME     := rtc_pca10056
TARGETS          := nrf52840_xxaa
OUTPUT_DIRECTORY := _build

MDK_ROOT := ../../../../..
SDK_ROOT := $(MDK_ROOT)/nrf_sdks/nRF5_SDK_17.0.2_d674dde
PROJ_DIR := ../..

$(OUTPUT_DIRECTORY)/nrf52840_xxaa.out: \
  LINKER_SCRIPT  := rtc_gcc_nrf52.ld

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52840.S \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_rtt.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_default_backends.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_str_formatter.c \
  $(SDK_ROOT)/components/boards/boards.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/atomic/nrf_atomic.c \
  $(SDK_ROOT)/components/libraries/atomic_flags/nrf_atflags.c \
  $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer2.c \
  $(SDK_ROOT)/components/libraries/atomic_fifo/nrf_atfifo.c \
  $(SDK_ROOT)/components/libraries/sortlist/nrf_sortlist.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf_format.c \
  $(SDK_ROOT)/components/libraries/gfx/nrf_gfx.c \
  $(SDK_ROOT)/components/libraries/memobj/nrf_memobj.c \
  $(SDK_ROOT)/components/libraries/ringbuf/nrf_ringbuf.c \
  $(SDK_ROOT)/components/libraries/experimental_section_vars/nrf_section_iter.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_clock.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_spi.c \
  $(SDK_ROOT)/components/ble/peer_manager/auth_status_tracker.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_params.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_state.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt/nrf_ble_gatt.c \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr/nrf_ble_qwr.c \
  $(SDK_ROOT)/modules/nrfx/soc/nrfx_atomic.c \
  $(SDK_ROOT)/components/libraries/timer/drv_rtc.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_clock.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_rtc.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_gpiote.c \
  $(PROJ_DIR)/main.c \
  $(PROJ_DIR)/graphics/fire.c \
  $(PROJ_DIR)/graphics/garage.c \
  $(PROJ_DIR)/ble_temp.c \
  $(PROJ_DIR)/peripherals/st7735/st7735.c \
  $(PROJ_DIR)/peripherals/max31865/max31865.c \
  $(PROJ_DIR)/lvgl/src/./lv_core/lv_obj.c \
  $(PROJ_DIR)/lvgl/src/./lv_core/lv_refr.c \
  $(PROJ_DIR)/lvgl/src/./lv_core/lv_style.c \
  $(PROJ_DIR)/lvgl/src/./lv_core/lv_group.c \
  $(PROJ_DIR)/lvgl/src/./lv_core/lv_indev.c \
  $(PROJ_DIR)/lvgl/src/./lv_core/lv_disp.c \
  $(PROJ_DIR)/lvgl/src/./lv_hal/lv_hal_tick.c \
  $(PROJ_DIR)/lvgl/src/./lv_hal/lv_hal_disp.c \
  $(PROJ_DIR)/lvgl/src/./lv_hal/lv_hal_indev.c \
  $(PROJ_DIR)/lvgl/src/./lv_draw/lv_img_decoder.c \
  $(PROJ_DIR)/lvgl/src/./lv_draw/lv_img_cache.c \
  $(PROJ_DIR)/lvgl/src/./lv_draw/lv_img_buf.c \
  $(PROJ_DIR)/lvgl/src/./lv_draw/lv_draw_img.c \
  $(PROJ_DIR)/lvgl/src/./lv_draw/lv_draw_rect.c \
  $(PROJ_DIR)/lvgl/src/./lv_draw/lv_draw_mask.c \
  $(PROJ_DIR)/lvgl/src/./lv_draw/lv_draw_blend.c \
  $(PROJ_DIR)/lvgl/src/./lv_draw/lv_draw_label.c \
  $(PROJ_DIR)/lvgl/src/./lv_draw/lv_draw_line.c \
  $(PROJ_DIR)/lvgl/src/./lv_misc/lv_task.c \
  $(PROJ_DIR)/lvgl/src/./lv_misc/lv_mem.c \
  $(PROJ_DIR)/lvgl/src/./lv_misc/lv_fs.c \
  $(PROJ_DIR)/lvgl/src/./lv_misc/lv_anim.c \
  $(PROJ_DIR)/lvgl/src/./lv_misc/lv_ll.c \
  $(PROJ_DIR)/lvgl/src/./lv_misc/lv_task.c \
  $(PROJ_DIR)/lvgl/src/./lv_misc/lv_debug.c \
  $(PROJ_DIR)/lvgl/src/./lv_misc/lv_color.c \
  $(PROJ_DIR)/lvgl/src/./lv_misc/lv_gc.c \
  $(PROJ_DIR)/lvgl/src/./lv_misc/lv_area.c \
  $(PROJ_DIR)/lvgl/src/./lv_misc/lv_utils.c \
  $(PROJ_DIR)/lvgl/src/./lv_misc/lv_txt.c \
  $(PROJ_DIR)/lvgl/src/./lv_misc/lv_math.c \
  $(PROJ_DIR)/lvgl/src/./lv_font/lv_font.c \
  $(PROJ_DIR)/lvgl/src/./lv_font/lv_font_fmt_txt.c \
  $(PROJ_DIR)/lvgl/src/./lv_font/lv_font_montserrat_14.c \
  $(PROJ_DIR)/lvgl/src/./lv_font/lv_font_montserrat_36.c \
  $(PROJ_DIR)/lvgl/src/./lv_themes/lv_theme.c \
  $(PROJ_DIR)/lvgl/src/./lv_themes/lv_theme_material.c \
  $(PROJ_DIR)/lvgl/src/./lv_widgets/lv_label.c \
  $(PROJ_DIR)/lvgl/src/./lv_widgets/lv_btn.c \
  $(PROJ_DIR)/lvgl/src/./lv_widgets/lv_cont.c \
  $(PROJ_DIR)/lvgl/src/./lv_widgets/lv_img.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
	$(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c \
	$(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
  $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52840.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ble.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_soc.c \
  $(SDK_ROOT)/components/libraries/button/app_button.c \
  $(SDK_ROOT)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c \
	$(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_timer.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_spi.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_spim.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus/ble_nus.c \
  $(SDK_ROOT)/components/ble/ble_link_ctx_manager/ble_link_ctx_manager.c \
  $(SDK_ROOT)/components/ble/ble_advertising/ble_advertising.c \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs/ble_lbs.c \
  

# Include folders common to all targets
INC_FOLDERS += \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/ble/ble_advertising \
  $(SDK_ROOT)/modules/nrfx/mdk \
  $(PROJ_DIR) \
  $(PROJ_DIR)/peripherals \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/softdevice/common \
  ../config \
  $(SDK_ROOT)/components/libraries/gfx \
  $(SDK_ROOT)/components/libraries/balloc \
  $(SDK_ROOT)/components/libraries/ringbuf \
  $(SDK_ROOT)/modules/nrfx/hal \
  $(SDK_ROOT)/components/libraries/bsp \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/ble/ble_services/ble_hts \
  $(SDK_ROOT)/components/ble/ble_services/ble_lbs \
  $(SDK_ROOT)/components/libraries/cli/ble_uart \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/ble/ble_link_ctx_manager \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/components/ble/peer_manager \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt \
  $(SDK_ROOT)/components/ble/nrf_ble_qwr \
  $(SDK_ROOT)/components/softdevice/s140/headers/nrf52 \
  $(SDK_ROOT)/components/softdevice/s140/headers \
  $(SDK_ROOT)/modules/nrfx \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/libraries/fds \
  $(SDK_ROOT)/integration/nrfx/legacy \
  $(SDK_ROOT)/integration/nrfx \
  $(SDK_ROOT)/components/drivers_nrf/nrf_soc_nosd \
  $(SDK_ROOT)/components/libraries/atomic \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/libraries/memobj \
  $(SDK_ROOT)/components/drivers_ext/st7735 \
  $(SDK_ROOT)/components/libraries/delay \
  $(SDK_ROOT)/modules/nrfx/drivers/include \
  $(SDK_ROOT)/components/libraries/atomic_flags \
  $(SDK_ROOT)/external/fprintf \
  $(SDK_ROOT)/external/segger_rtt \
  $(SDK_ROOT)/components/libraries/log/src \
  $(SDK_ROOT)/components/libraries/atomic_fifo \
  $(SDK_ROOT)/components/libraries/sortlist \
  $(SDK_ROOT)/components/libraries/button \
  $(SDK_ROOT)/components/libraries/pwr_mgmt \
  $(SDK_ROOT)/components/libraries/mutex \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus_c \
  $(SDK_ROOT)/components/ble/ble_services/ble_nus \
  $(SDK_ROOT)/components/ble/ble_link_ctx_manager \
  $(SDK_ROOT)/external/thedotfactory_fonts \

# Libraries common to all targets
LIB_FILES += \

# Optimization flags
# OPT = -O3 -g3
OPT = -ggdb
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DAPP_TIMER_V2
CFLAGS += -DAPP_TIMER_V2_RTC1_ENABLED
CFLAGS += -DBOARD_CUSTOM
CFLAGS += -DNRF52840_MDK_USB_DONGLE
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DNRF52840_XXAA
CFLAGS += -DNRF_SD_BLE_API_VERSION=7
CFLAGS += -DS140
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums

# C++ flags common to all targets
CXXFLAGS += $(OPT)
# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DAPP_TIMER_V2
ASMFLAGS += -DAPP_TIMER_V2_RTC1_ENABLED
ASMFLAGS += -DBOARD_CUSTOM
ASMFLAGS += -DNRF52840_MDK_USB_DONGLE
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=7
ASMFLAGS += -DS140
ASMFLAGS += -DSOFTDEVICE_PRESENT

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS += -u _printf_float
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs

nrf52840_xxaa: CFLAGS += -D__HEAP_SIZE=8192
nrf52840_xxaa: CFLAGS += -D__STACK_SIZE=8192
nrf52840_xxaa: ASMFLAGS += -D__HEAP_SIZE=8192
nrf52840_xxaa: ASMFLAGS += -D__STACK_SIZE=8192

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm


.PHONY: default help

# Default target - first one defined
default: nrf52840_xxaa

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		nrf52840_xxaa
	@echo		sdk_config - starting external tool for editing sdk_config.h
	@echo		flash      - flashing binary

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc


include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash flash_softdevice erase

# Flash the program
flash: default
	@echo Flashing: $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex
	nrfjprog -f nrf52 --program $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex --sectorerase
	nrfjprog -f nrf52 --reset

# Flash softdevice
flash_softdevice:
	@echo Flashing: s140_nrf52_7.0.1_softdevice.hex
	nrfjprog -f nrf52 --program $(SDK_ROOT)/components/softdevice/s140/hex/s140_nrf52_7.0.1_softdevice.hex --sectorerase
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall

SDK_CONFIG_FILE := $(PROJ_DIR)/config/sdk_config.h
CMSIS_CONFIG_TOOL := $(SDK_ROOT)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar
sdk_config:
	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)
