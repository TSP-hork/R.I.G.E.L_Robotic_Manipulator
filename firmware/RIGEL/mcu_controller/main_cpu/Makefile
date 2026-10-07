# ==============================================================================
#  HSPS ROBOTIC ARM - MAIN CONTROLLER MAKEFILE
#  STM32H7 Bare Metal
# ==============================================================================

# --- Project Configuration ---
BUILD_DIR          = build
PROJECT_NAME       = firmware
OPENOCD_INTERFACE  = interface/stlink.cfg
OPENOCD_TARGET     = target/stm32h7x.cfg

# Default Build Type (Release or Debug)
# Override with: make build TYPE=Debug
TYPE ?= Release

# --- Colors for beautiful output ---
CN  = \033[0m
CR  = \033[91m
CG  = \033[92m
CY  = \033[93m
CB  = \033[94m
CM  = \033[95m
CC  = \033[96m

# --- Main Targets ---
.PHONY: help all build flash clean

# Default target: Show help
help:
	@echo -e ""
	@echo -e "$(CC)#################################################################$(CN)"
	@echo -e "$(CC)#               HSPS ROBOTIC ARM - MAIN CONTROLLER              #$(CN)"
	@echo -e "$(CC)#                     STM32H7 Bare Metal                        #$(CN)"
	@echo -e "$(CC)#################################################################$(CN)"
	@echo -e ""
	@echo -e "$(CY)Current Build Type: $(CB)$(TYPE)$(CN)"
	@echo -e ""
	@echo -e "$(CY)Available commands:$(CN)"
	@echo -e "  $(CG)make build$(CN)    : Create build dir, configure CMake, compile."
	@echo -e "                            Usage: make build TYPE=Debug"
	@echo -e "  $(CG)make flash$(CN)    : Flash firmware via OpenOCD."
	@echo -e "  $(CG)make all$(CN)      : Full sequence: Clean -> Build -> Flash."
	@echo -e "  $(CG)make clean$(CN)    : Remove the $(BUILD_DIR) directory."
	@echo -e ""

# Full cycle: clean slate, build from scratch, flash to target
all: clean build flash

# Build: create directory, run CMake configure, compile
build:
	@echo -e ""
	@echo -e "$(CB)[HSPS] Creating build directory...$(CN)"
	@mkdir -p $(BUILD_DIR)
	@echo -e "$(CB)[HSPS] Configuring CMake as $(TYPE)...$(CN)"
	@cmake -B $(BUILD_DIR) -S . -DCMAKE_BUILD_TYPE=$(TYPE)
	@echo -e "$(CB)[HSPS] Compiling firmware...$(CN)"
	@make -C $(BUILD_DIR) -j$(shell nproc) --no-print-directory
	@echo -e "$(CG)[HSPS] Build Complete.$(CN)"

# Flash: program the MCU via ST-Link + OpenOCD
flash:
	@echo -e ""
	@echo -e "$(CM)[HSPS] Flashing Target (STM32H7)...$(CN)"
	@sudo openocd -f $(OPENOCD_INTERFACE) \
		-f $(OPENOCD_TARGET) \
		-c "program $(BUILD_DIR)/$(PROJECT_NAME).elf verify reset exit"
	@echo -e "$(CG)[HSPS] Flashing Done. FOC controller is alive.$(CN)"

# Clean: remove the entire build directory
clean:
	@echo -e "$(CR)[HSPS] Removing build directory...$(CN)"
	@rm -rf $(BUILD_DIR)