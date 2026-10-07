# ==============================================================================
#  HSPS ROBOTIC ARM - FIRMWARE BLADE MAKEFILE
# ==============================================================================

# --- Project Configuration ---
BUILD_DIR = build
PROJECT_NAME = blade_firmware
OPENOCD_INTERFACE = interface/stlink.cfg
OPENOCD_TARGET = target/stm32g4x.cfg

# Default Build Type (Release or Debug)
# Override with: make config TYPE=Debug
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
.PHONY: help all config build flash clean

# Default target: Show help
help:
	@echo -e ""
	@echo -e "$(CC)#################################################################$(CN)"
	@echo -e "$(CC)#               HSPS ROBOTIC ARM - MCU CONTROLLER               #$(CN)"
	@echo -e "$(CC)#                     STM32G4 Bare Metal                        #$(CN)"
	@echo -e "$(CC)#################################################################$(CN)"
	@echo -e ""
	@echo -e "$(CY)Current Build Type: $(CB)$(TYPE)$(CN)"
	@echo -e ""
	@echo -e "$(CY)Available commands:$(CN)"
	@echo -e "  $(CG)make config$(CN)   : Clean build dir and configure CMake (Default: Release)."
	@echo -e "                            Usage: make config TYPE=Debug"
	@echo -e "  $(CG)make build$(CN)    : Compile the project."
	@echo -e "  $(CG)make flash$(CN)    : Flash firmware via OpenOCD."
	@echo -e "  $(CG)make all$(CN)      : Full sequence: Config -> Build -> Flash."
	@echo -e "  $(CG)make clean$(CN)    : Remove the $(BUILD_DIR) directory."
	@echo -e ""

# Full cycle
all: config build flash

# 1. Configuration Step
config:
	@echo -e ""
	@echo -e "$(CB)[HSPS] Cleaning previous build...$(CN)"
	@rm -rf $(BUILD_DIR)
	@echo -e "$(CB)[HSPS] Configuring CMake as $(TYPE)...$(CN)"
	@cmake -B $(BUILD_DIR) -S . -DCMAKE_BUILD_TYPE=$(TYPE)
	@echo -e "$(CG)[HSPS] Configuration Done.$(CN)"

# 2. Compilation Step
build:
	@echo -e ""
	@echo -e "$(CB)[HSPS] Compiling firmware...$(CN)"
	@make -C $(BUILD_DIR) -j$(shell nproc) --no-print-directory
	@echo -e "$(CG)[HSPS] Build Complete.$(CN)"

# 3. Flashing Step
flash:
	@echo -e ""
	@echo -e "$(CM)[HSPS] Flashing Target...$(CN)"
	@sudo openocd -f $(OPENOCD_INTERFACE) \
		-f $(OPENOCD_TARGET) \
		-c "program $(BUILD_DIR)/$(PROJECT_NAME).elf verify reset exit"
	@echo -e "$(CG)[HSPS] Flashing Done. ADC is alive.$(CN)"

# Utility
clean:
	@echo -e "$(CR)[HSPS] Removing build directory...$(CN)"
	@rm -rf $(BUILD_DIR)