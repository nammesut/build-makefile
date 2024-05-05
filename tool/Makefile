PRO_DIR 	:= .
PATH_OUTPUT = ../output
PROJ_NAME 	:= gpio

INC_DIRS := $(PRO_DIR)/Inc
INC_DIRS += $(PRO_DIR)/../driver/gpio/Inc

SRC_DIRS := $(PRO_DIR)/Src
SRC_DIRS += $(PRO_DIR)/../driver/gpio/Src

INC_FILES := $(foreach INC_DIRS,$(INC_DIRS),$(wildcard $(INC_DIRS)/*))
SRC_FILES := $(foreach SRC_DIRS,$(SRC_DIRS),$(wildcard $(SRC_DIRS)/*))

COMPILER_DIR 	:= D:/SoftWare/GNUArm
LINKER_FILE		:= $(PRO_DIR)/linker/stm32f1xx_ls.ld

CC	:= $(COMPILER_DIR)/bin/arm-none-eabi-gcc
LD	:= $(COMPILER_DIR)/bin/arm-none-eabi-ld

INC_DIRS_OPT := $(foreach INC_DIRS,$(INC_DIRS),-I$(INC_DIRS))

CC_OPT 	:= -mcpu=cortex-m3 -c -O0 -g -mfloat-abi=soft -std=gnu11 -mthumb $(INC_DIRS_OPT)
LD_OPT	:=   -T $(LINKER_FILE) -Map $(PATH_OUTPUT)/$(PROJ_NAME).map

OBJ_FILES := $(notdir $(SRC_FILES))
OBJ_FILES := $(subst .c,.o,$(OBJ_FILES))
PATH_OBJS := $(foreach OBJ_FILES,$(OBJ_FILES),$(PATH_OUTPUT)/$(OBJ_FILES))

vpath %.c $(SRC_DIRS)
vpath %.h $(INC_DIRS)

build: $(OBJ_FILES) $(LINKER_FILE)
	$(LD) $(LD_OPT) $(PATH_OBJS) -o $(PATH_OUTPUT)/$(PROJ_NAME).elf
	$(COMPILER_DIR)/arm-none-eabi/bin/objcopy.exe -O ihex "$(PATH_OUTPUT)/$(PROJ_NAME).elf" "$(PATH_OUTPUT)/$(PROJ_NAME).hex"
	size $(PATH_OUTPUT)/$(PROJ_NAME).elf
	@echo "---------------- Build OK ----------------"

%.o: %.c $(INC_FILES)
	mkdir -p $(PATH_OUTPUT)
	$(CC) $(CC_OPT) -c $< -o $(PATH_OUTPUT)/$@
	
run:
	$(PRO_DIR)/ST_LINK/ST-LINK_CLI.exe -p "$(PATH_OUTPUT)/$(PROJ_NAME).hex" 0x08000000
	$(PRO_DIR)/ST_LINK/ST-LINK_CLI.exe -rst

clean:
	rm -rf $(PATH_OUTPUT)


