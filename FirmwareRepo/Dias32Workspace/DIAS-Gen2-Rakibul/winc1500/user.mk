#Userlib directories
USERLIB = ./winc1500

# List of all the Userlib files
USERSRC =  $(USERLIB)/bsp/source/nm_bsp_stm32f429.c \
			$(USERLIB)/bus_wrapper/source/nm_bus_wrapper_stm32f429.c \
			$(USERLIB)/common/source/nm_common.c \
			$(USERLIB)/driver/source/m2m_ate_mode.c \
			$(USERLIB)/driver/source/m2m_crypto.c \
			$(USERLIB)/driver/source/m2m_hif.c \
			$(USERLIB)/driver/source/m2m_ota.c \
			$(USERLIB)/driver/source/m2m_periph.c \
			$(USERLIB)/driver/source/m2m_ssl.c \
			$(USERLIB)/driver/source/m2m_wifi.c \
			$(USERLIB)/driver/source/nmasic.c \
			$(USERLIB)/driver/source/nmbus.c \
			$(USERLIB)/driver/source/nmdrv.c \
			$(USERLIB)/driver/source/nmi2c.c \
			$(USERLIB)/driver/source/nmspi.c \
			$(USERLIB)/driver/source/nmuart.c \
			$(USERLIB)/socket/source/socket.c \
			$(USERLIB)/spi_flash/source/spi_flash.c \
			
          
# Required include directories
USERINC =  $(USERLIB)/bsp/include \
			$(USERLIB)/bus_wrapper/include \
			$(USERLIB)/common/include \
			$(USERLIB)/config \
			$(USERLIB)/driver/include \
			$(USERLIB)/driver/source \
			$(USERLIB)/socket/include \
			$(USERLIB)/socket/source \
			$(USERLIB)/spi_flash/include \


# Shared variables
ALLCSRC += $(USERSRC)
ALLINC  += $(USERINC)