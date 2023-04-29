# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info

# defaults
SIM ?= icarus
TOPLEVEL_LANG ?= verilog

# normal simulation
ifneq ($(GATES),yes)

# this is the only part you should need to modify:
VERILOG_SOURCES += $(PWD)/tb_uart.v
VERILOG_SOURCES += $(PWD)/top_tt03_dlmiles_spinalhdl_uart.v
VERILOG_SOURCES += $(PWD)/dff.v $(PWD)/dffqn_negedge.v $(PWD)/clock_stretch_invert_mux.v $(PWD)/glitch_free_clock_mux.v
#VERILOG_SOURCES += $(PWD)/async_reset_ctrl.v $(PWD)/sr_latch_nand.v
VERILOG_SOURCES += $(PWD)/async_reset_ctrl__dff_async_set.v $(PWD)/dff_async_set.v
VERILOG_SOURCES += $(PWD)/inverter_reg_ladder.v
VERILOG_SOURCES += $(PWD)/uart_dummy.v
VERILOG_SOURCES += $(PWD)/Uart.v

else

# gate level simulation requires some extra setup, you shouldn't need to touch this
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DUSE_POWER_PINS
COMPILE_ARGS    += -DSIM
COMPILE_ARGS    += -DUNIT_DELAY=\#1
VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/primitives.v
VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/sky130_fd_sc_hd.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/tb_uart.v $(PWD)/gate_level_netlist.v
endif

# TOPLEVEL is the name of the toplevel module in your Verilog or VHDL file
TOPLEVEL = tb_uart

# MODULE is the basename of the Python test file
MODULE = test_uart

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
