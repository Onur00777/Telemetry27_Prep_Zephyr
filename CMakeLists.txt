cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(telemetry27)

target_sources(app PRIVATE
	src/main.c
	src/platform.c
	src/telemetry.c
	src/telemetry_buffer.c
	src/telemetry_pack.c
	src/can_decode.c
	src/can_bus.c
	src/modem_fsm.c
	src/modem_at.c
	src/modem_parser.c
	src/modem_uart.c
	src/modem_power.c
)

target_sources_ifdef(CONFIG_TELEMETRY_SIM app PRIVATE
	src/modem_port_sim.c
	src/sim_vehicle.c
)

target_sources_ifndef(CONFIG_TELEMETRY_SIM app PRIVATE
	src/modem_port_hw.c
)
