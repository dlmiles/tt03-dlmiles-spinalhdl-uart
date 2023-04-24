#!/bin/bash
#
#	This is safe to run everytime, it only edits the file if it needs editing.
#
#$ diff -u scripts/tcl_commands/synthesis.tcl~ scripts/tcl_commands/synthesis.tcl
#--- scripts/tcl_commands/synthesis.tcl~	2023-04-26 20:34:07.733672394 +0100
#+++ scripts/tcl_commands/synthesis.tcl	2023-04-26 20:36:07.737672468 +0100
#@@ -270,7 +270,7 @@
#     set arg "|& tee $log $::env(TERMINAL_OUTPUT)"
#     lappend arg_list {*}$arg
#     try_exec bash -c "verilator \
#-        --lint-only \
#+        --lint-only --no-timing +define+SYNTHESIS=1 +define+SYNTHESIS_VERILATOR_LINT_ONLY=1 \
#         -Wall \
#         --Wno-DECLFILENAME \
#         --top-module $::env(DESIGN_NAME) \
#
#

if [ ! -f scripts/tcl_commands/synthesis.tcl ]
then
	echo "$0: ERROR: scripts/tcl_commands/synthesis.tcl file not found: pwd=$(pwd)" 1>&2
	exit 1
fi

if fgrep -q -- "--lint-only \\" scripts/tcl_commands/synthesis.tcl
then
	cp -f scripts/tcl_commands/synthesis.tcl /tmp/synthesis$$.py
	if [ "$1" != "-nobackup" ]
	then
		cp -f scripts/tcl_commands/synthesis.tcl scripts/tcl_commands/synthesis.tcl~
	fi
	sed -e 's#--lint-only \\#--lint-only --no-timing +define+SYNTHESIS=1 +define+SYNTHESIS_VERILATOR_LINT_ONLY=1 \\#' -i scripts/tcl_commands/synthesis.tcl
	echo "EDITED: scripts/tcl_commands/synthesis.tcl"
	diff -u /tmp/synthesis$$.py scripts/tcl_commands/synthesis.tcl
	rm -f /tmp/synthesis$$.py
fi

exit 0
