#!/bin/bash
#
#	This is safe to run everytime, it only edits the file if it needs editing.
#
#	This was fixed in 2023.04.28 with:
#		PR: https://github.com/The-OpenROAD-Project/OpenLane/pull/1775
#		LOG: https://github.com/The-OpenROAD-Project/OpenLane/issues/1777
#		Fix: https://github.com/The-OpenROAD-Project/OpenLane/commit/4ced43c5f349a30e6dd1c4cf520be211b962bde8
#
#	Affected versions that still need this patch:
#		2023.03.28
#		2023.03.29
#		2023.03.30
#		2023.04.05
#		2023.04.06
#		2023.04.07
#		2023.04.11
#		2023.04.12
#		2023.04.14
#		2023.04.17
#		2023.04.18
#		2023.04.19
#		2023.04.20
#		2023.04.27
#
#
#$ diff -u scripts/report/report.py~ scripts/report/report.py
#--- scripts/report/report.py~	2023-04-26 18:58:28.000000000 +0100
#+++ scripts/report/report.py	2023-04-26 19:35:37.091606916 +0100
#@@ -120,7 +120,7 @@
#         self.run_path = run_path
#         self.configuration = params.values()
#         self.configuration_full = ConfigHandler.get_config_for_run(
#-            None, design_path, tag, full=True
#+            run_path, design_path, tag, full=True
#         )
#         self.raw_report = None
#         self.formatted_report = None
#
#

if [ ! -f scripts/report/report.py ]
then
	echo "$0: ERROR: scripts/report/report.py file not found: pwd=$(pwd)" 1>&2
	exit 1
fi

if fgrep -q "None, design_path, tag, full=True" scripts/report/report.py
then
	cp -f scripts/report/report.py /tmp/report$$.py
	if [ "$1" != "-nobackup" ]
	then
		cp -f scripts/report/report.py scripts/report/report.py~
	fi
	sed -e 's#None, design_path, tag, full=True#run_path, design_path, tag, full=True#' -i scripts/report/report.py
	echo "EDITED: scripts/report/report.py"
	diff -u /tmp/report$$.py scripts/report/report.py
	rm -f /tmp/report$$.py
fi

exit 0
