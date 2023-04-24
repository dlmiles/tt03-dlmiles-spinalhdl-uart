#!/bin/bash
#
#	This is safe to run everytime, it only edits the file if it need editing.
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
