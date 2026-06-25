#!/bin/sh
status=0

for report in "$@"; do
    if [ ! -f "$report" ]; then
        echo "MISSING $report"
        status=1
        continue
    fi

    uncovered_lines=$(grep -n '^[[:space:]]*#####:' "$report" || true)
    uncovered_branches=$(grep -n -E 'branch[[:space:]]+[0-9]+[[:space:]]+(taken 0|never executed)' "$report" || true)

    if [ -n "$uncovered_lines" ] || [ -n "$uncovered_branches" ]; then
        echo "FAIL $report"
        if [ -n "$uncovered_lines" ]; then
            echo "$uncovered_lines"
        fi
        if [ -n "$uncovered_branches" ]; then
            echo "$uncovered_branches"
        fi
        status=1
    else
        echo "PASS $report: 100% statements and branches"
    fi
done

exit "$status"
