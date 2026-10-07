#!/usr/bin/env bash
# Every regression test in this bundle. Each one encodes a fault that
# actually happened in flight, so a failure here is a repeat of a bad day.
cd "$(dirname "${BASH_SOURCE[0]}")"
fail=0
for t in test_*.py; do
    if python3 "$t" >/tmp/fcb_test_out 2>&1; then
        echo "  PASS  $t"
    else
        echo "  FAIL  $t"; sed 's/^/        /' /tmp/fcb_test_out | tail -15; fail=1
    fi
done
exit $fail
