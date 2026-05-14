"""Budget tracker CLI — summarizes Modal spend and alerts against the cap.

Each Modal account has $30 credit. We alert at $25 spent (room to wind
down) and hard-stop at $28 (force switch to the secondary account before
hitting $30 and being charged real money).

Usage:
    python -m wdt_modal.budget
    python wdt_modal/budget.py

Exit codes:
    0 — under alert
    1 — over alert, under hard stop
    2 — over hard stop, switch accounts NOW
"""

from __future__ import annotations

import json
import subprocess
import sys

BUDGET_ALERT = 25.00
BUDGET_HARD_STOP = 28.00


def _billing(for_range: str) -> list[dict]:
    """Return the parsed JSON list from `modal billing report --for <range>`."""
    raw = subprocess.run(
        ["modal", "billing", "report", "--for", for_range, "--json"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    return json.loads(raw)


def _sum_cost(rows: list[dict]) -> float:
    return sum(float(r.get("Cost", "0") or "0") for r in rows)


def main() -> int:
    month_total = _sum_cost(_billing("this month"))
    today_total = _sum_cost(_billing("today"))

    print(f"Modal spend this month: ${month_total:.2f}  (today: ${today_total:.2f})")
    print(f"Alert: ${BUDGET_ALERT:.2f}  Hard stop: ${BUDGET_HARD_STOP:.2f}")

    if month_total >= BUDGET_HARD_STOP:
        print("HARD STOP — switch to secondary Modal account before hitting $30.")
        return 2
    if month_total >= BUDGET_ALERT:
        print("ALERT — approaching cap, wind down non-critical runs.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
