#!/usr/bin/env python3
"""
Budget totals from docs/bom.csv, in USD and CAD.

    python3 tools/bom_total.py            # summary
    python3 tools/bom_total.py --windows  # spend per purchase window

Tick items off by editing the `status` column in docs/bom.csv:
    need        -> counted (you still have to buy it)
    have        -> not counted (you own it)
    makerspace  -> not counted (borrow / free)
    bought      -> counted as spent (keeps the total honest)
Exchange rate and tax rates below are estimates. Update when you order.
"""
import csv
import sys
from collections import defaultdict
from pathlib import Path

USD_TO_CAD = 1.40      # Sept 2026 is ~1.38-1.40
BUDGET_USD = 1000.0    # tools are tracked separately
CONTINGENCY = 0.10     # failed parts, shipping, duties surprises

# Estimated tax/fees by source (fraction of item cost).
# Ali/JLC: GST 5% + occasional duty/brokerage. Amazon.ca / local: GST + BC PST = 12%.
TAX = {"ali": 0.07, "jlc": 0.10, "amazon": 0.12, "local": 0.12}
COUNTED = {"need", "bought"}

ROOT = Path(__file__).resolve().parent.parent


def cad(row):
    price = float(row["unit_price"]) * int(row["qty"])
    if row["currency"] == "USD":
        price *= USD_TO_CAD
    return price * (1 + TAX[row["source"]])


def main():
    rows = [r for r in csv.DictReader(open(ROOT / "docs" / "bom.csv"))
            if r["status"] in COUNTED]
    by_scope = defaultdict(float)
    by_cat = defaultdict(float)
    by_window = defaultdict(float)
    spent = 0.0
    for r in rows:
        c = cad(r)
        by_scope[r["scope"]] += c
        if r["scope"] == "core":
            by_cat[r["category"]] += c
        by_window[r["buy_window"]] += c
        if r["status"] == "bought":
            spent += c

    def fmt(v):
        return f"{v:>7.0f} CAD {v / USD_TO_CAD:>6.0f} USD"

    if "--windows" in sys.argv:
        print("Spend by purchase window (everything still counted):")
        for w, v in sorted(by_window.items(), key=lambda kv: -kv[1]):
            print(f"  {w:<14}{fmt(v)}")
        return

    print("Core build by category (incl. est. tax):")
    for cat, v in sorted(by_cat.items(), key=lambda kv: -kv[1]):
        print(f"  {cat:<14}{fmt(v)}")
    core = by_scope["core"]
    print(f"  {'-' * 34}\n  {'core':<14}{fmt(core)}")
    with_cont = core * (1 + CONTINGENCY)
    print(f"  {'+10% cont.':<14}{fmt(with_cont)}   budget {BUDGET_USD:.0f} USD")
    print("\nOutside the core:")
    for scope in ("upgrade", "later", "tools"):
        print(f"  {scope:<14}{fmt(by_scope[scope])}")
    left = BUDGET_USD * USD_TO_CAD - with_cont
    print(f"\nBudget left after core + contingency: {fmt(left)}")
    print(f"Already spent (status=bought):        {fmt(spent)}")


if __name__ == "__main__":
    main()
