#!/usr/bin/env python3
"""Writes the reference files in tests/data.

statistics_reference.txt — mpmath at 20 digits: the chi-square quantile
1 − Pfa (regularised incomplete gamma) and the noncentrality λ at which the
noncentral chi-square leaves Pmd below that quantile, its distribution
function integrated from the Bessel-function density, a method other than
the Poisson mixture of the crate. Columns: dof pfa pmd threshold_squared
lambda.

raim_reference.txt — numpy: weighted least squares over real skies (the
used satellites of NAV-SAT records of u-blox receivers in kinavis-ubx's
reference), elevation-weighted σ = 0.5 m / sin(el), el ≥ 5°; clock per
constellation; the test statistic of the reported residuals and the
protection levels for Pfa = 1e-3, Pmd = 1e-3.
Columns: sky index, then per satellite gnss,sv,elev,azim,residual; then
'=> statistic threshold hpl vpl'.

    python3 -I tools/reference.py [statistics] [skies]

Requires numpy and mpmath.
"""

import math
import sys

import mpmath as mp
import numpy as np

mp.mp.dps = 20

PFA = [1e-2, 1e-3, 1e-5, 1e-7]
PMD = [1e-2, 1e-3, 1e-7]


def chi2_cdf(x, k):
    return mp.gammainc(mp.mpf(k) / 2, 0, mp.mpf(x) / 2, regularized=True)


def threshold(dof, pfa):
    target = 1 - mp.mpf(pfa)
    low, high = mp.mpf(0), mp.mpf(1000)
    for _ in range(64):
        middle = (low + high) / 2
        if chi2_cdf(middle, dof) < target:
            low = middle
        else:
            high = middle
    return float((low + high) / 2)


def ncx2_cdf(x, k, lam):
    nu = mp.mpf(k) / 2 - 1
    density = lambda t: (mp.mpf(1) / 2 * mp.e ** (-(t + lam) / 2) * (t / lam) ** (nu / 2)
                         * mp.besseli(nu, mp.sqrt(lam * t)))
    return mp.quad(density, [0, x / 2, x])


def noncentrality(t2, dof, pmd):
    if chi2_cdf(t2, dof) <= pmd:
        return 0.0
    f = lambda lam: ncx2_cdf(mp.mpf(t2), dof, lam) - mp.mpf(pmd)
    low, high = mp.mpf("1e-9"), mp.mpf(1)
    while f(high) > 0:
        high *= 2
    for _ in range(48):
        middle = (low + high) / 2
        if f(middle) > 0:
            low = middle
        else:
            high = middle
    return float((low + high) / 2)


def statistics():
    with open("tests/data/statistics_reference.txt", "w") as out:
        out.write("# dof pfa pmd threshold_squared lambda\n")
        for dof in list(range(1, 13)) + [16, 20, 30, 45, 60]:
            for pfa in PFA:
                for pmd in PMD:
                    t2 = threshold(dof, pfa)
                    out.write(f"{dof} {pfa:g} {pmd:g} {t2:.15g} {noncentrality(t2, dof, pmd):.15g}\n")


def skies():
    for line in open("../kinavis-ubx/tests/data/ubx_reference.txt"):
        if " NAV-SAT " not in line:
            continue
        sats = []
        for word in line.split():
            if word.startswith("sv="):
                gnss, sv, cno, elev, azim, prres, used = word[3:].split(",")
                if used == "1" and -90 <= int(elev) <= 90 and 0 <= int(azim) < 360 and gnss in "0123567":
                    sats.append((int(gnss), int(sv), int(elev), int(azim), float(prres)))
        yield sats


def raim(sats):
    clocks = sorted({g for g, *_ in sats})
    m = 3 + len(clocks)
    if len(sats) <= m:
        return None
    G = np.zeros((len(sats), m))
    sigma = np.zeros(len(sats))
    r = np.zeros(len(sats))
    for k, (g, sv, el, az, res) in enumerate(sats):
        e, a = math.radians(el), math.radians(az)
        G[k, :3] = [-math.cos(e) * math.sin(a), -math.cos(e) * math.cos(a), -math.sin(e)]
        G[k, 3 + clocks.index(g)] = 1.0
        sigma[k] = 0.5 / math.sin(math.radians(max(el, 5.0)))
        r[k] = res
    W = np.diag(1 / sigma**2)
    S = np.linalg.inv(G.T @ W @ G) @ G.T @ W
    P = np.eye(len(sats)) - G @ S
    rp = P @ r
    statistic = math.sqrt(rp @ W @ rp)
    dof = len(sats) - m
    t2 = threshold(dof, 1e-3)
    lam = noncentrality(t2, dof, 1e-3)
    pdiag = np.diag(P)
    # A satellite alone in its constellation has zero parity: its clock
    # absorbs it, and it bounds nothing.
    keep = pdiag > 1e-12
    hslope = np.sqrt(S[0][keep] ** 2 + S[1][keep] ** 2) * sigma[keep] / np.sqrt(pdiag[keep])
    vslope = np.abs(S[2][keep]) * sigma[keep] / np.sqrt(pdiag[keep])
    return statistic, math.sqrt(t2), hslope.max() * math.sqrt(lam), vslope.max() * math.sqrt(lam)


def raim_reference():
    with open("tests/data/raim_reference.txt", "w") as out:
        out.write("# sky satellites(gnss,sv,elev,azim,residual) => statistic threshold hpl vpl\n")
        seen = 0
        geometries = set()
        for index, sats in enumerate(skies()):
            geometry = tuple((g, sv, el, az) for g, sv, el, az, _ in sats)
            if geometry in geometries:
                continue
            geometries.add(geometry)
            result = raim(sats)
            if result is None or seen >= 60:
                continue
            seen += 1
            body = " ".join(f"{g},{sv},{el},{az},{res!r}" for g, sv, el, az, res in sats)
            out.write(f"{index} {body} => " + " ".join(f"{v:.12g}" for v in result) + "\n")


if __name__ == "__main__":
    # `statistics` takes minutes at mpmath's precision; `skies` seconds.
    parts = sys.argv[1:] or ["statistics", "skies"]
    if "statistics" in parts:
        statistics()
    if "skies" in parts:
        raim_reference()
