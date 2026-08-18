#!/usr/bin/env python3
"""Generate the fixed-N=8 Lu OMMPC OCP solver used by the C++ benchmark."""

from pathlib import Path
import argparse
import sys
import types

import cvxpy as cp
import numpy as np

# CVXPYgen 1.0 imports the optional PDAQP backend eagerly. This benchmark uses
# OSQP code generation only, so prevent PDAQP from bootstrapping Julia.
_pdaqp_stub = types.ModuleType("cvxpygen.solvers.pdaqp")
_pdaqp_stub.PDAQPInterface = type("PDAQPInterface", (), {})
sys.modules["cvxpygen.solvers.pdaqp"] = _pdaqp_stub
from cvxpygen import cpg


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default=".ommpc_deps/cvxpygen_ommpc")
    args = parser.parse_args()
    output = Path(args.output).resolve()

    nx, nu, horizon = 9, 4, 8
    q_diag = np.array([15000.0] * 3 + [40.0] * 3 + [80.0] * 3)
    r_diag = np.array([0.5, 0.6, 0.6, 0.6])
    q = np.diag(q_diag)
    r = np.diag(r_diag)

    x = cp.Variable((nx, horizon + 1), name="x")
    u = cp.Variable((nu, horizon), name="u")
    x0 = cp.Parameter(nx, name="x0")
    lower_u = cp.Parameter((nu, horizon), name="lower_u")
    upper_u = cp.Parameter((nu, horizon), name="upper_u")
    a = [cp.Parameter((nx, nx), name=f"A_{k}") for k in range(horizon)]
    b = [cp.Parameter((nx, nu), name=f"B_{k}") for k in range(horizon)]

    objective = 0
    constraints = [x[:, 0] == x0, lower_u <= u, u <= upper_u]
    for k in range(horizon):
        objective += 0.5 * cp.quad_form(x[:, k], q)
        objective += 0.5 * cp.quad_form(u[:, k], r)
        constraints.append(x[:, k + 1] == a[k] @ x[:, k] + b[k] @ u[:, k])
    objective += 0.5 * cp.quad_form(x[:, horizon], q)
    problem = cp.Problem(cp.Minimize(objective), constraints)
    if not problem.is_dpp():
        raise RuntimeError("OMMPC code-generation problem is not DPP")

    cpg.generate_code(
        problem,
        code_dir=str(output),
        solver="OSQP",
        solver_opts={"eps_abs": 1e-7, "eps_rel": 1e-7, "max_iter": 120,
                     "warm_starting": True, "polishing": True, "verbose": False},
        wrapper=False,
        prefix="lu_")


if __name__ == "__main__":
    main()
