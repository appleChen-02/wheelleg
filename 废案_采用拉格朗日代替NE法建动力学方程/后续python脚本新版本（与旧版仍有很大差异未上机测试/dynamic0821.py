"""Dynamics-corrected variant of ``compare_dynamics.m``.

The generalized coordinates and most kinematic constraints are kept unchanged.
The hip midpoint velocity is obtained from the actual left/right hip geometry.
The dynamics are rebuilt with a positive gravity magnitude,
centre-of-mass translational accelerations, exact moment equations about
moving reference points, a minimal no-roll body-moment closure, and
automatic force elimination.
The Euler--Lagrange model uses the same geometry and corrected actuator signs.
"""
from __future__ import annotations

import argparse
import numpy as np
import sympy as sp
from scipy.io import savemat
from scipy.optimize import least_squares, minimize


def col(*items):
    return sp.Matrix(items)


def symbolic_model():
    # Physical parameters and generalized coordinates.
    names = "g R R_w m_b I_b l_body_com theta_b0 m_l m_r I_l I_r l_l l_r l_leg_l_com l_leg_r_com theta_l0 theta_r0 m_wl m_wr I_wl I_wr m_t I_t l_tail_com delta_t l_tail_mount_h l_tail_mount_v I_yaw"
    (g, R, R_w, m_b, I_b, l_body_com, theta_b0, m_l, m_r, I_l, I_r,
     l_l, l_r, l_leg_l_com, l_leg_r_com, theta_l0, theta_r0, m_wl, m_wr,
     I_wl, I_wr, m_t, I_t, l_tail_com, delta_t, l_tail_mount_h,
     l_tail_mount_v, I_yaw) = sp.symbols(names, real=True)
    q = col(*sp.symbols("X_b_h phi theta_l theta_r theta_b theta_t", real=True))
    dq = col(*sp.symbols("dX_b_h dphi dtheta_l dtheta_r dtheta_b dtheta_t", real=True))
    ddq = col(*sp.symbols("ddX_b_h ddphi ddtheta_l ddtheta_r ddtheta_b ddtheta_t", real=True))
    X_b_h, phi, theta_l, theta_r, theta_b, theta_t = q
    dX_b_h, dphi, dtheta_l, dtheta_r, dtheta_b, dtheta_t = dq
    ddX_b_h, ddphi, ddtheta_l, ddtheta_r, ddtheta_b, ddtheta_t = ddq
    T_hip_r, T_hip_l, T_motor_wr, T_motor_wl, T_tail = u = col(*sp.symbols("T_hip_r T_hip_l T_motor_wr T_motor_wl T_tail", real=True))
    ddtheta_wl, ddtheta_wr = sp.symbols("ddtheta_wl ddtheta_wr", real=True)

    def cross2(r, force):
        """z component of r x force; positive is counter-clockwise."""
        return r[0]*force[1] - r[1]*force[0]

    # Kinematics are the same as the source code.  Distinguish the hip/body
    # reference acceleration from the actual rigid-body COM accelerations.
    beta_l, beta_r = theta_l + theta_l0, theta_r + theta_r0
    beta_b = theta_b + theta_b0
    alpha_t = theta_t + theta_b - delta_t
    dalpha, ddalpha = dtheta_t + dtheta_b, ddtheta_t + ddtheta_b

    a_wr_h, a_wl_h = R*ddtheta_wr, R*ddtheta_wl
    a_wr = col(a_wr_h, 0)
    a_wl = col(a_wl_h, 0)
    a_hr = col(a_wr_h + l_r*sp.cos(theta_r)*ddtheta_r
               - l_r*sp.sin(theta_r)*dtheta_r**2,
               -l_r*sp.sin(theta_r)*ddtheta_r
               - l_r*sp.cos(theta_r)*dtheta_r**2)
    a_hl = col(a_wl_h + l_l*sp.cos(theta_l)*ddtheta_l
               - l_l*sp.sin(theta_l)*dtheta_l**2,
               -l_l*sp.sin(theta_l)*ddtheta_l
               - l_l*sp.cos(theta_l)*dtheta_l**2)
    a_hip = (a_hr + a_hl)/2

    a_cl = col(a_wl_h + l_leg_l_com*sp.cos(beta_l)*ddtheta_l
               - l_leg_l_com*sp.sin(beta_l)*dtheta_l**2,
               -l_leg_l_com*sp.sin(beta_l)*ddtheta_l
               - l_leg_l_com*sp.cos(beta_l)*dtheta_l**2)
    a_cr = col(a_wr_h + l_leg_r_com*sp.cos(beta_r)*ddtheta_r
               - l_leg_r_com*sp.sin(beta_r)*dtheta_r**2,
               -l_leg_r_com*sp.sin(beta_r)*ddtheta_r
               - l_leg_r_com*sp.cos(beta_r)*dtheta_r**2)
    # theta_b is counter-clockwise positive and beta_b=0 points upward:
    # r_HCb=[-l_bc*sin(beta_b), l_bc*cos(beta_b)].
    r_hcb = col(-l_body_com*sp.sin(beta_b),
                 l_body_com*sp.cos(beta_b))
    a_cb = a_hip + col(-l_body_com*sp.cos(beta_b)*ddtheta_b
                       + l_body_com*sp.sin(beta_b)*dtheta_b**2,
                       -l_body_com*sp.sin(beta_b)*ddtheta_b
                       - l_body_com*sp.cos(beta_b)*dtheta_b**2)

    r_hp = col(l_tail_mount_h*sp.cos(theta_b)
               + l_tail_mount_v*sp.sin(theta_b),
               l_tail_mount_h*sp.sin(theta_b)
               - l_tail_mount_v*sp.cos(theta_b))
    a_tp = a_hip + col(-r_hp[1]*ddtheta_b-r_hp[0]*dtheta_b**2,
                       r_hp[0]*ddtheta_b-r_hp[1]*dtheta_b**2)
    r_pct = col(l_tail_com*sp.cos(alpha_t), l_tail_com*sp.sin(alpha_t))
    a_ct = a_tp + col(-l_tail_com*(ddalpha*sp.sin(alpha_t)
                                   + dalpha**2*sp.cos(alpha_t)),
                      l_tail_com*(ddalpha*sp.cos(alpha_t)
                                  - dalpha**2*sp.sin(alpha_t)))

    # Forces use the original naming: e.g. F_l_to_b is the force exerted by
    # the left leg on the body.  Gravity is [0, -m*g] with g > 0.
    (F_l_to_b_h, F_l_to_b_v, F_r_to_b_h, F_r_to_b_v,
     F_wl_to_l_h, F_wl_to_l_v, F_wr_to_r_h, F_wr_to_r_v,
     F_g_to_wl_h, F_g_to_wl_v, F_g_to_wr_h, F_g_to_wr_v,
     F_t_to_b_h, F_t_to_b_v) = sp.symbols(
        "F_l_to_b_h F_l_to_b_v F_r_to_b_h F_r_to_b_v "
        "F_wl_to_l_h F_wl_to_l_v F_wr_to_r_h F_wr_to_r_v "
        "F_g_to_wl_h F_g_to_wl_v F_g_to_wr_h F_g_to_wr_v "
        "F_t_to_b_h F_t_to_b_v", real=True)

    F_lb, F_rb = col(F_l_to_b_h, F_l_to_b_v), col(F_r_to_b_h, F_r_to_b_v)
    F_wll, F_wrr = col(F_wl_to_l_h, F_wl_to_l_v), col(F_wr_to_r_h, F_wr_to_r_v)
    F_gwl, F_gwr = col(F_g_to_wl_h, F_g_to_wl_v), col(F_g_to_wr_h, F_g_to_wr_v)
    F_tb = col(F_t_to_b_h, F_t_to_b_v)

    eq_body_h_raw = F_lb[0]+F_rb[0]+F_tb[0]-m_b*a_cb[0]
    eq_body_v_raw = F_lb[1]+F_rb[1]+F_tb[1]-m_b*g-m_b*a_cb[1]
    # Moment about the moving hip midpoint H:
    # sum(M_H) = I_H*alpha + r_HC x m*a_H.
    I_b_h = I_b+m_b*(r_hcb.dot(r_hcb))
    eq_body_rot_raw = (T_hip_l+T_hip_r+T_tail+cross2(r_hp, F_tb)
        + cross2(r_hcb, col(0, -m_b*g))-I_b_h*ddtheta_b
        - cross2(r_hcb, m_b*a_hip))

    eq_tail_h_raw = -F_tb[0]-m_t*a_ct[0]
    eq_tail_v_raw = -F_tb[1]-m_t*g-m_t*a_ct[1]
    # Moment about the moving tail mount P.
    I_t_p = I_t+m_t*(r_pct.dot(r_pct))
    eq_tail_rot_raw = (-T_tail+cross2(r_pct, col(0, -m_t*g))
                       -I_t_p*ddalpha-cross2(r_pct, m_t*a_tp))

    r_wcl = col(l_leg_l_com*sp.sin(beta_l), l_leg_l_com*sp.cos(beta_l))
    r_wcr = col(l_leg_r_com*sp.sin(beta_r), l_leg_r_com*sp.cos(beta_r))
    r_whl = col(l_l*sp.sin(theta_l), l_l*sp.cos(theta_l))
    r_whr = col(l_r*sp.sin(theta_r), l_r*sp.cos(theta_r))
    eq_leg_l_h_raw = -F_lb[0]+F_wll[0]-m_l*a_cl[0]
    eq_leg_l_v_raw = -F_lb[1]+F_wll[1]-m_l*g-m_l*a_cl[1]
    r_hcl = r_wcl-r_whl
    I_l_h = I_l+m_l*(r_hcl.dot(r_hcl))
    eq_leg_l_rot_raw = (cross2(-r_whl, F_wll)
        + cross2(r_hcl, col(0, -m_l*g))-T_hip_l-T_motor_wl
        + I_l_h*ddtheta_l-cross2(r_hcl, m_l*a_hl))
    eq_leg_r_h_raw = -F_rb[0]+F_wrr[0]-m_r*a_cr[0]
    eq_leg_r_v_raw = -F_rb[1]+F_wrr[1]-m_r*g-m_r*a_cr[1]
    r_hcr = r_wcr-r_whr
    I_r_h = I_r+m_r*(r_hcr.dot(r_hcr))
    eq_leg_r_rot_raw = (cross2(-r_whr, F_wrr)
        + cross2(r_hcr, col(0, -m_r*g))-T_hip_r-T_motor_wr
        + I_r_h*ddtheta_r-cross2(r_hcr, m_r*a_hr))

    eq_wl_h_raw = -F_wll[0]+F_gwl[0]-m_wl*a_wl[0]
    eq_wl_v_raw = -F_wll[1]+F_gwl[1]-m_wl*g
    eq_wl_rot_raw = T_motor_wl+R*F_gwl[0]+I_wl*ddtheta_wl
    eq_wr_h_raw = -F_wrr[0]+F_gwr[0]-m_wr*a_wr[0]
    eq_wr_v_raw = -F_wrr[1]+F_gwr[1]-m_wr*g
    eq_wr_rot_raw = T_motor_wr+R*F_gwr[0]+I_wr*ddtheta_wr
    # Body-only yaw balance.  I_yaw is the body/central-assembly yaw inertia;
    # for now the hip lateral half-spacing is approximated by R_w.
    eq_yaw_raw = R_w*(F_rb[0]-F_lb[0])-I_yaw*ddphi
    eqs_ne_raw_19 = [
        eq_body_h_raw, eq_body_v_raw, eq_body_rot_raw,
        eq_tail_h_raw, eq_tail_v_raw, eq_tail_rot_raw,
        eq_leg_l_h_raw, eq_leg_l_v_raw, eq_leg_l_rot_raw,
        eq_leg_r_h_raw, eq_leg_r_v_raw, eq_leg_r_rot_raw,
        eq_wl_h_raw, eq_wl_v_raw, eq_wl_rot_raw,
        eq_wr_h_raw, eq_wr_v_raw, eq_wr_rot_raw, eq_yaw_raw,
    ]
    assert len(eqs_ne_raw_19) == 19

    # Minimal no-roll closure for the central body.  With the left/right hips
    # at lateral offsets +R_h/-R_h and ddroll=0, the body roll moment balance is
    # R_h*(F_l_to_b_v-F_r_to_b_v)=0.  The nonzero common arm can be omitted.
    # Unlike N_l=N_r, this lets the two ground normal loads differ when the
    # left/right leg COM vertical accelerations differ.
    eq_body_no_roll = F_l_to_b_v-F_r_to_b_v
    eqs_ne_with_closure = eqs_ne_raw_19+[eq_body_no_roll]
    force_unknowns = col(
        F_l_to_b_h, F_l_to_b_v, F_r_to_b_h, F_r_to_b_v,
        F_wl_to_l_h, F_wl_to_l_v, F_wr_to_r_h, F_wr_to_r_v,
        F_g_to_wl_h, F_g_to_wl_v, F_g_to_wr_h, F_g_to_wr_v,
        F_t_to_b_h, F_t_to_b_v)
    E20 = sp.Matrix(eqs_ne_with_closure)
    A_force = E20.jacobian(force_unknowns)
    left_null_basis_raw = A_force.T.nullspace()
    if len(left_null_basis_raw) != 6:
        raise RuntimeError(
            f"NE force elimination expected 6 equations, got {len(left_null_basis_raw)}")

    # SymPy's generic nullspace basis can contain geometry-dependent
    # denominators.  Such a basis may produce zoo/nan at a perfectly regular
    # symmetric configuration even when the underlying equation space is not
    # singular.  Clear the denominators row by row before forming the six
    # symbolic reduced equations.
    left_null_basis = []
    for basis_vector in left_null_basis_raw:
        denominators = []
        seen = set()
        for entry in basis_vector:
            denominator = sp.fraction(sp.cancel(entry))[1]
            key = sp.srepr(denominator)
            if denominator != 1 and key not in seen:
                denominators.append(denominator)
                seen.add(key)
        common_denominator = sp.prod(denominators) if denominators else sp.Integer(1)
        left_null_basis.append(
            basis_vector.applyfunc(lambda entry: sp.cancel(common_denominator*entry)))
    elimination_matrix = sp.Matrix.hstack(*left_null_basis).T
    eqs_ne_wheels = list(elimination_matrix*E20)

    # Rolling/differential kinematics, using each leg's own wheel-to-hip
    # relative acceleration.  This is the acceleration-level derivative of
    # the two wheel velocities used by the Lagrange model below.
    leg_term_l = (l_l*sp.cos(theta_l)*ddtheta_l
                  - l_l*sp.sin(theta_l)*dtheta_l**2)
    leg_term_r = (l_r*sp.cos(theta_r)*ddtheta_r
                  - l_r*sp.sin(theta_r)*dtheta_r**2)
    wheel_subs = {
        ddtheta_wl: (ddX_b_h-R_w*ddphi-leg_term_l)/R,
        ddtheta_wr: (ddX_b_h+R_w*ddphi-leg_term_r)/R,
    }

    # Kinematic consistency checks against the geometric left/right hip
    # accelerations implied by the Lagrange velocity constraints.
    a_hl_constrained = a_hl.subs(wheel_subs)
    a_hr_constrained = a_hr.subs(wheel_subs)
    assert sp.simplify(a_hl_constrained[0]-(ddX_b_h-R_w*ddphi)) == 0
    assert sp.simplify(a_hr_constrained[0]-(ddX_b_h+R_w*ddphi)) == 0
    assert sp.simplify((a_hl_constrained[0]+a_hr_constrained[0])/2-ddX_b_h) == 0
    E20_constrained = E20.applyfunc(lambda e: sp.expand(e.subs(wheel_subs)))
    eqs_ne = [sp.expand(e.subs(wheel_subs)) for e in eqs_ne_wheels]
    M_ne, g_ne, B_ne = extract(eqs_ne, ddq, u)

    # A numerically robust NE representation retains all 20 equations and
    # solves simultaneously for 6 generalized accelerations and 14 forces.
    # This avoids evaluating an arbitrary symbolic nullspace basis.
    full_ne_unknowns = ddq.col_join(force_unknowns)
    D_ne_full = E20_constrained.jacobian(full_ne_unknowns)
    B_ne_full_raw = E20_constrained.jacobian(u)
    B_ne_full = -B_ne_full_raw
    zero_full_ne = {symbol: 0 for symbol in list(full_ne_unknowns)+list(u)}
    g_ne_full = E20_constrained.subs(zero_full_ne)
    ne_affine_error = (E20_constrained-D_ne_full*full_ne_unknowns
                       -g_ne_full+B_ne_full*u)
    if any(sp.simplify(entry) != 0 for entry in ne_affine_error):
        raise RuntimeError("the full NE equations are not affine in ddq, forces, and u")

    # Lagrange model.
    v_wl_h = dX_b_h-R_w*dphi-l_l*sp.cos(theta_l)*dtheta_l
    v_wr_h = dX_b_h+R_w*dphi-l_r*sp.cos(theta_r)*dtheta_r

    # Geometric hip velocities.  Each hip velocity is obtained from its wheel
    # centre plus the derivative of r_WH=[l*sin(theta), l*cos(theta)].
    v_hl_h = v_wl_h+l_l*sp.cos(theta_l)*dtheta_l
    v_hr_h = v_wr_h+l_r*sp.cos(theta_r)*dtheta_r
    v_hl_v = -l_l*sp.sin(theta_l)*dtheta_l
    v_hr_v = -l_r*sp.sin(theta_r)*dtheta_r
    v_hip_h = sp.simplify((v_hl_h+v_hr_h)/2)
    v_hip_v = (v_hl_v+v_hr_v)/2
    assert sp.simplify(v_hip_h-dX_b_h) == 0
    K_wl = sp.Rational(1,2)*(m_wl+I_wl/R**2)*v_wl_h**2
    K_wr = sp.Rational(1,2)*(m_wr+I_wr/R**2)*v_wr_h**2
    v_legl_h = v_wl_h+l_leg_l_com*sp.cos(theta_l+theta_l0)*dtheta_l
    v_legl_v = -l_leg_l_com*sp.sin(theta_l+theta_l0)*dtheta_l
    v_legr_h = v_wr_h+l_leg_r_com*sp.cos(theta_r+theta_r0)*dtheta_r
    v_legr_v = -l_leg_r_com*sp.sin(theta_r+theta_r0)*dtheta_r
    K_legl = sp.Rational(1,2)*m_l*(v_legl_h**2+v_legl_v**2)+sp.Rational(1,2)*I_l*dtheta_l**2
    K_legr = sp.Rational(1,2)*m_r*(v_legr_h**2+v_legr_v**2)+sp.Rational(1,2)*I_r*dtheta_r**2
    # x_Cb=X_b_h-l_body_com*sin(beta_b),
    # y_Cb=y_hip+l_body_com*cos(beta_b).
    v_b_h = v_hip_h-l_body_com*sp.cos(theta_b+theta_b0)*dtheta_b
    v_b_v = v_hip_v-l_body_com*sp.sin(theta_b+theta_b0)*dtheta_b
    K_body = sp.Rational(1,2)*m_b*(v_b_h**2+v_b_v**2)+sp.Rational(1,2)*I_b*dtheta_b**2
    r_t_h, r_t_v = r_hp[0], r_hp[1]
    v_mount_h, v_mount_v = v_hip_h-r_t_v*dtheta_b, v_hip_v+r_t_h*dtheta_b
    v_tail_h = v_mount_h-l_tail_com*sp.sin(alpha_t)*dalpha
    v_tail_v = v_mount_v+l_tail_com*sp.cos(alpha_t)*dalpha
    K_tail = sp.Rational(1,2)*m_t*(v_tail_h**2+v_tail_v**2)+sp.Rational(1,2)*I_t*dalpha**2
    T = K_wl+K_wr+K_legl+K_legr+K_body+sp.Rational(1,2)*I_yaw*dphi**2+K_tail
    y_hip = R+(l_l*sp.cos(theta_l)+l_r*sp.cos(theta_r))/2
    V = (m_wl*g*R+m_wr*g*R+m_l*g*(R+l_leg_l_com*sp.cos(theta_l+theta_l0))
         +m_r*g*(R+l_leg_r_com*sp.cos(theta_r+theta_r0))
         +m_b*g*(y_hip+l_body_com*sp.cos(theta_b+theta_b0))
         +m_t*g*(y_hip+r_t_v+l_tail_com*sp.sin(alpha_t)))
    L = T-V
    Q = col(-(T_motor_wl+T_motor_wr)/R, R_w/R*(T_motor_wl-T_motor_wr),
            T_motor_wl*(1+l_l*sp.cos(theta_l)/R)+T_hip_l,
            T_motor_wr*(1+l_r*sp.cos(theta_r)/R)+T_hip_r,
            T_hip_l+T_hip_r, -T_tail)
    eqs_lag = []
    for k in range(6):
        p = sp.diff(L, dq[k])
        dpdt = sum(sp.diff(p, q[j])*dq[j]+sp.diff(p, dq[j])*ddq[j] for j in range(6))
        eqs_lag.append(sp.expand(dpdt-sp.diff(L, q[k])-Q[k]))
    M_lag, g_lag, B_lag = extract(eqs_lag, ddq, u)
    return locals()


def extract(eqs, ddq, u):
    E = sp.Matrix(eqs)
    M = E.jacobian(ddq)
    B_raw = E.jacobian(u)
    B = -B_raw
    # E = M*ddq + g - B*u = 0.  Direct zero substitution is more robust
    # than relying on SymPy to cancel all linear terms in a large expression.
    zero_dynamic = {symbol: 0 for symbol in list(ddq)+list(u)}
    g = E.subs(zero_dynamic)
    affine_error = E-M*ddq-g+B*u
    if any(sp.simplify(entry) != 0 for entry in affine_error):
        raise RuntimeError("dynamics equations are not affine in ddq and u")
    return M, g, B


def numeric_substitutions(s):
    values = dict(g=9.81, R=.130, R_w=.386/2, m_b=6.9, I_b=59035.925e-6,
        l_body_com=4.3e-3, I_yaw=294272.34e-6, theta_b0=0, m_wl=.823, m_wr=.823,
        I_wl=6311.798e-6, I_wr=6311.798e-6, l_l=.17, l_r=.17, m_l=2.2, m_r=2.2,
        I_l=.034231929, I_r=.034231929, l_leg_l_com=.10157, l_leg_r_com=.10157,
        theta_l0=.582108261, theta_r0=.582108261, m_t=.83, I_t=29341.743e-6,
        l_tail_com=.17755, delta_t=.06597, l_tail_mount_h=.07125, l_tail_mount_v=.1105)
    subs = {s[name]: value for name, value in values.items()}
    # Deliberately asymmetric, nonzero-velocity test point.  A symmetric
    # zero-velocity pose cannot expose left/right kinematic sign errors.
    subs.update({s['theta_l']:.27, s['theta_r']:.34,
                 s['theta_b']:.08, s['theta_t']:-.12,
                 s['phi']:.04, s['X_b_h']:0})
    dq_values = (.12, -.07, .09, -.05, .04, -.06)
    subs.update({symbol: value for symbol, value in zip(s['dq'], dq_values)})
    subs.update({x:0 for x in list(s['ddq'])+list(s['u'])})
    return subs, values


def state_substitutions(s, parameter_values, q_values, dq_values):
    """Create a complete numeric substitution set for one comparison state."""
    if len(q_values) != 6 or len(dq_values) != 6:
        raise ValueError("q_values and dq_values must each contain six entries")
    subs = {s[name]: value for name, value in parameter_values.items()}
    subs.update({symbol: value for symbol, value in zip(s['q'], q_values)})
    subs.update({symbol: value for symbol, value in zip(s['dq'], dq_values)})
    subs.update({symbol: 0 for symbol in list(s['ddq']) + list(s['u'])})
    return subs


def evaluate(expr, subs, name="expression", imag_tolerance=1e-10):
    """Evaluate a symbolic matrix with actionable diagnostics.

    Converting a SymPy matrix directly to ``dtype=float`` hides whether a
    failure came from an unsubstituted symbol, zoo/nan, or a genuine complex
    value.  Evaluate through complex numbers, reject non-finite/significant
    imaginary entries, and return a real NumPy array.
    """
    evaluated = expr.subs(subs)
    unresolved = sorted(str(symbol) for symbol in evaluated.free_symbols)
    if unresolved:
        raise ValueError(f"{name} still contains symbols: {', '.join(unresolved)}")

    bad_symbolic = []
    for index, value in enumerate(evaluated):
        if value.has(sp.zoo, sp.oo, -sp.oo, sp.nan):
            bad_symbolic.append((index, value))
    if bad_symbolic:
        preview = ", ".join(f"flat[{i}]={value}" for i, value in bad_symbolic[:6])
        raise ValueError(f"{name} contains non-finite symbolic entries: {preview}")

    numeric = np.array(evaluated.evalf(), dtype=np.complex128)
    if not np.all(np.isfinite(numeric)):
        bad = np.argwhere(~np.isfinite(numeric))
        raise ValueError(f"{name} contains non-finite numeric entries at {bad.tolist()[:6]}")
    max_imag = float(np.max(np.abs(numeric.imag))) if numeric.size else 0.0
    if max_imag > imag_tolerance:
        index = np.unravel_index(np.argmax(np.abs(numeric.imag)), numeric.shape)
        raise ValueError(
            f"{name} has a genuine complex value at {index}: {numeric[index]} "
            f"(max |imag|={max_imag:.3e})")
    return numeric.real.astype(float)


def symbolic_strings(expr):
    """MATLAB files cannot directly store SymPy objects; preserve formulas as text."""
    return np.array([[str(expr[i, j]) for j in range(expr.cols)]
                     for i in range(expr.rows)], dtype=object)


def simplify_matrix(matrix):
    """Element-wise equivalent of MATLAB simplify(matrix)."""
    return matrix.applyfunc(sp.simplify)


def nonzero_count(matrix):
    """Equivalent to the MATLAB symbolic difference counters after simplify."""
    return sum(value != 0 for value in matrix)


def controllability(A, B):
    return np.hstack([np.linalg.matrix_power(A, k) @ B for k in range(A.shape[0])])


def interleave_state_space(A_qd, B_qd):
    """Convert [q; dq] ordering to the legacy [q1,dq1,q2,dq2,...] order.

    Python's generalized-coordinate order is
    [X, phi, theta_l, theta_r, theta_b, theta_t].  The old MATLAB/C
    controller expects [X,dX,phi,dphi,...].  This is a permutation only;
    no dynamics are recomputed here.
    """
    index = np.array([0, 6, 1, 7, 2, 8, 3, 9, 4, 10, 5, 11], dtype=int)
    A_qd = np.asarray(A_qd, dtype=float)
    B_qd = np.asarray(B_qd, dtype=float)
    if A_qd.shape != (12, 12) or B_qd.shape != (12, 5):
        raise ValueError(
            f"expected A 12x12 and B 12x5, got {A_qd.shape} and {B_qd.shape}")
    return A_qd[np.ix_(index, index)], B_qd[index, :], index


def leg_length_parameters(base_values, l_l_value, l_r_value):
    """Return the new-model parameter set for one asymmetric leg-length pair.

    These four polynomials are the empirical relationships already used by
    compute_lqr_0809.m.  Positive theta_*0 is deliberate: it matches the
    confirmed angle convention of this corrected model.
    """
    def theta0(length):
        return (277.69*length**4 - 304.33*length**3 + 130.52*length**2
                - 27.562*length + 2.7591)

    def leg_com(length):
        return (1.4205*length**4 - 2.0644*length**3 + 1.2526*length**2
                - .059*length + .0844)

    def leg_inertia(length):
        return (-.3588*length**4 + .3595*length**3 + .0659*length**2
                + .0318*length + .0255)

    def yaw_inertia(mean_length):
        return (-6.4624*mean_length**4 + 6.7339*mean_length**3
                - 3.0029*mean_length**2 + .5202*mean_length + .2758)

    # The historical polynomials do not pass exactly through the measured
    # nominal values used by this model at L=0.17 m.  Anchor them here so a
    # scan contains the validated nominal model exactly, while preserving the
    # old relation's change with leg length.
    reference_length = .17
    theta_l_delta = base_values['theta_l0']-theta0(reference_length)
    theta_r_delta = base_values['theta_r0']-theta0(reference_length)
    leg_com_l_delta = base_values['l_leg_l_com']-leg_com(reference_length)
    leg_com_r_delta = base_values['l_leg_r_com']-leg_com(reference_length)
    I_l_scale = base_values['I_l']/leg_inertia(reference_length)
    I_r_scale = base_values['I_r']/leg_inertia(reference_length)
    I_yaw_scale = base_values['I_yaw']/yaw_inertia(reference_length)

    values = dict(base_values)
    values.update(
        l_l=float(l_l_value), l_r=float(l_r_value),
        theta_l0=theta0(l_l_value)+theta_l_delta,
        theta_r0=theta0(l_r_value)+theta_r_delta,
        l_leg_l_com=leg_com(l_l_value)+leg_com_l_delta,
        l_leg_r_com=leg_com(l_r_value)+leg_com_r_delta,
        I_l=I_l_scale*leg_inertia(l_l_value),
        I_r=I_r_scale*leg_inertia(l_r_value),
        I_yaw=I_yaw_scale*yaw_inertia((l_l_value+l_r_value)/2))
    positive = ('l_l', 'l_r', 'l_leg_l_com', 'l_leg_r_com', 'I_l', 'I_r', 'I_yaw')
    invalid = [name for name in positive if values[name] <= 0]
    if invalid:
        raise ValueError(f"leg-length mapping produced non-positive values: {invalid}")
    return values


def build_lagrange_numeric_model(s):
    """Compile M(q), h(q,dq), B(q) once for a whole leg-length scan."""
    names = ('g', 'R', 'R_w', 'm_b', 'I_b', 'l_body_com', 'theta_b0',
             'm_l', 'm_r', 'I_l', 'I_r', 'l_l', 'l_r', 'l_leg_l_com',
             'l_leg_r_com', 'theta_l0', 'theta_r0', 'm_wl', 'm_wr',
             'I_wl', 'I_wr', 'm_t', 'I_t', 'l_tail_com', 'delta_t',
             'l_tail_mount_h', 'l_tail_mount_v', 'I_yaw')
    symbols = [s[name] for name in names] + list(s['q']) + list(s['dq'])
    M_fun = sp.lambdify(symbols, s['M_lag'], modules='numpy', cse=True)
    h_fun = sp.lambdify(symbols, s['g_lag'], modules='numpy', cse=True)
    B_fun = sp.lambdify(symbols, s['B_lag'], modules='numpy', cse=True)

    def evaluate_model(parameters, q_values, dq_values):
        args = [parameters[name] for name in names]
        args.extend(np.asarray(q_values, dtype=float).reshape(6))
        args.extend(np.asarray(dq_values, dtype=float).reshape(6))
        M = np.asarray(M_fun(*args), dtype=float).reshape(6, 6)
        h = np.asarray(h_fun(*args), dtype=float).reshape(6)
        B = np.asarray(B_fun(*args), dtype=float).reshape(6, 5)
        if not (np.all(np.isfinite(M)) and np.all(np.isfinite(h))
                and np.all(np.isfinite(B))):
            raise FloatingPointError('non-finite Lagrange model evaluation')
        return M, h, B
    return evaluate_model


def solve_lagrange_equilibrium(model, parameters, initial=None):
    """Select one physical asymmetric static equilibrium by constrained minimization.

    q=[0,0,theta_l,theta_r,theta_b,theta_t], dq=0.  All five equilibrium
    inputs are free, including the two wheel torques.  The six exact static
    equations h(q,0)-B(q)u=0 are equality constraints.  The objective merely
    resolves their three remaining degrees of freedom by preferring the
    nominal posture and small actuator torques; it is not substituted for the
    equilibrium residual.
    """
    angle_reference = np.array([.30, .30, 0., 0.])
    angle_weight = np.array([1., 1., 8., 8.])
    input_weight = np.array([.08, .08, .12, .12, .08])

    def static_terms(z):
        q = np.array([0., 0., z[0], z[1], z[2], z[3]])
        _, h, B = model(parameters, q, np.zeros(6))
        return q, h, B

    def objective(z):
        return .5*np.sum((angle_weight*(z[:4]-angle_reference))**2) + \
               .5*np.sum((input_weight*z[4:])**2)

    def equality(z):
        _, h, B = static_terms(z)
        return h-B@z[4:]

    if initial is None:
        q0 = np.r_[angle_reference, np.zeros(2)]
        _, h0, B0 = static_terms(np.r_[q0[2:], np.zeros(5)])
        u0 = np.linalg.lstsq(B0, h0, rcond=None)[0]
        initial = np.r_[angle_reference, u0]
    initial = np.asarray(initial, dtype=float).reshape(9)
    bounds = [(-np.pi, np.pi)]*4 + [(-80., 80.)]*5
    solution = minimize(
        objective, initial, method='SLSQP', bounds=bounds,
        constraints={'type': 'eq', 'fun': equality},
        options={'ftol': 1e-11, 'maxiter': 600, 'disp': False})
    residual = equality(solution.x)
    if (not solution.success) or np.linalg.norm(residual, np.inf) > 1e-7:
        raise RuntimeError(
            f"equilibrium solve failed: {solution.message}; "
            f"||h-Bu||_inf={np.linalg.norm(residual, np.inf):.3e}")
    q, _, _ = static_terms(solution.x)
    return dict(q=q, dq=np.zeros(6), u=solution.x[4:],
                angles=solution.x[:4], residual=residual,
                cost=solution.fun, iterations=solution.nit)


def linearize_lagrange_numeric(model, parameters, q_eq, u_eq,
                                relative_step=1e-5):
    """Finite-difference the full Lagrange acceleration field at equilibrium."""
    x_eq = np.r_[np.asarray(q_eq, dtype=float).reshape(6), np.zeros(6)]
    u_eq = np.asarray(u_eq, dtype=float).reshape(5)

    def acceleration(x):
        M, h, B = model(parameters, x[:6], x[6:])
        return np.linalg.solve(M, -h+B@u_eq)

    A = np.zeros((12, 12))
    A[:6, 6:] = np.eye(6)
    steps = relative_step*np.maximum(1., np.abs(x_eq))
    for col_index, step in enumerate(steps):
        xp, xm = x_eq.copy(), x_eq.copy()
        xp[col_index] += step
        xm[col_index] -= step
        A[6:, col_index] = (acceleration(xp)-acceleration(xm))/(2*step)
    M_eq, _, B_eq = model(parameters, q_eq, np.zeros(6))
    B_ss = np.vstack([np.zeros((6, 5)), np.linalg.solve(M_eq, B_eq)])
    return A, B_ss, x_eq, steps


def lqr_gain(A, B, Q, R):
    """Continuous-time LQR without requiring the Python-control package."""
    from scipy.linalg import solve_continuous_are
    S = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T@S)
    poles = np.linalg.eigvals(A-B@K)
    return K, S, poles


def scan_lagrange_leg_lengths(s, base_values, l_range, output_file,
                              q_weight=None, r_weight=None):
    """Generate the new-model K samples and 60x6 poly22 coefficients."""
    model = build_lagrange_numeric_model(s)
    Q = np.diag([120, 400, 4000, 1, 2000, 10, 2000, 10, 6000, 1, 300, 5]
                if q_weight is None else q_weight)
    R = np.diag([1, 1, 12, 12, 8] if r_weight is None else r_weight)
    l_range = np.asarray(l_range, dtype=float).reshape(-1)
    pairs = [(ll, lr) for ll in l_range for lr in l_range]
    count = len(pairs)
    K_samples = np.full((count, 62), np.nan)
    offset_samples = np.full((count, 6), np.nan)
    input_samples = np.full((count, 7), np.nan)  # ll, lr + five inputs
    A_samples = np.full((count, 144), np.nan)
    B_samples = np.full((count, 60), np.nan)
    residual_inf = np.full(count, np.nan)
    valid = np.zeros(count, dtype=bool)
    previous = None
    for index, (ll, lr) in enumerate(pairs):
        try:
            parameters = leg_length_parameters(base_values, ll, lr)
            equilibrium = solve_lagrange_equilibrium(model, parameters, previous)
            A_qd, B_qd, x_eq_qd, steps = linearize_lagrange_numeric(
                model, parameters, equilibrium['q'], equilibrium['u'])
            A_int, B_int, permutation = interleave_state_space(A_qd, B_qd)
            x_eq_int = x_eq_qd[permutation]
            K, _, poles = lqr_gain(A_int, B_int, Q, R)
            if np.max(np.real(poles)) >= 1e-7:
                print(f"  warning: closed loop not stable at ({ll:.4f},{lr:.4f})")
            # MATLAB's K_fit_trans(:)' where K_fit_trans=K' is row-major K.
            K_samples[index, :] = np.r_[ll, lr, K.reshape(-1)]
            offset_samples[index, :] = np.r_[ll, lr, equilibrium['angles']]
            input_samples[index, :] = np.r_[ll, lr, equilibrium['u']]
            A_samples[index, :] = A_int.reshape(-1)
            B_samples[index, :] = B_int.reshape(-1)
            residual_inf[index] = np.linalg.norm(equilibrium['residual'], np.inf)
            valid[index] = True
            previous = np.r_[equilibrium['angles'], equilibrium['u']]
        except Exception as error:
            print(f"  failed ({ll:.4f},{lr:.4f}): {error}")
            previous = None
        if (index+1) % max(1, count//20) == 0 or index+1 == count:
            print(f"  scan progress: {index+1}/{count}")

    if np.count_nonzero(valid) < 20:
        raise RuntimeError(f"only {np.count_nonzero(valid)} valid scan points; need at least 20")
    samples = K_samples[valid]
    offsets = offset_samples[valid]
    inputs = input_samples[valid]
    ll_values, lr_values = samples[:, 0], samples[:, 1]
    Phi = np.column_stack((np.ones_like(ll_values), ll_values, lr_values,
                           ll_values**2, ll_values*lr_values, lr_values**2))
    fit_values = np.column_stack((samples[:, 2:62], offsets[:, 2:6], inputs[:, 2:7]))
    coefficients, _, _, _ = np.linalg.lstsq(Phi, fit_values, rcond=None)
    K_fit = coefficients[:, :60].T
    offset_fit = coefficients[:, 60:64].T
    input_fit = coefficients[:, 64:69].T
    savemat(output_file, dict(
        K_sample_2d=K_samples, K_sample_2d_valid=samples,
        offset_sample_2d=offset_samples, offset_sample_2d_valid=offsets,
        input_eq_sample_2d=input_samples, input_eq_sample_2d_valid=inputs,
        K_Fit_Coefficients=K_fit,
        Offset_Fit_Coefficients=offset_fit,
        InputEq_Fit_Coefficients=input_fit,
        l_range=l_range, valid_mask=valid,
        equilibrium_residual_inf=residual_inf,
        A_sample_int=A_samples, B_sample_int=B_samples,
        poly22_rms_error=np.sqrt(np.mean((Phi@coefficients-fit_values)**2, axis=0)),
        state_order=np.array([
            'X_b_h', 'dX_b_h', 'phi', 'dphi', 'theta_l', 'dtheta_l',
            'theta_r', 'dtheta_r', 'theta_b', 'dtheta_b',
            'theta_t', 'dtheta_t'], dtype=object),
        input_order=np.array([
            'T_hip_r', 'T_hip_l', 'T_motor_wr', 'T_motor_wl', 'T_tail'],
            dtype=object),
        model='Lagrange'))
    print(f"saved {output_file}: {np.count_nonzero(valid)}/{count} valid points")
    print(f"K_Fit_Coefficients shape = {K_fit.shape}")
    return dict(K_samples=K_samples, valid=valid,
                K_fit=K_fit, offset_fit=offset_fit, input_fit=input_fit)


def print_matrix(name, matrix, fmt="10.4f"):
    print(f"--- {name} ---")
    for row in np.atleast_2d(matrix):
        print("  " + " ".join(f"{float(value):{fmt}}" for value in row))


def print_symbolic_differences(name, matrix, max_items=None):
    entries = []
    for i in range(matrix.rows):
        for j in range(matrix.cols):
            if matrix[i, j] != 0:
                label = f"{name}({i + 1})" if matrix.cols == 1 else f"{name}({i + 1},{j + 1})"
                entries.append(f"    {label}: {matrix[i, j]}")
    if not entries:
        print(f"  {name} 完全一致")
        return
    print(f"  {name} 存在 {len(entries)} 处差异")
    displayed = entries if max_items is None else entries[:max_items]
    print("\n".join(displayed))
    if max_items is not None and len(entries) > max_items:
        print(f"    ... (还有 {len(entries) - max_items} 处差异未显示)")


def explicit_dynamics_at(s, subs):
    """Evaluate the row-invariant explicit dynamics at one state.

    Returns d=-M^-1*h and G=M^-1*B for Lagrange and for the complete
    Newton--Euler system.  The numerical left-nullspace result is retained as
    a consistency check, but is not used as the primary NE answer.
    """
    D_ne_full = evaluate(s['D_ne_full'], subs, 'D_ne_full')
    g_ne_full = evaluate(s['g_ne_full'], subs, 'g_ne_full')
    B_ne_full = evaluate(s['B_ne_full'], subs, 'B_ne_full')
    M_lag = evaluate(s['M_lag'], subs, 'M_lag')
    g_lag = evaluate(s['g_lag'], subs, 'g_lag')
    B_lag = evaluate(s['B_lag'], subs, 'B_lag')

    rank_full = np.linalg.matrix_rank(D_ne_full)
    condition_full = np.linalg.cond(D_ne_full)
    if rank_full != 20:
        raise np.linalg.LinAlgError(
            f"D_ne_full is rank deficient: rank={rank_full}/20, "
            f"condition={condition_full:.3e}")

    full_drift_solution = np.linalg.solve(D_ne_full, -g_ne_full)
    full_input_solution = np.linalg.solve(D_ne_full, B_ne_full)
    drift_ne = full_drift_solution[:6, :]
    input_ne = full_input_solution[:6, :]

    A_ddq_num, A_force_num = D_ne_full[:, :6], D_ne_full[:, 6:]
    _, force_singular_values, force_vh = np.linalg.svd(
        A_force_num.T, full_matrices=True)
    force_tolerance = (max(A_force_num.T.shape) * np.finfo(float).eps
                       * force_singular_values[0])
    rank_force = int(np.sum(force_singular_values > force_tolerance))
    if rank_force != 14:
        raise np.linalg.LinAlgError(
            f"force coefficient rank is {rank_force}, expected 14")
    N_force = force_vh[rank_force:, :].T
    M_ne = N_force.T @ A_ddq_num
    g_ne = N_force.T @ g_ne_full
    B_ne = N_force.T @ B_ne_full
    drift_ne_reduced = np.linalg.solve(M_ne, -g_ne)
    input_ne_reduced = np.linalg.solve(M_ne, B_ne)
    if not (np.allclose(drift_ne, drift_ne_reduced, rtol=1e-8, atol=1e-9)
            and np.allclose(input_ne, input_ne_reduced,
                            rtol=1e-8, atol=1e-9)):
        raise RuntimeError("full NE solve and SVD-reduced NE solve disagree")

    drift_lag = np.linalg.solve(M_lag, -g_lag)
    input_lag = np.linalg.solve(M_lag, B_lag)
    return dict(
        D_ne_full=D_ne_full, g_ne_full=g_ne_full,
        B_ne_full=B_ne_full, M_ne=M_ne, g_ne=g_ne, B_ne=B_ne,
        N_force=N_force, M_lag=M_lag, g_lag=g_lag, B_lag=B_lag,
        rank_full=rank_full, condition_full=condition_full,
        drift_ne=drift_ne, input_ne=input_ne,
        drift_lag=drift_lag, input_lag=input_lag,
        drift_diff=drift_ne-drift_lag,
        input_diff=input_ne-input_lag)


def search_fixed_body_tail_symmetric_equilibria(s, parameter_values):
    """Find equilibria with theta_b=theta_t=0 and symmetric legs/hip torques.

    The wheel torques are fixed to zero.  A multi-start solve finds the common
    leg angle, common left/right hip torque, and tail torque.  The candidate
    closest to the source-code leg angle theta_s=0.3 is selected.
    """
    fixed = {s[name]: value for name, value in parameter_values.items()}
    fixed.update({s['X_b_h']: 0., s['phi']: 0.,
                  s['theta_b']: 0., s['theta_t']: 0.})
    fixed.update({symbol: 0. for symbol in s['dq']})
    theta_s = s['theta_l']
    symmetric = {s['theta_r']: theta_s}
    h_symmetric = s['g_lag'].subs(fixed).subs(symmetric)
    B_symmetric = s['B_lag'].subs(fixed).subs(symmetric)
    tau_hip, tau_tail = sp.symbols(
        'tau_hip_equilibrium tau_tail_equilibrium', real=True)
    u_symmetric = sp.Matrix([tau_hip, tau_hip, 0, 0, tau_tail])
    full_residual_symbolic = h_symmetric-B_symmetric*u_symmetric
    # theta_s is one reduced coordinate, so the left/right leg residuals add.
    reduced_residual = sp.Matrix([
        full_residual_symbolic[2] + full_residual_symbolic[3],
        full_residual_symbolic[4],
        full_residual_symbolic[5],
    ])
    unknowns = sp.Matrix([theta_s, tau_hip, tau_tail])
    allowed = set(unknowns)
    unresolved = (reduced_residual.free_symbols
                  | full_residual_symbolic.free_symbols) - allowed
    if unresolved:
        names = ", ".join(sorted(str(symbol) for symbol in unresolved))
        raise ValueError(f"equilibrium formulas contain symbols: {names}")

    residual_fun = sp.lambdify(tuple(unknowns), reduced_residual, modules='numpy')
    full_residual_fun = sp.lambdify(
        tuple(unknowns), full_residual_symbolic, modules='numpy')

    def residual(z):
        return np.asarray(residual_fun(*z), dtype=float).reshape(3)

    def wrap_angle(angle):
        return (float(angle) + np.pi) % (2*np.pi) - np.pi

    # Classify local stiffness on the symmetric shape subspace while holding
    # the equilibrium actuator torques constant.
    theta_b, theta_t = s['theta_b'], s['theta_t']
    shape_fixed = {s[name]: value for name, value in parameter_values.items()}
    shape_fixed.update({s['X_b_h']: 0., s['phi']: 0.})
    shape_fixed.update({symbol: 0. for symbol in s['dq']})
    h_shape = s['g_lag'].subs(shape_fixed).subs(symmetric)
    B_shape = s['B_lag'].subs(shape_fixed).subs(symmetric)
    shape_full_residual = h_shape-B_shape*u_symmetric
    shape_residual = sp.Matrix([
        shape_full_residual[2]+shape_full_residual[3],
        shape_full_residual[4], shape_full_residual[5]])
    shape_coordinates = sp.Matrix([theta_s, theta_b, theta_t])
    stiffness = shape_residual.jacobian(shape_coordinates)
    stiffness_fun = sp.lambdify(
        (theta_s, theta_b, theta_t, tau_hip, tau_tail),
        stiffness, modules='numpy')

    def stability_label(theta_value, hip_value, tail_value):
        K = np.asarray(stiffness_fun(
            theta_value, 0., 0., hip_value, tail_value),
            dtype=float).reshape(3, 3)
        eigenvalues = np.linalg.eigvalsh((K + K.T) / 2)
        tolerance = 1e-8 * max(1., np.max(np.abs(eigenvalues)))
        if np.all(eigenvalues > tolerance):
            label = "stable"
        elif np.all(eigenvalues < -tolerance):
            label = "unstable_maximum"
        elif np.any(eigenvalues > tolerance) and np.any(eigenvalues < -tolerance):
            label = "saddle"
        else:
            label = "degenerate"
        return label, eigenvalues

    reference_angle = .3
    seed_angles = np.r_[reference_angle, np.linspace(-2.8, 2.8, 17)]
    candidates = []
    for seed_angle in seed_angles:
        solution = least_squares(
            residual, np.array([seed_angle, 1., -1.]),
            bounds=(np.array([-np.pi, -500., -500.]),
                    np.array([np.pi, 500., 500.])),
            x_scale='jac', ftol=1e-12, xtol=1e-12, gtol=1e-12,
            max_nfev=500)
        values = solution.x.copy()
        values[0] = wrap_angle(values[0])
        residual_value = residual(values)
        full_residual_value = np.asarray(
            full_residual_fun(*values), dtype=float).reshape(6)
        if (not solution.success
                or np.linalg.norm(residual_value, np.inf) > 1e-8
                or np.linalg.norm(full_residual_value, np.inf) > 1e-8):
            continue
        if any(abs(wrap_angle(values[0]-item['theta_s'])) < 1e-6
               for item in candidates):
            continue
        theta_value, hip_value, tail_value = values
        label, eigenvalues = stability_label(
            theta_value, hip_value, tail_value)
        q_value = np.array([
            0., 0., theta_value, theta_value, 0., 0.])
        u_value = np.array([
            hip_value, hip_value, 0., 0., tail_value])
        candidates.append(dict(
            theta_s=theta_value, angles=q_value[[2, 4, 5]],
            q=q_value, u=u_value, residual=residual_value,
            full_residual=full_residual_value,
            stability=label, stiffness_eigenvalues=eigenvalues,
            distance_to_reference=abs(wrap_angle(
                theta_value-reference_angle)),
            optimizer_cost=solution.cost,
            optimizer_optimality=solution.optimality,
            optimizer_nfev=solution.nfev))

    if not candidates:
        raise RuntimeError(
            "no equilibrium was found with theta_b=theta_t=0, symmetric "
            "legs/hip torques, and zero wheel torques")
    candidates.sort(key=lambda item: item['distance_to_reference'])
    selected = candidates[0]
    return dict(selected=selected, candidates=candidates,
                dq=np.zeros(6), u=selected['u'],
                reference=np.array([reference_angle, 0., 0.]))


def linearize_explicit_dynamics(s, parameter_values, q_eq, dq_eq, u_eq,
                                relative_step=1e-5):
    """Numerically linearize both explicit dynamics at a controlled equilibrium.

    For x=[q; dq], the acceleration field is a(x,u)=d(x)+G(x)u.  Central
    differences are applied to the complete field at the nonzero equilibrium
    input, so derivatives of M, h, and B are all included automatically.
    """
    q_eq = np.asarray(q_eq, dtype=float).reshape(6)
    dq_eq = np.asarray(dq_eq, dtype=float).reshape(6)
    u_eq = np.asarray(u_eq, dtype=float).reshape(5, 1)
    x_eq = np.r_[q_eq, dq_eq]
    A_ne = np.zeros((12, 12))
    A_lag = np.zeros((12, 12))
    A_ne[:6, 6:] = np.eye(6)
    A_lag[:6, 6:] = np.eye(6)
    steps = relative_step*np.maximum(1., np.abs(x_eq))

    for column, step in enumerate(steps):
        print(f"  中心差分进度: {column+1}/12")
        x_plus = x_eq.copy()
        x_minus = x_eq.copy()
        x_plus[column] += step
        x_minus[column] -= step
        plus = explicit_dynamics_at(
            s, state_substitutions(
                s, parameter_values, x_plus[:6], x_plus[6:]))
        minus = explicit_dynamics_at(
            s, state_substitutions(
                s, parameter_values, x_minus[:6], x_minus[6:]))
        acceleration_ne_plus = plus['drift_ne']+plus['input_ne']@u_eq
        acceleration_ne_minus = minus['drift_ne']+minus['input_ne']@u_eq
        acceleration_lag_plus = plus['drift_lag']+plus['input_lag']@u_eq
        acceleration_lag_minus = minus['drift_lag']+minus['input_lag']@u_eq
        A_ne[6:, column] = (
            (acceleration_ne_plus-acceleration_ne_minus)/(2*step)).ravel()
        A_lag[6:, column] = (
            (acceleration_lag_plus-acceleration_lag_minus)/(2*step)).ravel()

    equilibrium_result = explicit_dynamics_at(
        s, state_substitutions(s, parameter_values, q_eq, dq_eq))
    B_ss_ne = np.vstack([np.zeros((6, 5)), equilibrium_result['input_ne']])
    B_ss_lag = np.vstack([np.zeros((6, 5)), equilibrium_result['input_lag']])
    eigenvalues_ne = np.linalg.eigvals(A_ne)
    eigenvalues_lag = np.linalg.eigvals(A_lag)
    controllability_ne = controllability(A_ne, B_ss_ne)
    controllability_lag = controllability(A_lag, B_ss_lag)
    controllability_singular_ne = np.linalg.svd(
        controllability_ne, compute_uv=False)
    controllability_singular_lag = np.linalg.svd(
        controllability_lag, compute_uv=False)
    return dict(
        x_eq=x_eq, u_eq=u_eq, finite_difference_steps=steps,
        A_ne=A_ne, A_lag=A_lag, B_ss_ne=B_ss_ne, B_ss_lag=B_ss_lag,
        A_diff=A_ne-A_lag, B_ss_diff=B_ss_ne-B_ss_lag,
        eigenvalues_ne=eigenvalues_ne, eigenvalues_lag=eigenvalues_lag,
        controllability_ne=controllability_ne,
        controllability_lag=controllability_lag,
        controllability_rank_ne=np.linalg.matrix_rank(controllability_ne),
        controllability_rank_lag=np.linalg.matrix_rank(controllability_lag),
        controllability_singular_ne=controllability_singular_ne,
        controllability_singular_lag=controllability_singular_lag)


def main(scan_leg_lengths=False, scan_start=.15, scan_stop=.30,
         scan_step=.005, scan_output='lqr_fit_samples_0821_Lag.mat'):
    print("="*64)
    print("动力学修正版：Newton--Euler / Euler--Lagrange 对比")
    print("="*64)
    print("使用几何髋中点速度；修正 COM 加速度、运动点矩方程、重力和执行器符号。")
    print("I_yaw 解释为机体中心组件惯量；NE yaw 力矩由左右髋纵向力之差产生。")
    print("正在建立 19 条刚体方程、1 条机体无 roll 力矩闭合方程并自动消元...")
    s = symbolic_model()
    subs, parameter_values = numeric_substitutions(s)
    if scan_leg_lengths:
        if scan_step <= 0 or scan_stop < scan_start:
            raise ValueError('scan range must satisfy stop >= start and step > 0')
        l_range = np.arange(scan_start, scan_stop+.5*scan_step, scan_step)
        print("\n开始 Lagrange 多腿长扫描")
        print(f"范围: {l_range[0]:.4f}:{scan_step:.4f}:{l_range[-1]:.4f} "
              f"({len(l_range)**2} points)")
        scan_lagrange_leg_lengths(s, parameter_values, l_range, scan_output)
        return
    force_cancel = simplify_matrix(s['elimination_matrix']*s['A_force'])
    if nonzero_count(force_cancel):
        raise RuntimeError("内力/接触力消元自检失败")
    print("消元自检通过：6 条最终 NE 方程不含 14 个力变量。")
    dynamic_result = explicit_dynamics_at(s, subs)
    drift_ne = dynamic_result['drift_ne']
    input_ne = dynamic_result['input_ne']
    drift_lag = dynamic_result['drift_lag']
    input_lag = dynamic_result['input_lag']
    drift_diff = dynamic_result['drift_diff']
    input_diff = dynamic_result['input_diff']
    rank_full = dynamic_result['rank_full']
    condition_full = dynamic_result['condition_full']
    print("\n非对称测试点: theta_l=0.27, theta_r=0.34 rad，dq 非零，u=0")
    print(f"完整 NE 系统: rank={rank_full}/20, condition={condition_full:.3e}")
    print_matrix("NE 零输入广义加速度", drift_ne)
    print_matrix("Lagrange 零输入广义加速度", drift_lag)
    print(f"||qdd_NE-qdd_Lag||_inf = {np.linalg.norm(drift_diff, np.inf):.6e}")
    print(f"||M_NE^-1 B_NE-M_Lag^-1 B_Lag||_inf = "
          f"{np.linalg.norm(input_diff, np.inf):.6e}")

    # Search for equilibria with theta_b=theta_t=0, symmetric legs and hip
    # torques, zero wheel torques, and a free equilibrium tail torque.
    equilibrium = search_fixed_body_tail_symmetric_equilibria(
        s, parameter_values)
    q0 = equilibrium['selected']['q']
    dq0 = equilibrium['dq']
    u0 = equilibrium['u'].reshape(5, 1)
    print("\n固定 theta_b=theta_t=0 的对称静态平衡点搜索")
    print("约束: theta_l=theta_r=theta_s, tau_hip_l=tau_hip_r, "
          "tau_motor_l=tau_motor_r=0")
    print(f"在 theta_s 属于 [-pi, pi] 中找到 "
          f"{len(equilibrium['candidates'])} 个不重复平衡点：")
    for index, candidate in enumerate(equilibrium['candidates'], start=1):
        print(f"  {index:2d}: theta_s={candidate['theta_s']:.8f}, "
              f"tau_hip={candidate['u'][0]:.8f}, "
              f"tau_tail={candidate['u'][4]:.8f}, "
              f"{candidate['stability']}, "
              f"distance={candidate['distance_to_reference']:.6e}, "
              f"||h-Bu||_inf="
              f"{np.linalg.norm(candidate['full_residual'], np.inf):.3e}")
    print("选择距离原始腿角 theta_s=0.3 最近的平衡点进行邻域比较。")
    print(f"q_eq = {np.array2string(q0, precision=8)}")
    print(f"u_eq = {np.array2string(u0.ravel(), precision=8)}")
    print(f"局部势能曲率特征值 = "
          f"{np.array2string(equilibrium['selected']['stiffness_eigenvalues'], precision=8)}")
    print(f"稳定性分类 = {equilibrium['selected']['stability']}")
    print(f"||h_Lag(q_eq,0)-B_Lag(q_eq)*u_eq||_inf = "
          f"{np.linalg.norm(equilibrium['selected']['full_residual'], np.inf):.6e}")
    eps = 1e-2
    neighborhood_cases = [
        ("搜索到的平衡点", q0, dq0),
        ("左右腿对称扰动", q0 + np.array([0, 0, eps, eps, 0, 0]), dq0),
        ("左右腿反对称扰动", q0 + np.array([0, 0, eps, -eps, 0, 0]), dq0),
        ("机身俯仰扰动", q0 + np.array([0, 0, 0, 0, eps, 0]), dq0),
        ("尾巴角扰动", q0 + np.array([0, 0, 0, 0, 0, eps]), dq0),
        ("yaw 扰动", q0 + np.array([0, eps, 0, 0, 0, 0]), dq0),
        ("小速度扰动", q0,
         np.array([eps, -eps, eps, -eps, eps, -eps])),
    ]
    neighborhood_results = []
    print("\n静态平衡点及其邻域的显式动力学对比")
    print("比较量: d=-M^-1 h, G=M^-1 B")
    print(f"{'case':<22} {'||d_NE-d_Lag||_inf':>23} "
          f"{'||G_NE-G_Lag||_inf':>23} "
          f"{'||a_NE-a_Lag||_inf':>23} {'cond(D_NE)':>14}")
    for case_name, q_values, dq_values in neighborhood_cases:
        case_subs = state_substitutions(
            s, parameter_values, q_values, dq_values)
        result = explicit_dynamics_at(s, case_subs)
        result['acceleration_ne_u0'] = result['drift_ne'] + result['input_ne'] @ u0
        result['acceleration_lag_u0'] = result['drift_lag'] + result['input_lag'] @ u0
        result['acceleration_diff_u0'] = (
            result['acceleration_ne_u0']-result['acceleration_lag_u0'])
        neighborhood_results.append(result)
        print(f"{case_name:<22} "
              f"{np.linalg.norm(result['drift_diff'], np.inf):23.6e} "
              f"{np.linalg.norm(result['input_diff'], np.inf):23.6e} "
              f"{np.linalg.norm(result['acceleration_diff_u0'], np.inf):23.6e} "
              f"{result['condition_full']:14.6e}")

    equilibrium_result = neighborhood_results[0]
    print_matrix("平衡点 NE 的 -M^-1 h", equilibrium_result['drift_ne'])
    print_matrix("平衡点 Lagrange 的 -M^-1 h", equilibrium_result['drift_lag'])
    print_matrix("平衡点 NE 的 M^-1 B", equilibrium_result['input_ne'])
    print_matrix("平衡点 Lagrange 的 M^-1 B", equilibrium_result['input_lag'])
    print_matrix("平衡输入下 NE 的广义加速度",
                 equilibrium_result['acceleration_ne_u0'])
    print_matrix("平衡输入下 Lagrange 的广义加速度",
                 equilibrium_result['acceleration_lag_u0'])
    print(f"平衡点检查: ||ddq_NE||_inf="
          f"{np.linalg.norm(equilibrium_result['acceleration_ne_u0'], np.inf):.6e}, "
          f"||ddq_Lag||_inf="
          f"{np.linalg.norm(equilibrium_result['acceleration_lag_u0'], np.inf):.6e}")

    print("\n在搜索到的受控平衡点处构造状态空间模型")
    print("x=[q;dq], delta_x_dot=A*delta_x+B_ss*delta_u")
    print("对完整 a(x,u_eq)=-M^-1*h+M^-1*B*u_eq 进行中心差分...")
    linearization = linearize_explicit_dynamics(
        s, parameter_values, q0, dq0, u0)
    print_matrix("NE 状态矩阵 A (12x12)",
                 linearization['A_ne'], fmt="12.5e")
    print_matrix("Lagrange 状态矩阵 A (12x12)",
                 linearization['A_lag'], fmt="12.5e")
    print(f"||A_NE-A_Lag||_inf = "
          f"{np.linalg.norm(linearization['A_diff'], np.inf):.6e}")
    print_matrix("NE 输入矩阵 B_ss (12x5)",
                 linearization['B_ss_ne'], fmt="12.5e")
    print_matrix("Lagrange 输入矩阵 B_ss (12x5)",
                 linearization['B_ss_lag'], fmt="12.5e")
    print(f"||B_ss_NE-B_ss_Lag||_inf = "
          f"{np.linalg.norm(linearization['B_ss_diff'], np.inf):.6e}")

    eig_ne = np.sort_complex(linearization['eigenvalues_ne'])
    eig_lag = np.sort_complex(linearization['eigenvalues_lag'])
    print("--- 平衡点 A 矩阵特征值 ---")
    print(f"{'No.':>4} | {'NE':>27} | {'Lagrange':>27}")
    for index, (value_ne, value_lag) in enumerate(
            zip(eig_ne, eig_lag), start=1):
        print(f"{index:4d} | "
              f"{value_ne.real:12.5e}{value_ne.imag:+12.5e}j | "
              f"{value_lag.real:12.5e}{value_lag.imag:+12.5e}j")
    print("可控性矩阵数值秩: "
          f"NE={linearization['controllability_rank_ne']}/12, "
          f"Lagrange={linearization['controllability_rank_lag']}/12")
    print("注：可控性矩阵含 A 的高次幂，秩对单位和数值尺度敏感，"
          "同时保存了其奇异值供复核。")

    lag_skew = simplify_matrix(s['M_lag']-s['M_lag'].T)
    print("Lagrange 质量矩阵对称性自检："+
          ("通过" if nonzero_count(lag_skew)==0 else "失败"))
    print("注意：NE 与 Lagrange 的左右轮/髋运动学现已对齐。若显式动力学仍有差异，")
    print("应继续检查无 roll 运动学一致性和髋关节横向力臂，而不是直接比较原始方程行。")

    # Export a MATLAB-compatible interface.  Keep both model variants so the
    # downstream script can choose Lag or NE without rebuilding symbolic
    # equations.  All exported state-space matrices use the legacy order:
    # [X,dX,phi,dphi,theta_l,dtheta_l,theta_r,dtheta_r,theta_b,dtheta_b,
    #  theta_t,dtheta_t].
    A_lag_int, B_lag_int, state_permutation = interleave_state_space(
        linearization['A_lag'], linearization['B_ss_lag'])
    A_ne_int, B_ne_int, _ = interleave_state_space(
        linearization['A_ne'], linearization['B_ss_ne'])
    x_eq_int = linearization['x_eq'][state_permutation]
    u_eq_int = linearization['u_eq'].reshape(5)
    state_order_int = np.array([
        'X_b_h', 'dX_b_h', 'phi', 'dphi',
        'theta_l', 'dtheta_l', 'theta_r', 'dtheta_r',
        'theta_b', 'dtheta_b', 'theta_t', 'dtheta_t'], dtype=object)
    input_order = np.array([
        'T_hip_r', 'T_hip_l', 'T_motor_wr', 'T_motor_wl', 'T_tail'],
        dtype=object)
    savemat('lqr_input_0821.mat', dict(
        A_num=A_lag_int, B_num=B_lag_int,
        A_Lag_int=A_lag_int, B_ss_Lag_int=B_lag_int,
        A_NE_int=A_ne_int, B_ss_NE_int=B_ne_int,
        A_Lag_qd=linearization['A_lag'], B_ss_Lag_qd=linearization['B_ss_lag'],
        A_NE_qd=linearization['A_ne'], B_ss_NE_qd=linearization['B_ss_ne'],
        X_eq=x_eq_int, U_eq=u_eq_int,
        equilibrium_q=q0, equilibrium_dq=dq0, equilibrium_u=u0,
        state_order=state_order_int, input_order=input_order,
        state_permutation=state_permutation + 1,
        selected_model='Lagrange'))
    print("MATLAB 输入已保存到 lqr_input_0821.mat")

    savemat('comparison_results_dynamics_corrected.mat', dict(
        M_NE_sym=symbolic_strings(s['M_ne']), g_NE_sym=symbolic_strings(s['g_ne']), B_NE_sym=symbolic_strings(s['B_ne']),
        M_Lag_sym=symbolic_strings(s['M_lag']), g_Lag_sym=symbolic_strings(s['g_lag']), B_Lag_sym=symbolic_strings(s['B_lag']),
        M_NE_num=dynamic_result['M_ne'], M_Lag_num=dynamic_result['M_lag'],
        g_NE_num=dynamic_result['g_ne'], g_Lag_num=dynamic_result['g_lag'],
        B_NE_num=dynamic_result['B_ne'], B_Lag_num=dynamic_result['B_lag'],
        D_NE_full_num=dynamic_result['D_ne_full'],
        g_NE_full_num=dynamic_result['g_ne_full'],
        B_NE_full_num=dynamic_result['B_ne_full'],
        force_left_null_num=dynamic_result['N_force'],
        NE_full_rank=rank_full, NE_full_condition=condition_full,
        qdd_drift_NE=drift_ne, qdd_drift_Lag=drift_lag,
        input_map_NE=input_ne, input_map_Lag=input_lag,
        drift_diff=drift_diff, input_map_diff=input_diff,
        param_val=parameter_values, theta_l_test=.27, theta_r_test=.34,
        dq_test=np.array([.12, -.07, .09, -.05, .04, -.06]),
        equilibrium_q=q0, equilibrium_dq=dq0, equilibrium_u=u0,
        equilibrium_Lag_equation_residual=
            equilibrium['selected']['full_residual'],
        equilibrium_NE_acceleration_residual=
            equilibrium_result['acceleration_ne_u0'],
        equilibrium_Lag_acceleration_residual=
            equilibrium_result['acceleration_lag_u0'],
        equilibrium_stability=equilibrium['selected']['stability'],
        equilibrium_stiffness_eigenvalues=
            equilibrium['selected']['stiffness_eigenvalues'],
        equilibrium_optimizer_cost=equilibrium['selected']['optimizer_cost'],
        equilibrium_optimizer_optimality=
            equilibrium['selected']['optimizer_optimality'],
        equilibrium_optimizer_nfev=equilibrium['selected']['optimizer_nfev'],
        all_equilibrium_angles=np.vstack([
            item['angles'] for item in equilibrium['candidates']]),
        all_equilibrium_inputs=np.vstack([
            item['u'] for item in equilibrium['candidates']]),
        all_equilibrium_stability=np.array([
            item['stability'] for item in equilibrium['candidates']], dtype=object),
        all_equilibrium_stiffness_eigenvalues=np.vstack([
            item['stiffness_eigenvalues'] for item in equilibrium['candidates']]),
        neighborhood_case_names=np.array(
            [case[0] for case in neighborhood_cases], dtype=object),
        neighborhood_q=np.vstack([case[1] for case in neighborhood_cases]),
        neighborhood_dq=np.vstack([case[2] for case in neighborhood_cases]),
        neighborhood_drift_NE=np.stack(
            [result['drift_ne'].ravel() for result in neighborhood_results]),
        neighborhood_drift_Lag=np.stack(
            [result['drift_lag'].ravel() for result in neighborhood_results]),
        neighborhood_input_map_NE=np.stack(
            [result['input_ne'] for result in neighborhood_results]),
        neighborhood_input_map_Lag=np.stack(
            [result['input_lag'] for result in neighborhood_results]),
        neighborhood_acceleration_NE_u_eq=np.stack([
            result['acceleration_ne_u0'].ravel()
            for result in neighborhood_results]),
        neighborhood_acceleration_Lag_u_eq=np.stack([
            result['acceleration_lag_u0'].ravel()
            for result in neighborhood_results]),
        neighborhood_acceleration_error_inf=np.array([
            np.linalg.norm(result['acceleration_diff_u0'], np.inf)
            for result in neighborhood_results]),
        neighborhood_drift_error_inf=np.array([
            np.linalg.norm(result['drift_diff'], np.inf)
            for result in neighborhood_results]),
        neighborhood_input_error_inf=np.array([
            np.linalg.norm(result['input_diff'], np.inf)
            for result in neighborhood_results]),
        neighborhood_NE_condition=np.array([
            result['condition_full'] for result in neighborhood_results]),
        linearization_x_eq=linearization['x_eq'],
        linearization_u_eq=linearization['u_eq'],
        linearization_finite_difference_steps=
            linearization['finite_difference_steps'],
        A_NE=linearization['A_ne'], A_Lag=linearization['A_lag'],
        A_state_space_diff=linearization['A_diff'],
        B_ss_NE=linearization['B_ss_ne'],
        B_ss_Lag=linearization['B_ss_lag'],
        B_ss_diff=linearization['B_ss_diff'],
        eig_A_NE=linearization['eigenvalues_ne'],
        eig_A_Lag=linearization['eigenvalues_lag'],
        controllability_NE=linearization['controllability_ne'],
        controllability_Lag=linearization['controllability_lag'],
        controllability_rank_NE=linearization['controllability_rank_ne'],
        controllability_rank_Lag=linearization['controllability_rank_lag'],
        controllability_singular_values_NE=
            linearization['controllability_singular_ne'],
        controllability_singular_values_Lag=
            linearization['controllability_singular_lag']))
    print("结果已保存到 comparison_results_dynamics_corrected.mat")



if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Corrected dynamics, legacy-order LQR export, and optional Lagrange leg-length scan.')
    parser.add_argument('--scan-leg-lengths', action='store_true',
                        help='generate Lagrange K samples and 60x6 poly22 fits')
    parser.add_argument('--scan-start', type=float, default=.15)
    parser.add_argument('--scan-stop', type=float, default=.30)
    parser.add_argument('--scan-step', type=float, default=.005)
    parser.add_argument('--scan-output', default='lqr_fit_samples_0821_Lag.mat')
    arguments = parser.parse_args()
    main(arguments.scan_leg_lengths, arguments.scan_start, arguments.scan_stop,
         arguments.scan_step, arguments.scan_output)
