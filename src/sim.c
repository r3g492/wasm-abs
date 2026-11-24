// Vehicle braking simulation: planar 3-DOF body + 4 wheel spin DOFs,
// quasi-static load transfer, rate-limited brake hydraulics, Burckhardt tires.
#include "sim.h"
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Data
// ---------------------------------------------------------------------------
const SurfaceParams SURFACES[SURF_COUNT] = {
    [SURF_DRY]   = { "Dry asphalt",    1.2801, 23.99,  0.52  },  // peak ~1.17, locked ~0.76
    [SURF_WET]   = { "Wet asphalt",    0.857,  33.822, 0.347 },  // peak ~0.80, locked ~0.51
    [SURF_WATER] = { "Standing water", 0.40,   33.8,   0.12  },  // peak ~0.38, locked ~0.28
    [SURF_ICE]   = { "Ice",            0.13,   110.0,  0.05  },  // peak ~0.13, locked ~0.08
};

const Scenario SCENARIOS[SCENARIO_COUNT] = {
    { "Dry asphalt",          "Uniform high-mu",                         100, SURF_DRY, SURF_DRY   },
    { "Rain",                 "Uniform wet asphalt",                     100, SURF_WET, SURF_WET   },
    { "Split-mu: water right","Dry left, standing water under right wheels", 80, SURF_DRY, SURF_WATER },
    { "Ice",                  "Uniform ice",                              50, SURF_ICE, SURF_ICE   },
    { "Split-mu: ice right",  "Dry left, ice under right wheels",         80, SURF_DRY, SURF_ICE   },
};

// Vehicle parameters
#define G           9.81
#define MASS        1500.0
#define IZ          2400.0
#define H_CG        0.55
#define IW          1.1         // wheel spin inertia (kg m^2)
#define KB_FRONT    20.0        // brake torque gain (Nm/bar)
#define KB_REAR     9.0
#define P_MAX       160.0       // master cylinder pressure at full pedal (bar)
#define P_TAU       0.008       // hydraulic time constant (s)
#define P_BUILD     1500.0      // max pressure build rate (bar/s)
#define P_DUMP      3000.0      // max pressure dump rate (bar/s)
#define PEDAL_RATE  7.0         // pedal apply rate (1/s) -> full in ~0.14 s
#define RUNUP_TIME  1.0         // seconds of cruising before the brake line
#define V_STOP      0.1         // run ends below this speed (m/s)
#define SLIP_VFLOOR 1.0         // slip denominator floor (m/s)
#define PI_D        3.14159265358979323846

static const double WX[4] = {  CAR_A,  CAR_A, -CAR_B, -CAR_B };
static const double WY[4] = {  CAR_TRACK/2, -CAR_TRACK/2, CAR_TRACK/2, -CAR_TRACK/2 };

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static double Clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

static double MuCurve(Surface surf, double s)
{
    const SurfaceParams *p = &SURFACES[surf];
    if (s > 1.0) s = 1.0;
    double mu = p->c1*(1.0 - exp(-p->c2*s)) - p->c3*s;
    return mu > 0 ? mu : 0;
}

// Tire force in the wheel frame from slip velocities.
static void TireForce(Surface surf, double Fz, double omegaR, double vxw, double vyw,
                      double *Fx, double *Fy, double *slipX, double *mu)
{
    double den = fabs(vxw) > SLIP_VFLOOR ? fabs(vxw) : SLIP_VFLOOR;
    double sx = (omegaR - vxw)/den;     // longitudinal slip (negative when braking)
    double sy = -vyw/den;               // ~ -tan(alpha)
    double sm = sqrt(sx*sx + sy*sy);
    *slipX = sx;
    if (sm < 1e-9) { *Fx = 0; *Fy = 0; *mu = 0; return; }
    double m = MuCurve(surf, sm);
    *mu = m;
    *Fx = m*Fz*sx/sm;
    *Fy = m*Fz*sy/sm;
}

Surface SimSurfaceAt(const Sim *s, double x, double y)
{
    if (x < 0) return SURF_DRY;
    const Scenario *sc = &SCENARIOS[s->scenario];
    return y >= 0 ? sc->left : sc->right;
}

void SimWheelPos(const Sim *s, int i, double *wx, double *wy)
{
    double c = cos(s->psi), sn = sin(s->psi);
    *wx = s->x + WX[i]*c - WY[i]*sn;
    *wy = s->y + WX[i]*sn + WY[i]*c;
}

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------
void SimReset(Sim *s, int scenario, int absOn, DriverMode driver, double v0Kmh, AbsFn fn, void *user)
{
    memset(s, 0, sizeof(*s));
    s->scenario = scenario;
    s->absOn = absOn;
    s->driver = driver;
    s->v0 = v0Kmh/3.6;
    s->absFn = fn;
    s->absUser = user;

    s->x = -s->v0*RUNUP_TIME;
    s->vx = s->v0;
    for (int i = 0; i < 4; i++) s->omega[i] = s->v0/CAR_R;
    s->phase = PHASE_RUNUP;
}

static void RunController(Sim *s)
{
    float in[ABS_IN_COUNT], out[4];
    in[ABS_IN_T] = (float)s->tBrake;
    in[ABS_IN_DT] = (float)SIM_CTRL_DT;
    in[ABS_IN_PEDAL] = (float)s->pedal;
    in[ABS_IN_PMASTER] = (float)s->pm;
    for (int i = 0; i < 4; i++) {
        in[ABS_IN_WSPEED + i] = (float)(s->omega[i]*CAR_R);
        in[ABS_IN_WPRESS + i] = (float)s->p[i];
    }
    in[ABS_IN_AX] = (float)s->axS;
    in[ABS_IN_AY] = (float)s->ayS;
    in[ABS_IN_YAWRATE] = (float)s->r;
    in[ABS_IN_STEER] = (float)s->steer;
    in[ABS_IN_VX_TRUE] = (float)s->vx;

    int ok = 0;
    if (s->absOn && !s->absFault && s->absFn) {
        ok = s->absFn(in, out, s->absUser);
        for (int i = 0; ok && i < 4; i++) if (!isfinite(out[i])) ok = 0;
        if (!ok) s->absFault = 1;
        s->absCalls++;
    }
    for (int i = 0; i < 4; i++) s->pcmd[i] = ok ? out[i] : s->pm;
}

static void UpdateSkid(Sim *s, int i)
{
    double wx, wy;
    SimWheelPos(s, i, &wx, &wy);
    int sliding = fabs(s->slip[i]) > 0.25 && s->Fz[i] > 0;
    if (!sliding) { s->skidActive[i] = 0; return; }
    if (s->skidCount[i] >= SIM_SKID_MAX) return;
    float dx = (float)wx - s->skidLastX[i], dy = (float)wy - s->skidLastY[i];
    if (s->skidActive[i] && dx*dx + dy*dy < 0.15f*0.15f) return;
    SkidPoint *p = &s->skid[i][s->skidCount[i]++];
    p->x = (float)wx; p->y = (float)wy;
    p->surf = (unsigned char)s->surf[i];
    p->start = !s->skidActive[i];
    s->skidActive[i] = 1;
    s->skidLastX[i] = (float)wx; s->skidLastY[i] = (float)wy;
}

void SimStep(Sim *s)
{
    if (s->phase == PHASE_STOPPED) return;
    const double dt = SIM_DT;

    // --- Driver -----------------------------------------------------------
    if (s->phase == PHASE_RUNUP && s->x >= 0) {
        s->phase = PHASE_BRAKING;
        s->ctrlTimer = 0;
        s->telTimer = 0;
    }
    if (s->phase == PHASE_BRAKING) {
        s->pedal = fmin(1.0, s->pedal + PEDAL_RATE*dt);
        if (s->driver == DRIVER_CORRECT) {
            // Human-like counter-steer: heading + yaw rate + lateral drift, slow and limited
            double target = -(1.2*s->psi + 0.25*s->r + 0.04*s->y);
            target = Clamp(target, -0.12, 0.12);
            double rate = Clamp((target - s->steer)/0.15, -0.5, 0.5);
            s->steer += rate*dt;
        }
    }
    s->pm = s->pedal*P_MAX;

    // --- ABS controller (fixed rate) --------------------------------------
    if (s->phase == PHASE_BRAKING) {
        s->ctrlTimer -= dt;
        if (s->ctrlTimer <= 0) { RunController(s); s->ctrlTimer += SIM_CTRL_DT; }
    } else {
        for (int i = 0; i < 4; i++) s->pcmd[i] = 0;
    }

    // --- Hydraulics: valves can only reduce below master pressure ---------
    for (int i = 0; i < 4; i++) {
        double cmd = Clamp(s->pcmd[i], 0, s->pm);
        double rate = Clamp((cmd - s->p[i])/P_TAU, -P_DUMP, P_BUILD);
        s->p[i] = fmax(0, s->p[i] + rate*dt);
    }

    // --- Vertical loads (quasi-static transfer from sensed accelerations) --
    const double L = CAR_A + CAR_B;
    double Ff = MASS*(G*CAR_B - s->axS*H_CG)/L;
    double Fr = MASS*(G*CAR_A + s->axS*H_CG)/L;
    double dLat = MASS*s->ayS*H_CG/CAR_TRACK;
    s->Fz[0] = Ff/2 - dLat*CAR_B/L;
    s->Fz[1] = Ff/2 + dLat*CAR_B/L;
    s->Fz[2] = Fr/2 - dLat*CAR_A/L;
    s->Fz[3] = Fr/2 + dLat*CAR_A/L;
    for (int i = 0; i < 4; i++) if (s->Fz[i] < 50) s->Fz[i] = 50;

    // --- Tires & wheels ---------------------------------------------------
    double FxSum = 0, FySum = 0, Mz = 0;
    for (int i = 0; i < 4; i++) {
        double wx, wy;
        SimWheelPos(s, i, &wx, &wy);
        s->surf[i] = SimSurfaceAt(s, wx, wy);

        double delta = i < 2 ? s->steer : 0;
        double cd = cos(delta), sd = sin(delta);
        double vbx = s->vx - s->r*WY[i];
        double vby = s->vy + s->r*WX[i];
        double vxw =  vbx*cd + vby*sd;
        double vyw = -vbx*sd + vby*cd;

        double fx, fy, sx, mu;
        TireForce(s->surf[i], s->Fz[i], s->omega[i]*CAR_R, vxw, vyw, &fx, &fy, &sx, &mu);
        s->slip[i] = sx; s->mu[i] = mu;

        // Wheel spin: linearised backward Euler, brake acts as a Coulomb clutch
        double Tb = (i < 2 ? KB_FRONT : KB_REAR)*s->p[i];
        double Tt = -CAR_R*fx;
        if (s->omega[i] > 1e-9 || Tt > Tb) {
            const double h = 0.01;
            double fx2, fy2, sx2, mu2;
            TireForce(s->surf[i], s->Fz[i], (s->omega[i] + h)*CAR_R, vxw, vyw, &fx2, &fy2, &sx2, &mu2);
            double f = (Tt - Tb)/IW;
            double fp = (-CAR_R*fx2 - Tt)/h/IW;
            double den = 1.0 - dt*fp;
            if (den < 0.5) den = 0.5;
            s->omega[i] += dt*f/den;
            if (s->omega[i] < 0) s->omega[i] = 0;
        }
        s->spin[i] = fmod(s->spin[i] + s->omega[i]*dt, 2*PI_D);

        s->Fx[i] = fx*cd - fy*sd;
        s->Fy[i] = fx*sd + fy*cd;
        FxSum += s->Fx[i];
        FySum += s->Fy[i];
        Mz += WX[i]*s->Fy[i] - WY[i]*s->Fx[i];
    }

    // --- Body -------------------------------------------------------------
    double ax = FxSum/MASS, ay = FySum/MASS;
    s->vx += dt*(ax + s->vy*s->r);
    s->vy += dt*(ay - s->vx*s->r);
    s->r  += dt*Mz/IZ;
    double c = cos(s->psi), sn = sin(s->psi);
    s->x += dt*(s->vx*c - s->vy*sn);
    s->y += dt*(s->vx*sn + s->vy*c);
    s->psi += dt*s->r;
    s->axS += (ax - s->axS)*dt/0.02;
    s->ayS += (ay - s->ayS)*dt/0.02;
    s->t += dt;

    double speed = sqrt(s->vx*s->vx + s->vy*s->vy);
    if (s->phase == PHASE_BRAKING) {
        s->tBrake += dt;
        s->dist += speed*dt;
        if (fabs(s->psi) > s->maxYaw) s->maxYaw = fabs(s->psi);
        if (fabs(s->y) > s->maxLat) s->maxLat = fabs(s->y);
        for (int i = 0; i < 4; i++) UpdateSkid(s, i);

        s->telTimer -= dt;
        if (s->telTimer <= 0 && s->telCount < SIM_TEL_MAX) {
            TelSample *ts = &s->tel[s->telCount++];
            ts->t = (float)s->tBrake;
            ts->v = (float)speed;
            ts->pm = (float)s->pm;
            for (int i = 0; i < 4; i++) {
                ts->ws[i] = (float)(s->omega[i]*CAR_R);
                ts->p[i] = (float)s->p[i];
                ts->slip[i] = (float)s->slip[i];
            }
            s->telTimer += SIM_TEL_DT;
        }
        if (speed < V_STOP || s->tBrake > SIM_MAX_TIME) s->phase = PHASE_STOPPED;
    }
}

void SimRunToEnd(Sim *s)
{
    while (s->phase != PHASE_STOPPED) SimStep(s);
}
