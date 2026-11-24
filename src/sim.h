// Vehicle braking simulation core: no rendering, no platform code.
#ifndef SIM_H
#define SIM_H

#define SIM_DT          0.0005      // physics step (s), 2 kHz
#define SIM_CTRL_DT     0.005       // ABS controller period (s), 200 Hz
#define SIM_TEL_DT      0.01        // telemetry sample period (s)
#define SIM_TEL_MAX     4500        // 45 s of telemetry
#define SIM_MAX_TIME    45.0        // braking timeout (s)
#define SIM_SKID_MAX    3000        // skid mark points per wheel

// ABS controller I/O (float buffers shared with the JS bridge)
enum {
    ABS_IN_T = 0,           // time since brake onset (s)
    ABS_IN_DT,              // controller period (s)
    ABS_IN_PEDAL,           // brake pedal 0..1
    ABS_IN_PMASTER,         // master cylinder pressure (bar)
    ABS_IN_WSPEED,          // 4 wheel speeds omega*R (m/s): FL FR RL RR
    ABS_IN_WPRESS = ABS_IN_WSPEED + 4,  // 4 measured wheel pressures (bar)
    ABS_IN_AX = ABS_IN_WPRESS + 4,      // body longitudinal accel (m/s^2)
    ABS_IN_AY,              // body lateral accel (m/s^2, + left)
    ABS_IN_YAWRATE,         // yaw rate (rad/s, + left)
    ABS_IN_STEER,           // road-wheel steer angle (rad, + left)
    ABS_IN_VX_TRUE,         // ground-truth longitudinal speed (m/s)
    ABS_IN_COUNT
};

// Returns 1 and fills out[4] with pressure commands (bar), or 0 on failure
typedef int (*AbsFn)(const float *in, float *out, void *user);

typedef enum { SURF_DRY = 0, SURF_WET, SURF_WATER, SURF_ICE, SURF_COUNT } Surface;

typedef struct {
    const char *name;
    double c1, c2, c3;      // Burckhardt friction curve: mu(s) = c1(1 - e^(-c2 s)) - c3 s
} SurfaceParams;

typedef struct {
    const char *name;
    const char *desc;
    double v0Kmh;
    Surface left, right;    // surfaces on left (y > 0) / right (y < 0) half after the brake line
} Scenario;

#define SCENARIO_COUNT 5
extern const Scenario SCENARIOS[SCENARIO_COUNT];
extern const SurfaceParams SURFACES[SURF_COUNT];

typedef enum { PHASE_RUNUP = 0, PHASE_BRAKING, PHASE_STOPPED } Phase;
typedef enum { DRIVER_STRAIGHT = 0, DRIVER_CORRECT } DriverMode;

typedef struct {
    float t, v;
    float ws[4], p[4], slip[4];
    float pm;
} TelSample;

typedef struct { float x, y; unsigned char surf, start; } SkidPoint;

typedef struct {
    // configuration
    int scenario;
    int absOn;
    DriverMode driver;
    double v0;                  // initial speed (m/s)
    AbsFn absFn;
    void *absUser;

    // vehicle state
    double x, y, psi;           // world position (m), heading (rad)
    double vx, vy, r;           // body-frame velocity (m/s), yaw rate (rad/s)
    double axS, ayS;            // sensed (filtered) body accelerations
    double omega[4], spin[4];   // wheel speed (rad/s), wheel rotation angle (rad)
    double p[4], pcmd[4];       // wheel pressure / command (bar)
    double steer, pedal, pm;    // road-wheel angle (rad), pedal 0..1, master pressure (bar)

    // per-wheel diagnostics
    double Fz[4], Fx[4], Fy[4], slip[4], mu[4];
    Surface surf[4];

    // run state
    Phase phase;
    double t, tBrake;           // total time, time since brake onset
    double ctrlTimer, telTimer;
    int absFault;               // controller threw / returned garbage this run
    int absCalls;

    // results
    double dist;                // distance travelled since brake onset (m)
    double maxYaw;              // max |psi| (rad)
    double maxLat;              // max |y| (m)

    TelSample tel[SIM_TEL_MAX];
    int telCount;

    SkidPoint skid[4][SIM_SKID_MAX];
    int skidCount[4];
    int skidActive[4];
    float skidLastX[4], skidLastY[4];
} Sim;

void SimReset(Sim *s, int scenario, int absOn, DriverMode driver, double v0Kmh, AbsFn fn, void *user);
void SimStep(Sim *s);                       // advance one SIM_DT step
void SimRunToEnd(Sim *s);                   // run until stopped or timeout
Surface SimSurfaceAt(const Sim *s, double x, double y);
void SimWheelPos(const Sim *s, int i, double *wx, double *wy);  // world contact point

// Vehicle constants exposed for rendering
#define CAR_A       1.15        // CG to front axle (m)
#define CAR_B       1.45        // CG to rear axle (m)
#define CAR_TRACK   1.55        // track width (m)
#define CAR_R       0.31        // wheel radius (m)

#endif
