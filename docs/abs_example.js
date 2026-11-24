// ABS controller, called every 5 ms (200 Hz) while the driver brakes.
//
//   s.t                 time since brake onset (s)
//   s.dt                controller period (s)
//   s.pedal             driver brake pedal, 0..1
//   s.masterPressure    master-cylinder pressure (bar): the most any wheel can get
//   s.wheelSpeed[i]     wheel speed omega*R (m/s)        i: 0 FL, 1 FR, 2 RL, 3 RR
//   s.wheelPressure[i]  measured wheel brake pressure (bar)
//   s.ax, s.ay          body acceleration (m/s^2), ax < 0 while braking
//   s.yawRate           yaw rate (rad/s), + = turning left
//   s.steer             road-wheel steer angle (rad), + = left
//   s.truth.vx          true vehicle speed (m/s). A real ECU doesn't know this!
//   mem                 your state, kept between calls, emptied at each run start
//
// Return [FL, FR, RL, RR] pressure commands in bar.
// Valves can only reduce pressure: commands are clamped to 0..masterPressure.

function abs(s, mem) {
  const dt = s.dt;
  const SLIP_DUMP = 0.12;    // dump above this slip
  const DECEL_DUMP = -40;    // ...or when the wheel decelerates harder than this (m/s^2)
  const ACCEL_HOLD = 10;     // hold while the wheel is spinning back up (m/s^2)
  const BUILD_RATE = 350;    // gentle re-apply rate (bar/s)
  const DUMP_RATE = 2500;    // release rate (bar/s)
  const GMA_RATE = 60;       // how fast the left/right front difference may grow (bar/s)
  const YAW_GAIN = 300;      // front pressure cut per unit yaw rate (bar per rad/s)

  if (!mem.init) {
    mem.init = true;
    mem.vRef = Math.max(...s.wheelSpeed);
    mem.prev = s.wheelSpeed.slice();
    mem.cmd = s.wheelPressure.slice();
  }

  // 1. Reference speed: the fastest wheel, but it may not fall faster than the
  //    accelerometer allows (plus a small margin), so it survives all wheels slipping.
  const vPred = mem.vRef + (Math.min(s.ax, 0) - 0.5) * dt;
  mem.vRef = Math.max(Math.max(...s.wheelSpeed), vPred);

  // 2. Per-wheel dump / hold / build
  for (let i = 0; i < 4; i++) {
    const w = s.wheelSpeed[i];
    const slip = (mem.vRef - w) / Math.max(mem.vRef, 1);   // 0 rolling .. 1 locked
    const wAcc = (w - mem.prev[i]) / dt;
    mem.prev[i] = w;

    if (slip > SLIP_DUMP || (wAcc < DECEL_DUMP && slip > 0.03)) {
      mem.cmd[i] = s.wheelPressure[i] - DUMP_RATE * dt;    // release
    } else if (wAcc > ACCEL_HOLD) {
      mem.cmd[i] = s.wheelPressure[i];                     // let it recover
    } else {
      mem.cmd[i] = Math.min(mem.cmd[i], s.wheelPressure[i] + 5) + BUILD_RATE * dt;
    }
    mem.cmd[i] = Math.max(0, Math.min(mem.cmd[i], s.masterPressure));
  }

  // 3. Rear axle "select-low": both rears get the lower pressure, which keeps
  //    the rear axle's side grip on split-mu and stops the car from spinning.
  const rear = Math.min(mem.cmd[2], mem.cmd[3]);

  // 4. Yaw-moment build-up delay: on split-mu the high-grip front wheel may only
  //    exceed the low-grip one by a margin that grows slowly, so the yaw moment
  //    rises gently instead of snapping the car around.
  const frontGap = GMA_RATE * s.t;
  let fl = mem.cmd[0], fr = mem.cmd[1];
  fl = Math.min(fl, fr + frontGap);
  fr = Math.min(fr, fl + frontGap);

  // 5. Yaw-rate feedback: if the car starts rotating anyway, ease off the front
  //    wheel on the side it is turning towards (more braking left = turns left).
  const yawFix = YAW_GAIN * s.yawRate;
  if (yawFix > 0) fl = Math.max(0, fl - yawFix);
  else fr = Math.max(0, fr + yawFix);

  return [fl, fr, rear, rear];
}
