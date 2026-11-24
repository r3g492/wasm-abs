// Compiles the user's ABS source and adapts it to the simulator's float buffer.
// Used by the page and by the headless test (Node).
var AbsRuntime = (function () {
  // Must match the ABS_IN_* layout in src/sim.h
  var I = { T: 0, DT: 1, PEDAL: 2, PMASTER: 3, WSPEED: 4, WPRESS: 8, AX: 12, AY: 13, YAW: 14, STEER: 15, VX: 16 };

  function create(onError) {
    var fn = null, mem = {}, failed = false;

    function compile(src) {
      var f = new Function('"use strict";\n' + src + '\n;return typeof abs === "function" ? abs : null;')();
      if (!f) throw new Error('Your code must define a function named abs(s, mem).');
      fn = f; mem = {}; failed = false;
    }

    function fail(msg) {
      failed = true;
      if (onError) onError(msg);
      return null;
    }

    function step(b) {
      if (!fn || failed) return null;
      var s = {
        t: b[I.T], dt: b[I.DT], pedal: b[I.PEDAL], masterPressure: b[I.PMASTER],
        wheelSpeed: [b[I.WSPEED], b[I.WSPEED + 1], b[I.WSPEED + 2], b[I.WSPEED + 3]],
        wheelPressure: [b[I.WPRESS], b[I.WPRESS + 1], b[I.WPRESS + 2], b[I.WPRESS + 3]],
        ax: b[I.AX], ay: b[I.AY], yawRate: b[I.YAW], steer: b[I.STEER],
        truth: { vx: b[I.VX] }
      };
      var out;
      try { out = fn(s, mem); }
      catch (e) { return fail('abs() threw at t=' + s.t.toFixed(3) + ' s: ' + (e && e.message ? e.message : e)); }
      if (!out || out.length !== 4) return fail('abs() must return an array of 4 pressures [FL, FR, RL, RR].');
      for (var i = 0; i < 4; i++) {
        if (typeof out[i] !== 'number' || !isFinite(out[i]))
          return fail('abs() returned a non-finite pressure for wheel ' + i + ' at t=' + s.t.toFixed(3) + ' s.');
      }
      return out;
    }

    function reset() { mem = {}; failed = false; }

    return { compile: compile, step: step, reset: reset };
  }

  return { create: create };
})();

if (typeof module !== 'undefined') module.exports = AbsRuntime;
