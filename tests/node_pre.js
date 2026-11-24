// --pre-js for the headless test: load the ABS runtime + example controller in Node.
(function () {
  var fs = require('fs'), path = require('path');
  var root = process.env.ABS_ROOT || process.cwd();
  var AbsRuntime = require(path.join(root, 'web/abs_runtime.js'));
  var src = fs.readFileSync(process.env.ABS_FILE || path.join(root, 'web/abs_example.js'), 'utf8');
  var rt = AbsRuntime.create(function (msg) { console.error('ABS error: ' + msg); });
  rt.compile(src);
  Module.absStep = rt.step;
  Module.absReset = rt.reset;
})();
