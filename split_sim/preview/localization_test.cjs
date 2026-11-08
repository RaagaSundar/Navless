// Portable offline tests; synthetic source metadata only, never flight evidence.
// Run: node split_sim/preview/localization_test.cjs
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, 'app.js'), 'utf8');
const start = source.indexOf('const localizationColors=');
const pollStart = source.indexOf('async function pollLocalization()', start);
assert(start >= 0 && pollStart > start, 'localization module is present');
const end = source.indexOf('\n', pollStart);
const context = {console};
vm.createContext(context);
vm.runInContext(source.slice(start, end), context);

const fixture = {
  schema_version: 1, run_id: 'synthetic-not-a-flight',
  sources: ['raw_nvidia', 'admitted_vo', 'px4_fused', 'truth'].map(id => ({
    id, label: `Synthetic ${id}`, frame: 'course_enu', units: 'm, rad',
    origin: 'Synthetic test origin; no actual flight', status: 'available',
  })),
  registration: {status: 'fixture_declared', truth_used_for_alignment: false},
  samples: [
    {t: 10, sources: {
      raw_nvidia: {stamp_us: 10000000, xyz: [1, 2, 3], yaw: .2, status: 'observed', age_s: 0},
      admitted_vo: {stamp_us: 9900000, xyz: [1.1, 2.1, 3.1], status: 'observed', age_s: .1},
      px4_fused: {stamp_us: 9950000, xyz: [1.2, 2.2, 3.2], status: 'observed', confidence: {xy_valid: 1, z_valid: 1}},
      truth: {stamp_us: 10000000, xyz: [1.3, 2.3, 3.3], status: 'reference_only'},
    }},
    {t: 11, sources: {raw_nvidia: {stamp_us: 11000000, xyz: [2, 3, 4], status: 'observed'}}},
  ],
};
const frozen = JSON.stringify(fixture);
context.fixture = fixture;
const series = vm.runInContext('normalizeLocalization(fixture)', context);
assert.equal(series.size, 4);
assert.equal(JSON.stringify(fixture), frozen, 'normalization cannot change recorded values');
for (const id of ['raw_nvidia', 'admitted_vo', 'px4_fused', 'truth']) {
  const points = series.get(id);
  assert(points.length);
  assert(points[0].drawable);
  assert.equal(points[0].origin, 'Synthetic test origin; no actual flight');
}
assert.equal(series.get('admitted_vo')[0].t, 9.9, 'use sensor source stamp, not row receipt time');
assert.equal(JSON.stringify(series.get('raw_nvidia')[0].xyz), '[1,2,3]');
context.points = series.get('raw_nvidia');
assert.equal(vm.runInContext('localizationAt(points, 9)', context), null);
assert.equal(vm.runInContext('localizationAt(points, 10.5).t', context), 10, 'never use a future sample');

context.fixture = {
  sources: [{source_id: 'cad:test', frame_id: 'course_enu', units: 'm', registration_status: 'documented_course_frame'}],
  samples: [
    {stamp_s: 1, source_id: 'cad:test', pose_enu_m: [1, 2, 3], status: 'accepted'},
    {stamp_s: 2, source_id: 'cad:test', pose_enu_m: [1, 2, 3], status: 'ambiguous'},
    {stamp_s: 3, source_id: 'cad:test', pose_enu_m: null, status: 'fit_rejected'},
    {stamp_s: 4, source_id: 'scan:test', frame_id: 'scan_native', pose_enu_m: null, status: 'unregistered'},
  ],
};
const maps = vm.runInContext('normalizeLocalization(fixture)', context);
assert.equal(maps.get('cad:test')[0].drawable, true);
assert.equal(maps.get('cad:test')[1].drawable, false);
assert.equal(maps.get('cad:test')[2].xyz, null);
assert.equal(maps.get('scan:test')[0].drawable, false);
context.fixture = {samples: [{t: 1, sources: {
  raw_nvidia: {xyz: [1, 2, 3], frame: 'odom', status: 'observed'},
  px4_fused: {xyz: [1, 2, 3], frame: 'course', status: 'observed', confidence: {xy_valid: 0, z_valid: 1}},
}}]};
const invalid = vm.runInContext('normalizeLocalization(fixture)', context);
assert.equal(invalid.get('raw_nvidia')[0].drawable, false, 'unregistered odom cannot be placed on course');
assert.equal(invalid.get('px4_fused')[0].drawable, false, 'invalid PX4 flags remain invalid');

// Polling must leave a watched video and the replay cursor untouched.
let loads = 0;
const video = {paused: false, currentTime: 12.5};
const elements = {
  'videos': {querySelectorAll: () => [video]},
  'localization-live': {checked: true}, 'scrub': {value: 7, max: 10},
};
Object.assign(context, {
  selected: {id: 'nvidia:synthetic', in_progress: true}, generation: 1,
  playing: false, track: [{t: 10}],
  $: id => elements[id],
  json: async () => [{id: 'nvidia:synthetic', has_track: true}],
  draw: () => { throw Error('watched video must not trigger map/video sync redraw'); },
  loadLocalization: async () => { loads++; },
});
(async () => {
  await vm.runInContext('pollLocalization()', context);
  assert.equal(loads, 1);
  assert.equal(video.paused, false);
  assert.equal(video.currentTime, 12.5);
  assert.equal(elements.scrub.value, 7);
  assert(!source.includes('&&selected.in_progress&&!playing&&'), 'legacy reset timer removed');
  assert(source.includes('loadVideos(r,gen)'), 'existing video load integration retained');
  assert(source.includes("$('localization-live').checked=false;playing=false"), 'scrubbing exits live latest');
  console.log('PASS: four sources, exact values/origin/time, no future samples, map rejection, invalid frames/flags, video/cursor preservation');
})().catch(error => { console.error(error); process.exitCode = 1; });
