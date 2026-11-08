/* Portable adapter regression check: synthetic evidence only; no server or flight. */
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
class Element {
  constructor(tag = 'div') { this.tag = tag; this.children = []; this.hidden = false; this.value = ''; this.textContent = ''; this.style = {}; this.dataset = {}; this.className = ''; this.classList = {toggle: (name, enabled) => { const names = new Set(this.className.split(' ').filter(Boolean)); enabled ? names.add(name) : names.delete(name); this.className = [...names].join(' '); }}; }
  append(...nodes) { for (const n of nodes) { n.parentElement = this; this.children.push(n); } }
  prepend(n) { n.parentElement = this; this.children.unshift(n); }
  replaceChildren(...nodes) { this.children = []; this.append(...nodes); }
  insertBefore(n, before) { n.parentElement = this; this.children.splice(this.children.indexOf(before), 0, n); }
  setAttribute(name, value) { this[name] = value; }
}
const section = new Element('section'), right = new Element('aside'), sidebar = new Element('aside');
const visiblePanel = new Element(), hiddenPanel = new Element(); hiddenPanel.hidden = true;
section.append(visiblePanel, hiddenPanel);
const ids = Object.fromEntries(['session-filter', 'runs', 'run-counts', 'count', 'compare', 'error', 'wall-bench'].map(id => [id, new Element()]));
ids['wall-bench'].hidden = true; sidebar.append(ids['session-filter'], ids.runs);
const roots = [section, right, sidebar, ...Object.values(ids)];
const all = () => { const found = new Set(); const visit = n => { if (found.has(n)) return; found.add(n); n.children.forEach(visit); }; roots.forEach(visit); return [...found]; };
const document = {
  createElement: tag => new Element(tag),
  getElementById: id => ids[id] || all().find(n => n.id === id),
  querySelector: selector => selector === 'main > section' ? section : right,
  querySelectorAll: selector => selector === 'video' ? [] : all().filter(n => n.className.split(' ').includes('run')),
};
const base = [1, 2, 3].map(i => ({id: 'flight:' + i, observed_flight: true, course_pass: true, has_track: true, kind: 'flight'}));
base.push({id: 'native:1', trajectory_archive: true}, {id: 'setup:1'}, {id: 'bench:m3-wall-rotation', experiment: true});
const record = (id, kind, extra = {}) => ({id, kind, title: id, group: 'Synthetic', source: 'fixture', outcome: 'passed', summary: 'Fixture receipt', date_basis: 'fixture', files: [{name: 'receipt.json', url: '/fixture', viewable: true, size: 10}], ...extra});
const records = [record('bench', 'passive bench'), record('replay', 'offline replay / analysis'), record('unit', 'planner / geometry'), record('preflight', 'setup / preflight check'), record('unknown', 'unknown'), record('run:flight:1', 'flight'), record('wall', 'passive bench', {source: 'wall-rotation/20261107T224200Z-experimental-m3-tangent-pair'})];
const delegated = [];
const context = vm.createContext({document, URL, URLSearchParams, Date, Map, Set, Option: class extends Element { constructor(label, value) { super('option'); this.textContent = label; this.value = value; } }, runs: base, selected: null, track: [], generation: 0, playing: true, renderArchive: null, location: {href: 'http://fixture/?run=test%3Abench', search: '?run=test%3Abench'}, history: {replaceState(_a, _b, url) { context.location.href = String(url); }}, json: async () => ({records}), select: async id => { delegated.push(id); context.selected = context.runs.find(r => r.id === id); const wall = id === 'bench:m3-wall-rotation'; ids['wall-bench'].hidden = !wall; section.hidden = wall; right.hidden = wall; }});
context.window = context;
vm.runInContext(fs.readFileSync(path.join(__dirname, 'all_tests_picker.js'), 'utf8'), context);
(async () => {
  await context.allTestsPicker.renderPicker();
  assert.equal(context.selected.id, 'test:bench', 'generic initial deep link');
  assert.equal(delegated.length, 0, 'generic evidence must not delegate to flight viewer');
  const detail = document.getElementById('all-test-detail');
  assert.equal(detail.hidden, false); assert.equal(right.hidden, true); assert.equal(visiblePanel.hidden, true);
  assert.equal(ids['run-counts'].textContent, '5 session archives · 3 observed flights · 3 course passes · 5 other saved tests');
  const expected = {flights: ['flight:1','flight:2','flight:3'], trajectories: ['native:1'], sessions: ['setup:1','test:preflight'], bench: ['bench:m3-wall-rotation','test:bench'], replay: ['test:replay'], unit: ['test:unit'], unknown: ['test:unknown']};
  for (const [kind, values] of Object.entries(expected)) { ids['session-filter'].value = kind; await context.allTestsPicker.renderPicker(false); assert.deepEqual(ids.runs.children.filter(n => n.dataset.id).map(n => n.dataset.id), values, kind); }
  ids['session-filter'].value = 'all'; document.getElementById('all-tests-search').value = 'receipt.json'; await context.allTestsPicker.renderPicker(false); assert.equal(ids.count.textContent, '(5)', 'search evidence filenames');
  document.getElementById('all-tests-search').value = ''; await context.allTestsPicker.renderPicker(false);
  assert.equal(context.runs.length, 11, 'refresh must not duplicate generic records or canonical wall row');
  assert.equal(ids.compare.children.length, 4, 'only native track-bearing base records enter comparison');
  for (const id of ['bench:m3-wall-rotation', 'test:replay', 'flight:1', 'test:bench', 'bench:m3-wall-rotation', 'flight:2']) { await context.select(id); if (id.startsWith('test:')) { assert.equal(detail.hidden, false); assert.equal(ids['wall-bench'].hidden, true); assert.equal(right.hidden, true); } else { assert.equal(detail.hidden, true); assert.equal(visiblePanel.hidden, false, 'restore visible panel'); assert.equal(hiddenPanel.hidden, true, 'preserve already hidden panel'); assert.equal(section.hidden, id.startsWith('bench:')); assert.equal(right.hidden, id.startsWith('bench:')); } }
  assert.deepEqual(delegated, ['bench:m3-wall-rotation','flight:1','bench:m3-wall-rotation','flight:2']);
  assert.equal(context.runs.filter(r => r.test_record && (r.course_pass || r.observed_flight)).length, 0, 'passed bench/unit receipts cannot become flight passes');
  const queue=context.allTestsPicker.queueSummary({assigned:['Q1'],next_available:'Q2',next_to_claim:null,trials:[{id:'Q1',name:'Synthetic assigned intent'},{id:'Q2',name:'Synthetic next intent'}]});
  assert.equal(queue.assigned[0],'Q1 · Synthetic assigned intent');assert.equal(queue.next,'Q2 · Synthetic next intent');assert.equal(queue.claim,'Not recorded');
  console.log('all_tests_picker synthetic adapter checks passed');
})().catch(error => { console.error(error); process.exitCode = 1; });
