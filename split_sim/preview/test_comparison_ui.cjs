// Offline DOM fixture; no browser, ROS, simulator or flight access.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const source=fs.readFileSync(__dirname+'/app.js','utf8');
function element(tag='div'){return {tag,children:[],style:{},append(child){this.children.push(child)},replaceChildren(){this.children=[]}}}
const box=element(),selector=element();let overlays=0;
const baseline={id:'nvidia:baseline',name:'unused',has_track:true,files:[],comparison:{schema:1,run_id:'nvidia:baseline',name:'Baseline'}};
const context={document:{createElement:element},runs:[baseline],$:id=>id==='run-changes'?box:selector,
 text:(parent,tag,value)=>{const child=element(tag);child.value=value;parent.append(child);return child},
 list:(parent,values)=>values.forEach(v=>parent.append({value:v})),evidenceLinks:()=>{},loadComparison:()=>overlays++};
vm.createContext(context);vm.runInContext(source.slice(source.indexOf('function runComparison'),source.indexOf('function validPlan')),context);
assert.equal(context.runName({id:'a',comparison:{schema:1,run_id:'other',name:'Wrong'}}),'Unnamed archived session');
context.renderComparison({id:'a'});assert(box.children.some(x=>String(x.value).includes('unknown')));
const run={id:'a',files:[],comparison:{schema:1,run_id:'a',name:'Centered window',kind:'flight attempted',baseline_id:'nvidia:baseline',changes:[{field:'height',before:.85,after:.762,evidence:[]}],notes:[],confounds:[]}};
context.renderComparison(run);const link=box.children.find(x=>x.tag==='a');let prevented=false;link.onclick({preventDefault(){prevented=true}});assert(prevented);assert.equal(overlays,1);assert.equal(selector.value,'nvidia:baseline');assert.equal(run.id,'a');
console.log('PASS comparison identity, unknown state, baseline overlay without changing selected run');
