/* Shared saved-evidence catalog. No execution, flight grading or flight commands. */
(()=>{
 const originalSelect=select;
 let inventoryData=null,inventoryAt=0,loading=null,limit=70;
 const detail=document.createElement('div');detail.id='all-test-detail';detail.className='panel';detail.hidden=true;
 const section=document.querySelector('main > section');section.prepend(detail);
 const right=document.querySelector('main > aside:last-child');
 const hiddenBefore=new Map();let originalColumn=null;
 const picker=document.getElementById('session-filter');
 picker.replaceChildren(...[['all','All tests & runs'],['flights','Observed flights'],['trajectories','Native trajectories · ungraded'],['sessions','Setup / preflight'],['bench','Passive / hardware benches'],['replay','Offline replays / analysis'],['unit','Unit / planner checks'],['unknown','Other saved evidence']].map(([value,label])=>new Option(label,value)));
 picker.value='all';
 const search=document.createElement('input');search.type='search';search.id='all-tests-search';search.placeholder='Search all tests, runs, receipts…';search.setAttribute('aria-label','Search all tests and runs');search.style.width='100%';search.style.margin='12px 0';search.style.padding='10px';search.style.font='inherit';search.style.background='#152432';search.style.color='#d8e4ea';search.style.border='1px solid #345064';search.style.borderRadius='6px';picker.parentElement.insertBefore(search,document.getElementById('runs'));
 const scope=document.createElement('small');scope.textContent='Saved evidence only. Unknown outcomes stay unknown; replay and bench checks do not count as flight passes.';picker.parentElement.insertBefore(scope,search);
 const add=(parent,tag,value,cls)=>{const n=document.createElement(tag);n.textContent=value??'';if(cls)n.className=cls;parent.append(n);return n};
 const category=r=>r.test_record?/bench/.test(r.test_record.kind)?'bench':/replay|analysis/.test(r.test_record.kind)?'replay':/unit|planner|geometry/.test(r.test_record.kind)?'unit':/setup|preflight/.test(r.test_record.kind)?'sessions':'unknown':r.observed_flight?'flights':r.id.startsWith('bench:')?'bench':r.trajectory_archive?'trajectories':'sessions';
 const label=r=>r.test_record?r.test_record.title:r.display_name||(typeof runName==='function'?runName(r):null)||r.run_id||r.id;
 const corpus=r=>[label(r),r.id,r.platform_label,r.status,r.test_record?.source,r.test_record?.summary,...(r.test_record?.files||[]).map(f=>f.name)].join(' ').toLowerCase();
 async function fetchInventory(){if(inventoryData&&Date.now()-inventoryAt<15000)return inventoryData;if(!loading)loading=json('/api/tests').then(d=>{inventoryData=d;inventoryAt=Date.now();return d}).finally(()=>loading=null);return loading}
 function attach(data){const base=runs.filter(r=>!r.test_record);const known=new Set(base.map(r=>r.id));for(const record of data.records||[]){if(record.id.startsWith('run:'))continue;
   // The paired wall experiment already has a specialized row, maps and cameras.
   if(record.source==='wall-rotation/20261107T224200Z-experimental-m3-tangent-pair'&&known.has('bench:m3-wall-rotation'))continue;
   base.push({id:'test:'+record.id,run_id:record.title,display_name:record.title,platform_label:record.group,status:record.outcome,kind:record.kind,test_record:record,observed_flight:false,course_pass:false,in_progress:false,has_track:false,files:[]});
 }runs=base}
 function restore(){for(const[node,hidden]of hiddenBefore)node.hidden=hidden;hiddenBefore.clear();if(originalColumn!==null){section.style.gridColumn=originalColumn;originalColumn=null}detail.hidden=true}
 function show(record,row){const wall=document.getElementById('wall-bench');if(wall)wall.hidden=true;section.hidden=false;if(originalColumn===null)originalColumn=section.style.gridColumn||'';section.style.gridColumn='2 / -1';for(const node of [...section.children,right])if(node!==detail){if(!hiddenBefore.has(node))hiddenBefore.set(node,node.hidden);node.hidden=true}detail.hidden=false;detail.replaceChildren();add(detail,'div',record.group+' / '+record.kind,'tag');add(detail,'h1',record.title);add(detail,'p','Outcome: '+record.outcome);add(detail,'p',record.summary);add(detail,'p',(record.date||'Date unknown')+' · '+record.date_basis,'muted');if(record.outcome_basis)add(detail,'p','Outcome source: '+record.outcome_basis,'muted');
  const link=document.createElement('a');link.href='/?run='+encodeURIComponent(row.id);link.textContent='Link to this saved test';detail.append(link);
  if(record.parent_viewer_url){const parent=add(detail,'a','Open parent flight');parent.href=record.parent_viewer_url;parent.style.marginLeft='18px'}
  for(const parentId of record.parent_run_ids||[]){const match=runs.find(r=>!r.test_record&&(r.id===parentId||r.id.endsWith(parentId)||r.run_id===parentId));if(match){const a=add(detail,'a','Related run: '+label(match));a.href='/?run='+encodeURIComponent(match.id);a.style.display='block'}}
  add(detail,'h2','Scope & limits');for(const value of record.limits||[])add(detail,'p',value,'muted');add(detail,'h2','Exact saved evidence');
  const filter=document.createElement('input');filter.type='search';filter.placeholder='Filter evidence files';filter.setAttribute('aria-label','Filter evidence files for this test');filter.style.width='100%';filter.style.padding='10px';filter.style.font='inherit';filter.style.background='#152432';filter.style.color='#d8e4ea';filter.style.border='1px solid #345064';filter.style.borderRadius='6px';detail.append(filter);const files=add(detail,'div','');const renderFiles=()=>{files.replaceChildren();for(const f of record.files.filter(f=>f.name.toLowerCase().includes(filter.value.toLowerCase()))){const line=add(files,'div','');if(f.viewable){const a=add(line,'a',f.name);a.href=f.url;a.target='_blank';a.rel='noopener'}else add(line,'span',f.name+' · over viewing limit');if(f.size!==null)add(line,'small',' · '+(f.size/1024).toFixed(1)+' KiB','muted')}};filter.oninput=renderFiles;renderFiles();
  const footer=add(detail,'p','');const index=add(footer,'a','Browse the standalone searchable index');index.href='/tests';
 }
 select=async id=>{const row=runs.find(r=>r.id===id);if(!row?.test_record){restore();return originalSelect(id)}++generation;playing=false;for(const video of document.querySelectorAll('video'))video.pause();selected=row;track=[];const u=new URL(location.href);u.searchParams.set('run',id);history.replaceState(null,'',u);show(row.test_record,row);document.querySelectorAll('.run').forEach(b=>b.classList.toggle('active',b.dataset.id===id))};
 async function renderPicker(choose=true){try{attach(await fetchInventory())}catch(e){document.getElementById('error').textContent='Saved-test inventory unavailable: '+e.message}
  const base=runs.filter(r=>!r.test_record);if(typeof enrichComparisons==='function')await enrichComparisons(base);const tests=runs.length-base.length;document.getElementById('run-counts').textContent=`${base.filter(r=>!r.experiment).length} session archives · ${base.filter(r=>r.observed_flight).length} observed flights · ${base.filter(r=>r.course_pass).length} course passes · ${tests} other saved tests`;
  const query=search.value.toLowerCase().trim(),kind=picker.value;const visible=runs.filter(r=>(kind==='all'||category(r)===kind)&&(!query||corpus(r).includes(query)));const box=document.getElementById('runs');box.replaceChildren();document.getElementById('count').textContent=`(${visible.length})`;
  for(const r of visible.slice(0,limit)){const b=add(box,'button','', 'run');b.dataset.id=r.id;b.classList.toggle('active',selected?.id===r.id);add(b,'div',label(r));add(b,'small',r.platform_label+' · '+r.kind);add(b,'div',r.status,'status '+(r.course_pass?'passed':r.observed_flight?'failed':''));b.onclick=()=>select(r.id)}
  if(visible.length>limit){const more=add(box,'button',`Show next ${Math.min(70,visible.length-limit)} of ${visible.length}`);more.onclick=()=>{limit+=70;renderPicker(false)}}if(!visible.length)add(box,'p','No saved evidence matches this search.','muted');
  const compare=document.getElementById('compare'),old=compare.value;compare.replaceChildren(new Option('No comparison',''));for(const r of base)if(r.has_track)compare.append(new Option(label(r),r.id));compare.value=old;
  if(choose){const requested=new URLSearchParams(location.search).get('run');const id=runs.some(r=>r.id===(selected?.id||requested))?(selected?.id||requested):(base.find(r=>r.observed_flight)||base[0])?.id;if(id)await select(id)}
 }
 renderArchive=()=>renderPicker(true);search.oninput=()=>{limit=70;renderPicker(false)};picker.onchange=()=>{limit=70;renderPicker(false)};
 window.allTestsPicker={fetchInventory,renderPicker};
})();
