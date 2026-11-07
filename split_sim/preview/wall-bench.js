/* Paired views of one immutable passive experiment. Coordinates are ENU metres. */
async function selectWallBench(run, current) {
  const main = document.querySelector('main');
  const flight = main.querySelector('section:not(#wall-bench)');
  const details = main.querySelectorAll(':scope > aside')[1];
  let panel = document.getElementById('wall-bench');
  const isWall = run.id === 'bench:m3-wall-rotation';
  if (panel) panel.hidden = !isWall;
  if (flight) flight.hidden = isWall;
  if (details) details.hidden = isWall;
  if (!isWall) return;
  if (!panel) {
    panel = document.createElement('section');
    panel.id = 'wall-bench';
    panel.className = 'panel';
    panel.style.gridColumn = '2 / -1';
    flight.before(panel);
  }
  document.querySelectorAll('video').forEach(v => v.pause());
  document.querySelectorAll('.run').forEach(b => b.classList.toggle('active', b.dataset.id === run.id));
  panel.innerHTML = '<div class="tag">MAC GAZEBO · PAIRED STEREO TEST</div><h1>Unexpected wall · M3 rotated 18°</h1><p>The simulation moved the last maze wall. The drone started with the original map.</p><h2>Course overview</h2><div class="controls"><label>Position recording <select id="wall-source"><option value="baseline">Baseline NVIDIA flight · original wall · 221438Z</option><option value="rig">Rotated-wall camera bench · rig reference only</option></select></label><label><input type="checkbox" id="wall-zoom"> Zoom to M3</label></div><p id="wall-source-note" class="muted"></p><div id="wall-moving-flights"></div><div id="wall-maps"></div><div class="controls"><button type="button" id="wall-play">Play positions</button><label for="wall-time">Simulation time</label><output id="wall-sec"></output></div><input type="range" id="wall-time" min="0" max="0" value="0"><div id="wall-coordinates"></div><p><span style="color:#ffa458">━ Actual M3</span> &nbsp; <span style="color:#c9a0ff">┅ Preloaded M3</span> &nbsp; <span style="color:#65a7ec">┄ Full A* plan</span> &nbsp; <span style="color:#62dbbc">━ Tested approach</span> &nbsp; North ↑</p><h2>What changed</h2><p>M3 rotated 18° clockwise about its north end. The shared map shows the outbound A* plan. The green segment and its 0.30 m envelope mark the portion used in the camera test. The translucent band overlaps the wall in the actual course.</p><h2>What the cameras saw</h2><video style="width:100%" controls playsinline preload="metadata" poster="/wall-test/cameras.png" src="/wall-test/cameras.mp4"></video><p>The original view is on the left and the rotated wall on the right. The detector first reported a persistent obstruction at 5.7 s. The planned envelope would meet the changed wall at 8.0 s. The clean camera sequence produced no persistent warnings.</p><p class="muted">Passive camera test with a commanded camera pose. Braking and avoidance in flight were not tested. The preloaded map shows what the navigation system assumed before sensing the wall.</p><a href="/wall-test/results.json">Detector results</a> · <a href="/wall-test/fixture.json">Wall geometry</a>';
  try {
    const [course, fixture, route, fullRoute] = await Promise.all(['/api/course','/wall-test/fixture.json','/wall-test/route.json','/evidence/nvidia%3A20261107T221438Z-split-astar-flight/flight/planned-route.json'].map(async path => {
      const response = await fetch(path); if (!response.ok) throw Error(response.status); return response.json();
    }));
    if (!current()) return;
    const ns = 'http://www.w3.org/2000/svg';
    const inches = x => x * .0254 - 3.048;
    function svgEl(parent, name, attrs) {
      const el = document.createElementNS(ns,name);
      Object.entries(attrs).forEach(([key,value]) => el.setAttribute(key,value));
      parent.append(el); return el;
    }
    const colors={truth:'#59d8ba',px4_fused:'#ffd267',raw_nvidia:'#dc9aff',admitted_vo:'#72b6ff',rig:'#59d8ba'};
    const labels={truth:'Gazebo truth',px4_fused:'PX4 fused',raw_nvidia:'NVIDIA SLAM',admitted_vo:'Admitted odometry',rig:'Camera rig reference'};
    const baselineID='nvidia%3A20261107T221438Z-split-astar-flight';
    const [poseResponse,rigResponse]=await Promise.all([fetch('/evidence/'+baselineID+'/localization-poses.json'),fetch('/wall-test/reference.jsonl')]);
    if(!poseResponse.ok||!rigResponse.ok)throw Error('Position recordings unavailable');
    const [poses,rigText]=await Promise.all([poseResponse.json(),rigResponse.text()]);
    if(!current())return;
    const rigRows=rigText.trim().split('\n').map(line=>{const r=JSON.parse(line);return {t:r.trajectory_t,sources:{rig:{xyz:r.position_enu,frame:'course_enu',status:'reference_only',age_s:0}}};});
    const slider=panel.querySelector('#wall-time'), source=panel.querySelector('#wall-source'), play=panel.querySelector('#wall-play');
    let rows=[],playing=false;
    function resetSource(){
      playing=false;play.textContent='Play positions';
      const baseline=source.value==='baseline';rows=baseline?poses.samples:rigRows;
      panel.querySelector('#wall-source-note').textContent=baseline?'Position traces come from the recorded 221438Z flight with M3 in its original position. The orange rotated wall is shown for comparison.':'The rotated-wall bench recorded camera rig references and stereo images. PX4 and NVIDIA SLAM were not running in this recording.';
      slider.max=Math.max(0,rows.length-1);
      slider.value=baseline?Math.max(0,rows.findIndex(r=>r.t>=120)):Math.max(0,rows.findIndex(r=>r.t>=5.7));
      render();
    }
    function valid(point){return point?.frame==='course_enu' && ['observed','reference_only'].includes(point.status) && point.age_s<=.5 && point.xyz?.every(Number.isFinite);}
    function render() {
      const zoom = panel.querySelector('#wall-zoom').checked;
      const maps=panel.querySelector('#wall-maps');maps.replaceChildren();
      const card=maps, label='Actual and preloaded course maps';
      {
        const svg = svgEl(card,'svg',{viewBox:zoom?'-1.43 -2.65 2.62 2.62':'-3.5 -3.5 7 7',role:'img','aria-label':label,style:'border:1px solid #304454;display:block;width:100%;max-height:63vh'});
        const g = svgEl(svg,'g',{transform:'scale(1,-1)'});
        drawCourseBase(g,course,'M3');
        const path = (points,attrs) => svgEl(g,'polyline',{points:points.map(p=>p.slice(0,2).join(',')).join(' '),fill:'none','stroke-linecap':'round','stroke-linejoin':'round',...attrs});
        path(fullRoute.waypoints_enu,{stroke:'#65a7ec','stroke-width':.032,'stroke-dasharray':'.07 .06','data-layer':'full-plan'});
        path(route.waypoints_enu,{stroke:'#62dbbc','stroke-width':.6,opacity:.17});
        function wall(pose,color,dashed) {
          const transform='translate('+pose[0]+' '+pose[1]+') rotate('+pose[5]*180/Math.PI+')';
          svgEl(g,'rect',{x:-fixture.wall_dimensions_m[0]/2,y:-fixture.wall_dimensions_m[1]/2,width:fixture.wall_dimensions_m[0],height:fixture.wall_dimensions_m[1],transform,fill:color,'fill-opacity':dashed?.14:.86,stroke:color,'stroke-width':.024,'stroke-dasharray':dashed?'.065 .035':'none','data-wall':dashed?'preloaded':'actual'});
        }
        wall(fixture.new_collision_pose,'#ffa458',false);
        wall(fixture.old_collision_pose,'#c9a0ff',true);
        path(route.waypoints_enu,{stroke:'#62dbbc','stroke-width':.035});
        svgEl(g,'circle',{cx:fixture.pivot_enu_xy_m[0],cy:fixture.pivot_enu_xy_m[1],r:.039,fill:'#f1f4e8'});
        if(!zoom){
          const launch=fullRoute.waypoints_enu[0],pad=course.landing_pad_in.center.map(inches);
          svgEl(g,'circle',{cx:launch[0],cy:launch[1],r:.065,fill:'#d5eadd'});
          function note(x,y,caption){const t=svgEl(svg,'text',{x,y:-y,fill:'#d5eadd','font-size':.13,'paint-order':'stroke',stroke:'#0c171f','stroke-width':.04});t.textContent=caption;}
          note(launch[0]-.04,launch[1]-.19,'Launch'); note(pad[0]-.27,pad[1],'Goal pad'); note(fixture.pivot_enu_xy_m[0]+.07,fixture.pivot_enu_xy_m[1]+.06,'M3');
        }
      }
      const g=maps.querySelector('g'), index=Number(slider.value), sample=rows[index], info=panel.querySelector('#wall-coordinates');info.replaceChildren();
      if(!sample)return;
      panel.querySelector('#wall-sec').textContent=sample.t.toFixed(2)+' s';
      for(const [key,color] of Object.entries(colors)){
        if(key==='rig' ? source.value!=='rig' : source.value==='rig')continue;
        let segment=[],previous=-Infinity;
        function flush(){if(segment.length>1)svgEl(g,'polyline',{points:segment.join(' '),fill:'none',stroke:color,'stroke-width':key==='truth'?.032:.022,'stroke-dasharray':key==='px4_fused'?'.08 .05':key==='raw_nvidia'?'.04 .045':'none',opacity:.83,'data-track':key});segment=[];}
        for(let j=0;j<=index;j+=2){const p=rows[j].sources[key];if(!valid(p)){flush();continue;}if(rows[j].t-previous>.6)flush();segment.push(p.xyz.slice(0,2).join(','));previous=rows[j].t;}flush();
        const p=sample.sources[key], el=document.createElement('span');el.style.cssText='display:inline-block;margin:5px 16px 5px 0;color:'+color;
        el.textContent=labels[key]+' · '+(valid(p)?p.xyz.map(x=>x.toFixed(2)).join(', ')+' m ENU':'unavailable');info.append(el);
        if(valid(p))svgEl(g,'circle',{cx:p.xyz[0],cy:p.xyz[1],r:({truth:.084,px4_fused:.067,raw_nvidia:.049,admitted_vo:.029,rig:.055})[key],fill:key==='admitted_vo'||key==='rig'?color:'none',stroke:color,'stroke-width':.017,'data-position':key});
      }
    }
    slider.oninput=()=>{playing=false;play.textContent='Play positions';render();};
    source.onchange=resetSource;
    panel.querySelector('#wall-zoom').onchange=render;
    play.onclick=()=>{playing=!playing;play.textContent=playing?'Pause':'Play positions';if(!playing)return;if(+slider.value>=rows.length-1)slider.value=0;
      const t0=rows[+slider.value].t,wall0=performance.now();
      function tick(){if(!current()||!playing)return;let j=+slider.value;const now=t0+(performance.now()-wall0)/1000;while(j+1<rows.length&&rows[j+1].t<=now)j++;if(j!==+slider.value){slider.value=j;render();}if(j===rows.length-1){playing=false;play.textContent='Play positions';}else requestAnimationFrame(tick);}requestAnimationFrame(tick);
    };
    resetSource();
    const movingLinks=panel.querySelector('#wall-moving-flights');
    for(const item of [...runs].filter(r=>r.observed_flight && r.id.startsWith('nvidia:') && r.files.includes('map-comparison.json'))){
      try{
        const meta=await json(url(item.id,'map-comparison.json'));
        if(!current())return;
        const wall=meta.changed_walls?.find(w=>w.name==='M3' && Math.abs(w.angle_enu_deg+18)<1e-4);
        if(!wall)continue;
        const link=document.createElement('a');link.href='/?run='+encodeURIComponent(item.id);link.textContent='Open real changed-wall flight · '+runName(item);movingLinks.append(link,document.createElement('br'));
      }catch{/* Keep the verified passive recording available if a live archive is incomplete. */}
    }
  } catch (error) {
    if(current()) panel.querySelector('#wall-maps').textContent='Map data unavailable: '+error.message;
  }
}
