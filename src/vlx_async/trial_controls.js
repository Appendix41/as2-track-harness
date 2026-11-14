'use strict';
(() => {
 const el=id=>document.getElementById(id);
 // Depth evidence age and the API call budget are gone: each task allows exactly
 // one DeepSeek plan, requested before the robot moves.
 const mapping={testMode:'mode',testSpeed:'speed_mps',testLateral:'lateral_mps',testYaw:'yaw_rps',
  linearAcceleration:'linear_accel_mps2',yawAcceleration:'yaw_accel_rps2',testDuration:'model_duration_s',
  motionDuration:'motion_duration_s',observedSpeedLimit:'observed_speed_limit_mps',startupTimeout:'startup_timeout_s',maxTravel:'max_travel_m'};
 const fields=Object.keys(mapping);
 // One-time migration: the old saved form carried 5 m/s limits that are unsafe
 // for the monocular route. Speeds are reset once; later edits persist.
 const storeKey='vlxTestForm.v2', legacyKey='vlxTestForm';
 const SAFE_DEFAULTS={testSpeed:'0.05',testLateral:'0.02',testYaw:'0.10',
  linearAcceleration:'0.30',yawAcceleration:'0.60',observedSpeedLimit:'0.12',maxTravel:'2.0'};
 const storeForm=()=>{try{const data=Object.fromEntries(fields.map(id=>[id,el(id).value]));data.taskQueue=el('taskQueue').value;localStorage.setItem(storeKey,JSON.stringify(data))}catch(e){}};
 const migrateForm=()=>{try{if(localStorage.getItem(storeKey))return;
  const legacy=JSON.parse(localStorage.getItem(legacyKey)||'{}');
  localStorage.setItem(storeKey,JSON.stringify({...legacy,...SAFE_DEFAULTS}));
 }catch(e){}};
 const restoreForm=()=>{try{migrateForm();const data=JSON.parse(localStorage.getItem(storeKey)||'{}');for(const id of fields){const node=el(id);if(!node||data[id]==null)continue;if(node.tagName==='SELECT'){if([...node.options].some(o=>o.value===data[id]))node.value=data[id]}else node.value=data[id]}if(typeof data.taskQueue==='string')el('taskQueue').value=data.taskQueue}catch(e){}};
 restoreForm();
 let originalSettings=null,liveExperiment=null;
 let state=null,busy=false,online=false,limitLoaded=false,limitDirty=false;
 let owned=sessionStorage.getItem('vlxTestSession')||null;
 const phases={idle:'未启动',preparing:'准备独立任务',waiting_vlx:'等待 VLX 初始化 / 新鲜输入',
  waiting_plan:'等待本任务 DeepSeek 规划',plan_ready:'计划就绪 · 等待起步',
  waiting_execution_checks:'计划就绪 · 等待运动检查',running_motion:'实机执行中',
  running_shadow:'观察测试 · 不发送运动',task_completed:'任务已完成',stopping:'正在停止',finished:'本轮已结束'};
 const reasons={plan_ready:'本任务计划已就绪',plan_timeout:'本任务规划超时，未出发',
  startup_task_changed:'任务或模型流变化，请重新开始',original_controller_owns_lock:'原控制服务仍持有控制权',
  browser_heartbeat_lost:'操作页面心跳丢失，本轮已停止',test_already_running:'已有任务运行中',
  api_key_unavailable_on_server:'服务端尚未载入 API 密钥',observation_only:'观察测试，不发送运动',
  user_stopped:'已请求停止',model_duration_finished:'任务窗口结束',
  model_duration_finished_without_motion_start:'任务窗口结束，运动未启动',
  model_process_failed:'模型进程失败，请查看本轮日志',motion_window_finished:'任务运动窗口结束',
  task_completed:'任务已完成',execution_rejected_or_stopped:'运动检查拒绝或执行器已停止',
  fresh_execution_packet_required:'等待本轮新鲜执行数据',plan_pending:'等待 DeepSeek 规划',
  plan_rejected:'本任务计划被拒绝，未出发',first_plan_timeout:'首次规划超时，未出发',
  task_timeout:'任务超时结束',route_completed:'已到达任务终点',
  plan_locked:'本任务已锁定，不再推理',route_completed:'已到达任务终点',route_drift:'偏离计划路线',
  plan_requires_reverse:'计划要求掉头，已拒绝',plan_route_above_horizon:'路线落在地平线以上',
  plan_route_too_short:'计划路线过短',low_plan_confidence:'计划置信度不足',
  plan_pose_unavailable:'缺少规划帧位姿',pose_unavailable:'缺少里程计位姿',
  odometry_frame_changed:'里程计坐标系变化',invalid_plan_clock:'计划时钟无效',invalid_vlx_path:'VLX 路径无效',
  plan_not_ready:'计划未就绪',plan_already_active:'本任务已有计划',task_ready:'计划就绪 · 执行中',
  camera_frame_stale:'相机帧过期',prediction_stale:'VLX 预测过期',robot_telemetry_stale:'机器人遥测过期',
  robot_not_in_ready_mode:'机器人不在就绪模式',robot_tilt_invalid:'机身倾角过大',
  observed_speed_limit:'实测速度超过上限',remote_input_or_unknown_state:'遥控器输入或状态未知',
  control_watchdog_expired:'控制更新超时，已停止',motion_duration_complete:'设定运动时间结束',
  transferring_controller:'正在交接空闲控制服务',waiting_fresh_fusion_after_handoff:'控制权已交接，等待当前计划帧',
  restoring_idle_controller:'停止后正在恢复原控制服务',controller_restore_unconfirmed:'控制服务恢复未确认，请查看执行详情'};
 const meaning=x=>reasons[x]||describe(x);
 const describe=x=>x||'—';
 const write=(id,value)=>{const node=el(id);if(node)node.textContent=value};
 const fmt=(x,n=2)=>typeof x==='number'&&Number.isFinite(x)?x.toFixed(n):'—';

 function render(){
  const t=state||{},active=!!t.active;
  if(active&&t.settings){for(const [id,key] of Object.entries(mapping))if(t.settings[key]!=null&&el(id))el(id).value=t.settings[key]}
  const saved=t.saved_execution_limits&&t.saved_execution_limits.observed_speed_limit_mps;
  if(saved!=null&&!active&&!limitDirty){el('observedSpeedLimit').value=saved;limitLoaded=true}
  for(const [id,key] of Object.entries(mapping)){const range=t.limits&&t.limits[key];if(range&&el(id)){el(id).min=range[0];if(range[1]<1e30)el(id).max=range[1];else el(id).removeAttribute('max')}}
  el('useOriginalSettings').disabled=busy||active||!online||!originalSettings;
  el('saveLimits').disabled=busy||active||!online||!limitLoaded;
  write('limitStatus',active?'本轮实测速度上限 '+fmt(t.settings&&t.settings.observed_speed_limit_mps)+' m/s · 运行中锁定':limitDirty?'实测上限已修改；点击保存可供下一轮使用。':saved!=null?'已保存实测上限 '+fmt(saved)+' m/s · 下轮生效':'等待服务端上限配置');
  write('keyStatus',t.api_key_ready?'API 已在服务端内存中配置':'服务端 API 未配置');
  const stack=t.stack||{}, comps=stack.components||{};
  const stackRunning=!!stack.running, stackBusy=stack.phase&&stack.phase!=='idle';
  write('stackState',stackBusy?(stack.phase==='starting'?'正在启动…':'正在停止…'):stackRunning?'运行中':'已停止');
  el('stackState').className='pill '+(stackBusy?'warn':stackRunning?'good':'bad');
  const depthLabel=comps.depth_enabled===false?'已停用':(comps.depth?'✓':'✗');
  write('stackComponents','VLX 模型 '+(comps.model?'✓':'✗')+' · 手动服务 '+(comps.manual?'✓':'✗')+' · 遥测 '+(comps.telemetry?'✓':'✗')+' · 深度障碍 '+depthLabel+(stack.error?' · '+stack.error:''));
  el('stackStart').disabled=busy||stackBusy||stackRunning;
  el('stackStop').disabled=busy||stackBusy||(!stackRunning&&!active);
  el('startTest').disabled=busy||active||!online||!t.api_key_ready||!!t.restart_blocker||!limitLoaded||!stackRunning;
  el('stopTest').disabled=busy||!active;
  el('clearBlocker').disabled=busy||active||!online||!t.restart_blocker;
  for(const id of fields)if(el(id))el(id).disabled=active||busy;
  write('testStatus',(online?'':'连接中断 · ')+(phases[t.phase]||meaning(t.phase)||'等待测试服务')+(t.reason?' · '+meaning(t.reason):'')+(t.task_total?' · 队列 '+Math.min(t.task_index||0,t.task_total)+'/'+t.task_total:'')+(t.instruction?' · 任务：'+t.instruction:''));
  const exp=liveExperiment||{},task=exp.task||{};
  write('pipelineStatus','本任务 DeepSeek：'+(task.plan_calls??0)+' / 1 · 计划：'+(exp.plan_status?meaning(exp.plan_status):'等待')+' · 执行器：'+(t.executor_phase?meaning(t.executor_phase):t.execution_started?'已启动':(t.mode==='shadow'?'观察测试不启动':'未启动'))+' · 已发送运动命令：'+(t.motion_commands_sent??0)+(task.remaining_route_m!=null?' · 剩余路线 '+fmt(task.remaining_route_m)+' m':''));
  write('testModeBadge',active?(t.mode==='execute'?'实机任务 · '+(t.execution_started?'执行器已启动':'尚未出发'):'观察测试 · 不运动'):'测试未运行');
  write('waitTime','等待 VLX '+fmt(t.startup&&t.startup.waiting_elapsed_s,1)+' s');
  write('activeTime','本任务规划 '+fmt(t.active_elapsed_s,1)+' s / '+(t.settings&&t.settings.model_duration_s!=null?t.settings.model_duration_s:'—')+' s');
  write('motionTime',t.execution_started?'运动 '+fmt(t.motion_elapsed_s,1)+' s / '+(t.settings&&t.settings.motion_duration_s!=null?t.settings.motion_duration_s:'—')+' s':'运动未启动');
  write('callCount','DeepSeek '+(task.plan_calls??0)+' / 1');
  if(!active&&owned===t.session_id){owned=null;sessionStorage.removeItem('vlxTestSession')}
 }
 async function post(route,payload){
  if(!state||!state.control_token)throw Error('测试服务不可用');
  const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),3000);
  try{
   const response=await fetch(route,{method:'POST',headers:{'Content-Type':'application/json','X-Test-Token':state.control_token},body:JSON.stringify(payload),signal:abort.signal});
   const result=await response.json();if(!response.ok)throw Error(meaning(result.error));return result;
  }finally{clearTimeout(timer)}
 }
 el('observedSpeedLimit').oninput=()=>{limitDirty=true;render()};
 el('useOriginalSettings').onclick=()=>{
  if(!originalSettings||(state&&state.active))return;
  for(const [id,key] of [['testSpeed','forward_mps'],['testLateral','lateral_mps'],['testYaw','yaw_rps']])el(id).value=originalSettings.speed[key];
  el('linearAcceleration').value=originalSettings.acceleration.linear_mps2;el('yawAcceleration').value=originalSettings.acceleration.yaw_rps2;
  write('parameterNote','已把原网页的速度和加速度填入本轮设置；请同时检查实测速度停止上限。');
 };
 el('saveLimits').onclick=async()=>{
  if(busy||!el('observedSpeedLimit').reportValidity())return;
  busy=true;write('testError','');render();
  try{const result=await post('/test/limits',{observed_speed_limit_mps:Number(el('observedSpeedLimit').value)});state.saved_execution_limits=result.settings;limitDirty=false;limitLoaded=true}
  catch(e){write('testError',String(e.message))}finally{busy=false;render()}
 };
 el('startTest').onclick=async()=>{
  if(busy)return;for(const id of fields)if(el(id)&&!el(id).reportValidity())return;
  storeForm();busy=true;write('testError','');render();
  try{const payload=Object.fromEntries(Object.entries(mapping).map(([id,key])=>[key,key==='mode'?el(id).value:Number(el(id).value)]));
   payload.tasks=el('taskQueue').value.split('\n').map(line=>line.trim()).filter(Boolean);
   const result=await post('/test/start',payload);owned=result.session_id;sessionStorage.setItem('vlxTestSession',owned);
  }catch(e){write('testError',String(e.message))}finally{busy=false;render()}
 };
 el('stackStart').onclick=async()=>{
  if(busy)return;busy=true;write('testError','');try{await post('/test/stack/start',{})}catch(e){write('testError',String(e.message))}finally{busy=false;render()}
 };
 el('stackStop').onclick=async()=>{
  if(busy)return;busy=true;write('testError','');try{await post('/test/stack/stop',{})}catch(e){write('testError',String(e.message))}finally{busy=false;render()}
 };
 el('clearBlocker').onclick=async()=>{
  if(busy)return;busy=true;write('testError','');render();
  try{const result=await post('/test/clear-blocker',{});state.restart_blocker=null;write('parameterNote','已解除阻塞：'+(result.previous||''))}
  catch(e){write('testError',String(e.message))}finally{busy=false;render()}
 };
 el('stopTest').onclick=async()=>{
  if(busy||!state||!state.session_id)return;busy=true;render();
  try{await post('/test/stop',{session_id:state.session_id})}catch(e){write('testError',String(e.message))}finally{busy=false;render()}
 };
 document.addEventListener('change',e=>{if(e.target&&(mapping[e.target.id]||e.target.id==='taskQueue'))storeForm()});
 document.addEventListener('observation-state',e=>{state=(e.detail&&e.detail.test)||{};liveExperiment=(e.detail&&e.detail.experiment)||null;const r=(e.detail&&e.detail.robot)||{};originalSettings=r.speed_limits&&r.acceleration_limits?{speed:r.speed_limits,acceleration:r.acceleration_limits}:null;online=true;render()});
 document.addEventListener('observation-disconnected',()=>{online=false;render()});
 setInterval(()=>{if(online&&owned&&state&&state.active&&state.session_id===owned)post('/test/heartbeat',{session_id:owned}).catch(()=>{})},500);
})();
