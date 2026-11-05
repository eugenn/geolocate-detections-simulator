require('./core.js'); const G=globalThis.GLD; const ref=require('./ref.json');
let worst=0;
for (const c of ref.cases){ const i=c.in;
  const m={u:i.u,v:i.v,W:ref.W,H:ref.H,hfov:ref.hfov,pan:i.pan,tilt:i.tilt,yaw:i.yaw,pitch:i.pitch,roll:i.roll,pos:i.pos,lever:[0,0,0],surf:{off:0,slope:0,az:0,dn:0,de:0}};
  const r=G.chain(m); const o=c.out;
  const dd=Math.max(...r.dned.map((x,k)=>Math.abs(x-o.dir[k])));
  const dg=o.ground&&r.hit?Math.max(...[0,1,2].map(k=>Math.abs(r.hit.p[k]-o.ground[k]))):(o.ground||r.hit?1e9:0);
  worst=Math.max(worst,dg); console.log(JSON.stringify(i), 'dir', dd.toExponential(1),'ground', dg.toExponential(1), 'dep', (r.dep-o.depression_deg).toExponential(1));
}
console.log('worst ground diff m', worst);
// injection
for (const r of ref.inject){ const m={u:960,v:540,W:1920,H:1080,hfov:45,pan:0,tilt:-r.dep,yaw:0,pitch:0,roll:0,pos:[0,0,-100],surf:{off:0,slope:0,az:0,dn:0,de:0}};
 const b=G.groundOf(m); const p=G.clone(m);
 if(r.source==='tilt')p.tilt-=0.5; if(r.source==='pitch')p.pitch+=0.5; if(r.source==='heading')p.yaw+=0.5; if(r.source==='roll')p.roll+=0.5; if(r.source==='vertical')p.pos[2]+=2.5;
 const g=G.groundOf(p); console.log(r.dep,r.source,(Math.hypot(g[0]-b[0],g[1]-b[1])).toFixed(4),'py',r.measured.toFixed(4)); }
const m={u:960,v:540,W:1920,H:1080,hfov:45,pan:0,tilt:-45,yaw:0,pitch:0,roll:0,pos:[0,0,-100],surf:{off:0,slope:0,az:0,dn:0,de:0}};
const c=G.chain(m);
const sg={heading:0.5,att:0,bore:0,vert:0,gnss:2,survey:3,terrain:0.3,dt:2,rate:90,speed:0,cam:0.3,box:5};
const st=G.specTerms(c,m,sg); console.log(st.terms, st.tol, st.dom, 'py', ref.terms.composed, ref.terms.dominant);
const sg2={heading:1,att:0.2,bore:0.1,vert:3,gnss:2,survey:0,terrain:0.3,dt:35,rate:3,speed:0,cam:0.3,box:5};
const J=G.jacobianCov(m,sg2); const ac=G.alongCross(J.cov,c.az); console.log('along',ac.along.toFixed(2),'cross',ac.cross.toFixed(2),'r95',G.radiusP(J.cov).toFixed(2));
const pts=G.monteCarlo(m,sg2,4000,1); const rr=pts.map(p=>Math.hypot(...p)); console.log('MC r95', G.percentile(rr,0.95).toFixed(2), 'spec tol', G.specTerms(c,m,sg2).tol.toFixed(2), 'cov spec', (rr.filter(x=>x<=G.specTerms(c,m,sg2).tol).length/rr.length).toFixed(3), 'cov r95', (rr.filter(x=>x<=G.radiusP(J.cov)).length/rr.length).toFixed(3));
const base={h:100,dep:45,W:1920,H:1080,hfov:45};
const t0=Date.now(); const fx=G.fusionExperiment(base,sg2,0.8,1500,7); console.log('fusion', JSON.stringify(Object.fromEntries(Object.entries(fx.r95).map(([k,v])=>[k,+v.toFixed(2)]))), Date.now()-t0,'ms');
