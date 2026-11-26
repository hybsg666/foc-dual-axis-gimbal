# -*- coding: utf-8 -*-
"""
生成双轴指向云台打印件（arm + clamp）的网页预览 preview.html。
原理：把 STL 转成 base64 内嵌进 HTML，浏览器端用 three.js 解析渲染。
坐标映射：STL(x,y,z) -> three(x,z,-y)。

★ 本版新增：任意两点距离标注（黄色线 + 中点文字），可一键开关。
   标注的孔位坐标与 generate_gimbal_stl.py 里的常量一一对应，改尺寸要两边一起改。
"""
import base64
import os

HERE = os.path.dirname(os.path.abspath(__file__))
STL = os.path.join(HERE, 'stl')

# ===== 与 generate_gimbal_stl.py 保持一致的孔位参数 =====
# 转子 3 孔品字形（相邻中心距 10mm）外接圆半径
SCW = 10.0 / (3.0 ** 0.5)      # ≈ 5.7735
# 定子 4 孔正方形偏移（相邻 21mm / 对角 29.7mm）
MOFF = 10.5
# 布局
ARM_X = 40.0
ARM_Z = 30.0
CLAMP_Y = 24.0                 # STL y → three.z = -24


def b64(name):
    with open(os.path.join(STL, name), 'rb') as f:
        return base64.b64encode(f.read()).decode('ascii')


ARM = b64('arm.stl')
CLAMP = b64('clamp.stl')

html = r'''<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>双轴云台打印件 3D 预览（L 臂 + 夹笔座）</title>
<style>
  html,body{margin:0;height:100%;overflow:hidden;font-family:system-ui,-apple-system,'Segoe UI',sans-serif;}
  #c{display:block;width:100%;height:100%;}
  #legend{position:absolute;top:16px;left:16px;background:rgba(28,30,34,.94);border:1px solid #3a3d43;border-radius:10px;padding:12px 16px;font-size:13px;color:#e8e8e6;line-height:2;}
  #legend .dot{display:inline-block;width:12px;height:12px;border-radius:3px;margin-right:8px;vertical-align:-1px;}
  #legend .hd{display:inline-block;width:12px;height:12px;border-radius:50%;margin-right:8px;vertical-align:-1px;background:#ff3355;}
  #legend .dim{display:inline-block;width:14px;height:3px;border-radius:2px;margin-right:8px;vertical-align:2px;background:#ffd24a;}
  #panel{position:absolute;top:16px;right:16px;background:rgba(28,30,34,.95);border:1px solid #3a3d43;border-radius:10px;padding:12px 14px;font-size:12px;color:#e8e8e6;}
  #panel .ttl{font-size:13px;font-weight:600;margin-bottom:8px;color:#fff;}
  #panel table{border-collapse:collapse;}
  #panel th,#panel td{border:1px solid #3a3d43;padding:4px 8px;text-align:center;white-space:nowrap;}
  #panel th{background:#23262b;font-weight:600;color:#b9bec6;}
  #panel td.k{color:#ffd24a;font-weight:700;}
  #panel td.s{color:#7fd8b4;}
  #panel .tog{display:block;margin-top:10px;cursor:pointer;color:#b9bec6;user-select:none;}
  #panel .tog input{vertical-align:-2px;margin-right:6px;}
  #panel .note{margin-top:8px;color:#8b9099;line-height:1.6;max-width:300px;}
  #hint{position:absolute;bottom:14px;left:50%;transform:translateX(-50%);background:rgba(28,30,34,.92);border:1px solid #3a3d43;border-radius:20px;padding:6px 16px;font-size:12px;color:#aaa;white-space:nowrap;}
</style>
</head>
<body>
<canvas id="c"></canvas>

<div id="legend">
  <div><span class="dot" style="background:#1D9E75"></span>L 型连接臂 arm.stl（套A转子 + 固定B定子）</div>
  <div><span class="dot" style="background:#3b82c4"></span>夹笔座 clamp.stl（套B转子顶部 + φ8笔套筒）</div>
  <div><span class="dot" style="background:#b9bec6"></span>半透明 = 电机位置示意（A竖 / B横）</div>
  <div><span class="hd"></span>红点 = 螺丝孔中心</div>
  <div><span class="dim"></span>黄线 = 孔中心距（数字单位 mm）</div>
</div>

<div id="panel">
  <div class="ttl">孔位尺寸（mm）</div>
  <table>
    <tr><th>位置</th><th>孔组</th><th>孔数</th><th>孔径</th><th>中心距</th></tr>
    <tr>
      <td>arm 水平盘<br><span style="color:#8b9099">套 A 转子顶</span></td>
      <td>品字形</td><td>3</td><td>φ3.4</td>
      <td class="k">两两 10.0</td>
    </tr>
    <tr>
      <td>arm 水平盘 中央</td><td>凸台定位</td><td>1</td><td>φ9.6</td>
      <td class="s">同心</td>
    </tr>
    <tr>
      <td>arm 竖直盘<br><span style="color:#8b9099">固定 B 定子</span></td>
      <td>正方形</td><td>4</td><td>φ3.4</td>
      <td class="k">相邻 21.0<br>对角 29.7</td>
    </tr>
    <tr>
      <td>arm 竖直盘 中央</td><td>螺丝头避让</td><td>1</td><td>φ8.0</td>
      <td class="s">同心</td>
    </tr>
    <tr>
      <td>clamp 安装盘<br><span style="color:#8b9099">套 B 转子顶</span></td>
      <td>品字形</td><td>3</td><td>φ3.4</td>
      <td class="k">两两 10.0</td>
    </tr>
    <tr>
      <td>clamp 笔套筒</td><td>笔孔</td><td>1</td><td>φ8.0</td>
      <td class="s">外径 φ18</td>
    </tr>
  </table>
  <label class="tog"><input type="checkbox" id="cbDim" checked>显示距离标注</label>
  <div class="note">品字形 = 3 孔等边三角形，外接圆 φ11.55；相邻中心距 10.0。<br>正方形 = 4 孔在 ±10.5 处，相邻 21.0、对角 29.7。</div>
</div>

<div id="hint">鼠标左键拖拽旋转 · 滚轮缩放 · 右键平移 · 黄线标的是孔中心距</div>

<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/gh/mrdoob/three.js@r128/examples/js/controls/OrbitControls.js"></script>
<script>
var ARM="__ARM__";
var CLAMP="__CLAMP__";

function parseStl(b64){
  var bin=atob(b64), len=bin.length, bytes=new Uint8Array(len);
  for(var i=0;i<len;i++) bytes[i]=bin.charCodeAt(i);
  var view=new DataView(bytes.buffer);
  var n=view.getUint32(80,true);
  var pos=new Float32Array(n*9);
  var off=84;
  for(var t=0;t<n;t++){
    off+=12;
    for(var v=0;v<3;v++){
      pos[t*9+v*3]   = view.getFloat32(off,true);      // STL.x -> three.x
      pos[t*9+v*3+1] = view.getFloat32(off+8,true);    // STL.z -> three.y
      pos[t*9+v*3+2] = -view.getFloat32(off+4,true);   // STL.y -> three.z
      off+=12;
    }
    off+=2;
  }
  var g=new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos,3));
  g.computeVertexNormals();
  return g;
}

var scene=new THREE.Scene();
scene.background=new THREE.Color(0x14161a);

var renderer=new THREE.WebGLRenderer({canvas:document.getElementById('c'), antialias:true});
renderer.setPixelRatio(Math.min(window.devicePixelRatio,2));

var camera=new THREE.PerspectiveCamera(45,1,0.1,600);
camera.position.set(115,75,145);

var controls=new THREE.OrbitControls(camera, renderer.domElement);
controls.target.set(32,16,-6);
controls.update();

scene.add(new THREE.HemisphereLight(0xffffff,0x30333a,0.85));
var d1=new THREE.DirectionalLight(0xffffff,0.6); d1.position.set(1,1.5,1); scene.add(d1);
var d2=new THREE.DirectionalLight(0xffffff,0.3); d2.position.set(-1,-0.5,-1); scene.add(d2);

// L 臂本体
var arm=new THREE.Mesh(parseStl(ARM), new THREE.MeshPhongMaterial({color:0x1D9E75, side:THREE.DoubleSide, shininess:35}));
scene.add(arm);

// 夹笔座本体
var clamp=new THREE.Mesh(parseStl(CLAMP), new THREE.MeshPhongMaterial({color:0x3b82c4, side:THREE.DoubleSide, shininess:35}));
scene.add(clamp);

// 半透明电机示意
var ghost=new THREE.MeshPhongMaterial({color:0x999999,transparent:true,opacity:0.18,side:THREE.DoubleSide});
// 电机A：竖直，转子顶面在 three.y=0，本体在 y<0
var mA=new THREE.Mesh(new THREE.CylinderGeometry(15,15,16,48),ghost); mA.position.set(0,-8,0); scene.add(mA);
// 电机B：水平，轴沿 three.z，定子在 three.z=0（接 arm 竖直盘），转子顶部在 three.z=-24（接 clamp）
var mB=new THREE.Mesh(new THREE.CylinderGeometry(15,15,24,48),ghost);
mB.rotation.x=Math.PI/2; mB.position.set(40,30,-12); scene.add(mB);

// ============================================================
// 尺寸标注：黄色细杆 + 端点小球 + 中点文字
// 全部塞进 dimGroup，一个勾选框就能整组显隐
// ============================================================
var dimGroup=new THREE.Group();
scene.add(dimGroup);

var DIM_COLOR=0xffd24a;

function roundRect(ctx,x,y,w,h,r){
  ctx.beginPath();
  ctx.moveTo(x+r,y);
  ctx.lineTo(x+w-r,y); ctx.quadraticCurveTo(x+w,y,x+w,y+r);
  ctx.lineTo(x+w,y+h-r); ctx.quadraticCurveTo(x+w,y+h,x+w-r,y+h);
  ctx.lineTo(x+r,y+h); ctx.quadraticCurveTo(x,y+h,x,y+h-r);
  ctx.lineTo(x,y+r); ctx.quadraticCurveTo(x,y,x+r,y);
  ctx.closePath();
}

// 用 canvas 画文字 -> 纹理 -> Sprite（永远面向相机，永远可见）
function makeLabel(text){
  var fs=30, padX=12, padY=8;
  var cv=document.createElement('canvas');
  var ctx=cv.getContext('2d');
  ctx.font='bold '+fs+'px system-ui,-apple-system,sans-serif';
  var tw=ctx.measureText(text).width;
  cv.width=Math.ceil(tw)+padX*2;
  cv.height=fs+padY*2;
  ctx=cv.getContext('2d');
  ctx.font='bold '+fs+'px system-ui,-apple-system,sans-serif';
  ctx.textAlign='center'; ctx.textBaseline='middle';
  ctx.fillStyle='rgba(10,12,16,0.88)';
  roundRect(ctx,0.5,0.5,cv.width-1,cv.height-1,9); ctx.fill();
  ctx.strokeStyle='rgba(255,210,74,0.75)'; ctx.lineWidth=2; ctx.stroke();
  ctx.fillStyle='#ffd24a';
  ctx.fillText(text, cv.width/2, cv.height/2+1);
  var tex=new THREE.CanvasTexture(cv);
  tex.minFilter=THREE.LinearFilter;
  var sp=new THREE.Sprite(new THREE.SpriteMaterial({map:tex,transparent:true,depthTest:false,depthWrite:false}));
  var worldH=4.6;                       // 标签在世界坐标里的高度（mm）
  sp.scale.set(worldH*cv.width/cv.height, worldH, 1);
  sp.renderOrder=1001;
  return sp;
}

// 在 p1-p2 之间画一根细杆
function bar(p1,p2,radius){
  var v1=new THREE.Vector3(p1[0],p1[1],p1[2]);
  var v2=new THREE.Vector3(p2[0],p2[1],p2[2]);
  var dir=new THREE.Vector3().subVectors(v2,v1);
  var len=dir.length();
  if(len<1e-6) return;
  var geo=new THREE.CylinderGeometry(radius,radius,len,10);
  var mat=new THREE.MeshBasicMaterial({color:DIM_COLOR,depthTest:false});
  var m=new THREE.Mesh(geo,mat);
  m.position.copy(v1).add(dir.clone().multiplyScalar(0.5));
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0), dir.clone().normalize());
  m.renderOrder=1000;
  dimGroup.add(m);
}

function dot(p,r){
  var s=new THREE.Mesh(new THREE.SphereGeometry(r,12,12),
        new THREE.MeshBasicMaterial({color:0xff3355,depthTest:false}));
  s.position.set(p[0],p[1],p[2]);
  s.renderOrder=1000;
  dimGroup.add(s);
}

// 标注两点距离：画线 + 端点 + 中点文字
function dim(p1,p2,text,labelOffset){
  bar(p1,p2,0.42);
  dot(p1,1.15); dot(p2,1.15);
  var mid=new THREE.Vector3(
    (p1[0]+p2[0])/2, (p1[1]+p2[1])/2, (p1[2]+p2[2])/2);
  if(labelOffset) mid.add(new THREE.Vector3(labelOffset[0],labelOffset[1],labelOffset[2]));
  var sp=makeLabel(text);
  sp.position.copy(mid);
  dimGroup.add(sp);
}

// ---------- ① arm 水平盘：3 孔品字形，两两 10mm ----------
// 孔位 STL(x,y) -> three(x, 5.9, -y)，抬高到盘顶面之上，避免被盘体挡住
var HY=5.9;
var A0=[ __SCW__, HY, 0];
var A1=[__NSCW_H__, HY, -5];
var A2=[__NSCW_H__, HY,  5];
dim(A0,A1,'10.0',[ 2.0, 3.0,  0.0]);
dim(A1,A2,'10.0',[-4.5, 3.0,  0.0]);
dim(A2,A0,'10.0',[ 2.0, 3.0,  0.0]);

// ---------- ② arm 竖直盘：4 孔正方形，相邻 21 / 对角 29.7 ----------
var VZ=3.4;                      // 浮在竖直盘前表面（three.z≈+2.5）之外
var VX=__ARM_X__, VY=__ARM_Z__, OFF=__MOFF__;
var P00=[VX-OFF, VY+OFF, VZ];
var P10=[VX+OFF, VY+OFF, VZ];
var P11=[VX+OFF, VY-OFF, VZ];
var P01=[VX-OFF, VY-OFF, VZ];
dim(P00,P10,'21.0',[0.0, 3.2, 0.0]);     // 上边
dim(P00,P01,'21.0',[-7.0, 0.0, 0.0]);    // 左边
dim(P00,P11,'29.7',[3.5, -4.0, 0.0]);    // 对角

// ---------- ③ clamp 安装盘：3 孔品字形，两两 10mm ----------
// 孔位 STL(40+hx, 24, 30+hy) -> three(40+hx, 30+hy, -24)
var CZ=-21.2;                    // 浮在安装盘靠相机那面（three.z≈-21.5）之外
var B0=[__BC0__, VY, CZ];
var B1=[__BC1__, VY+5, CZ];
var B2=[__BC1__, VY-5, CZ];
dim(B0,B1,'10.0',[ 2.0, 3.0, 0.0]);
dim(B1,B2,'10.0',[-4.5, 0.0, 0.0]);
dim(B2,B0,'10.0',[ 2.0,-3.0, 0.0]);

// 勾选框控制显隐
document.getElementById('cbDim').addEventListener('change', function(e){
  dimGroup.visible = e.target.checked;
});

function resize(){
  var w=window.innerWidth,h=window.innerHeight;
  renderer.setSize(w,h);
  camera.aspect=w/h; camera.updateProjectionMatrix();
}
window.addEventListener('resize',resize); resize();

function loop(){ requestAnimationFrame(loop); controls.update(); renderer.render(scene,camera); }
loop();
</script>
</body>
</html>'''

html = (html
        .replace('__ARM__', ARM)
        .replace('__CLAMP__', CLAMP)
        .replace('__SCW__', '%.4f' % SCW)
        .replace('__NSCW_H__', '%.4f' % (-SCW * 0.5))
        .replace('__BC0__', '%.4f' % (ARM_X + SCW))
        .replace('__BC1__', '%.4f' % (ARM_X - SCW * 0.5))
        .replace('__ARM_X__', '%.1f' % ARM_X)
        .replace('__ARM_Z__', '%.1f' % ARM_Z)
        .replace('__MOFF__', '%.1f' % MOFF))

out = os.path.join(HERE, 'preview.html')
with open(out, 'w', encoding='utf-8') as f:
    f.write(html)

print('written:', out)
print('size:', os.path.getsize(out), 'bytes')
