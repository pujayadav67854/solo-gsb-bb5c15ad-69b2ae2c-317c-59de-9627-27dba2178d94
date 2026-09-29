'use strict';

/**
 * 滚动窗口奔波负荷（rolling-window travel load）引擎用例：
 *  - 沿分段直线轨迹计算每个完整 W 秒窗口的实际行进路程（首尾停驻不计）；
 *  - 按关键帧时刻及其前移 W 秒的断点精确求 F(s)>D（严格大于，相等不违规）；
 *  - 相接超限范围合并（帐篷形峰值跨折点为一段）；
 *  - W=总时长时起点域退化为 {0}；
 *  - 参数规整与清晰报错（非法 W/D）；
 *  - analyzeScene 的 travel 装配及旧场景（无 travel）兼容；内置样例数值；
 *  - 与分段解析式独立参照在随机轨迹稠密扫描上逐点一致（引擎本身不采样）。
 */
const { test } = require('node:test');
const assert = require('node:assert/strict');
const core = require('../public/core.js');

function approx(actual, expected, tol = 1e-7, label) {
  assert.ok(
    Math.abs(actual - expected) <= tol,
    `${label || '值'} 期望 ${expected}，实际 ${actual}`
  );
}

function performer(keys, extra) {
  return Object.assign({ id: 'p', name: '甲', radius: 0, keys: keys }, extra || {});
}

test('匀速直线：F(s) 恒定，全程超限 / 全程不超限 / 恰等于上限不违规', () => {
  // 10s 内走 100 单位（10/s）
  const p = performer([{ t: 0, x: 0, y: 0 }, { t: 10, x: 100, y: 0 }]);
  const over = core.analyzePerformerTravel(p, 10, { windowSeconds: 4, distanceLimit: 30 });
  assert.equal(over.overloaded, true);
  assert.equal(over.ranges.length, 1);
  approx(over.ranges[0].start, 0);
  approx(over.ranges[0].end, 6);
  approx(over.ranges[0].maxDistance, 40);
  approx(over.ranges[0].maxStart, 0); // 平台型 F：范围最大路程的最早起点为范围左端
  approx(over.maxDistance, 40);
  approx(over.maxStart, 0); // 平台最大路程：最早起点 0
  approx(over.totalDistance, 40);

  const safe = core.analyzePerformerTravel(p, 10, { windowSeconds: 4, distanceLimit: 50 });
  assert.equal(safe.overloaded, false);
  assert.deepEqual(safe.ranges, []);
  approx(safe.maxDistance, 40);

  // 恰等于上限：不违规（严格大于才算）
  const equal = core.analyzePerformerTravel(p, 10, { windowSeconds: 4, distanceLimit: 40 });
  assert.equal(equal.overloaded, false);
  assert.deepEqual(equal.ranges, []);
});

test('首尾停驻不计路程：静止段窗口路程为 0，窗口只覆盖部分运动段', () => {
  // 0~2 静止、2~6 以 100/s 走 400、6~10 静止；W=4，s∈[0,6]
  const p = performer([
    { t: 0, x: 0, y: 0 }, { t: 2, x: 0, y: 0 },
    { t: 6, x: 400, y: 0 }, { t: 10, x: 400, y: 0 }
  ]);
  const r = core.analyzePerformerTravel(p, 10, { windowSeconds: 4, distanceLimit: 0 });
  assert.equal(r.overloaded, true);
  // F(0)=200（窗口 0~4 只覆盖 2~4 的运动），F(2)=400，F(6)=200
  // D=0：s=0 已超限；s=6 时窗口 6~10 静止，F(6)=0 不超限，范围结束于 s=6 之前
  assert.equal(r.ranges.length, 1);
  approx(r.ranges[0].start, 0);
  approx(r.ranges[0].end, 6);
  approx(r.ranges[0].maxDistance, 400);
  approx(r.ranges[0].maxStart, 2);
  approx(r.maxDistance, 400);
  approx(r.maxStart, 2);
  approx(r.totalDistance, 200);
});

test('帐篷形 F(s)：两侧线性求根，相接范围合并为一段', () => {
  // 0~2 静止、2~6 以 100/s 运动、6~8 静止；W=4，s∈[0,4]
  const p = performer([
    { t: 0, x: 0, y: 0 }, { t: 2, x: 0, y: 0 },
    { t: 6, x: 400, y: 0 }, { t: 8, x: 400, y: 0 }
  ]);
  const r = core.analyzePerformerTravel(p, 8, { windowSeconds: 4, distanceLimit: 250 });
  // F(s)=200+100s（s≤2），F(s)=600−100s（s≥2），峰值 400@2
  // F>250 ⇔ s>0.5 且 s<3.5 → 合并为一段 [0.5, 3.5]，范围最大路程 400@2
  assert.equal(r.ranges.length, 1);
  approx(r.ranges[0].start, 0.5);
  approx(r.ranges[0].end, 3.5);
  approx(r.ranges[0].maxDistance, 400);
  approx(r.ranges[0].maxStart, 2);
  approx(r.maxDistance, 400);
  approx(r.maxStart, 2);

  // 恰等于峰值的上限：不违规
  const eq = core.analyzePerformerTravel(p, 8, { windowSeconds: 4, distanceLimit: 400 });
  assert.equal(eq.overloaded, false);
  assert.deepEqual(eq.ranges, []);
});

test('多折点不同速度：范围在内部折点处相接自动合并', () => {
  // [0,4] 速度 100、[4,10] 速度 50；W=4，s∈[0,6]
  // F(s)：s=0:400 → s∈[0,?]；构造 D 使两段在折点 s=4 处同为超限
  const p = performer([
    { t: 0, x: 0, y: 0 },
    { t: 4, x: 400, y: 0 },
    { t: 10, x: 700, y: 0 }
  ]);
  // 数值参照：直接积分
  function ref(s) {
    const segs = [[0, 4, 100], [4, 10, 50]];
    return segs.reduce((acc, [a, b, v]) => {
      const lo = Math.max(a, s), hi = Math.min(b, s + 4);
      return acc + (hi > lo ? v * (hi - lo) : 0);
    }, 0);
  }
  const D = 300;
  const r = core.analyzePerformerTravel(p, 10, { windowSeconds: 4, distanceLimit: D });
  // 用 0.001 步长稠密核对每个窗口起点的超限归属
  const N = 6000;
  for (let i = 0; i <= N; i++) {
    const s = 6 * i / N;
    const f = ref(s);
    if (Math.abs(f - D) < 1e-6) continue;
    assert.equal(core.isTravelOverAt(r, s), f > D, `s=${s} F=${f}`);
  }
  // F(0)=400 超限；右端 F(6)=200 不超限 → 范围必从 0 开始、在 (0,6) 内结束
  assert.equal(r.ranges.length >= 1, true);
  approx(r.ranges[0].start, 0);
  // 范围相接于折点（若存在分界）必须被合并成一段
  assert.equal(r.ranges.length, 1);
  // 范围终点是 F=300 的根
  approx(ref(r.ranges[0].end), 300, 1e-6);
});

test('W 等于总时长：起点域只有 {0}，只检查起点 0', () => {
  const p = performer([{ t: 0, x: 0, y: 0 }, { t: 12, x: 880, y: 0 }]);
  const over = core.analyzePerformerTravel(p, 12, { windowSeconds: 12, distanceLimit: 800 });
  assert.equal(over.overloaded, true);
  assert.equal(over.ranges.length, 1);
  assert.deepEqual([over.ranges[0].start, over.ranges[0].end], [0, 0]);
  approx(over.ranges[0].maxDistance, 880);
  approx(over.maxDistance, 880);
  approx(over.maxStart, 0);

  const safe = core.analyzePerformerTravel(p, 12, { windowSeconds: 12, distanceLimit: 880 });
  assert.equal(safe.overloaded, false);
  assert.deepEqual(safe.ranges, []);
  approx(safe.maxDistance, 880); // 最大路程仍返回，便于展示
});

test('路程与安全半径无关：改半径不改变结果', () => {
  const keys = [{ t: 0, x: 0, y: 0 }, { t: 5, x: 30, y: 40 }]; // 50 单位
  const r0 = core.analyzePerformerTravel(performer(keys, { radius: 0 }), 5,
    { windowSeconds: 5, distanceLimit: 40 });
  const r1 = core.analyzePerformerTravel(performer(keys, { radius: 60 }), 5,
    { windowSeconds: 5, distanceLimit: 40 });
  assert.equal(r0.overloaded, true);
  assert.equal(r1.overloaded, true);
  approx(r1.maxDistance, 50);
});

test('多段往返路径：窗口路程按实际折线长度（非位移）', () => {
  // 0~2: 0→200，2~4: 200→0，W=4，T=4：实际路程 400，净位移 0
  const p = performer([
    { t: 0, x: 0, y: 0 },
    { t: 2, x: 200, y: 0 },
    { t: 4, x: 0, y: 0 }
  ]);
  const r = core.analyzePerformerTravel(p, 4, { windowSeconds: 4, distanceLimit: 300 });
  assert.equal(r.overloaded, true);
  approx(r.maxDistance, 400);
});

test('normalizeTravel：合法规整、钳制、非法 W/D 清晰报错', () => {
  assert.deepEqual(core.normalizeTravel({ windowSeconds: 4, distanceLimit: 10 }, 12),
    { windowSeconds: 4, distanceLimit: 10 });
  // W 恰等于总时长合法
  assert.deepEqual(core.normalizeTravel({ windowSeconds: 12, distanceLimit: 0 }, 12),
    { windowSeconds: 12, distanceLimit: 0 });
  // 浮点超出钳制
  approx(core.normalizeTravel({ windowSeconds: 12 + 1e-9, distanceLimit: 0 }, 12).windowSeconds, 12);

  assert.throws(() => core.normalizeTravel(null, 12), /travel/);
  assert.throws(() => core.normalizeTravel({}, 12), /windowSeconds/);
  assert.throws(() => core.normalizeTravel({ windowSeconds: 0, distanceLimit: 1 }, 12), /正数/);
  assert.throws(() => core.normalizeTravel({ windowSeconds: -2, distanceLimit: 1 }, 12), /正数/);
  assert.throws(() => core.normalizeTravel({ windowSeconds: NaN, distanceLimit: 1 }, 12), /正数/);
  assert.throws(() => core.normalizeTravel({ windowSeconds: 'abc', distanceLimit: 1 }, 12), /正数/);
  assert.throws(() => core.normalizeTravel({ windowSeconds: 13, distanceLimit: 1 }, 12), /不能超过总时长/);
  assert.throws(() => core.normalizeTravel({ windowSeconds: 4, distanceLimit: -1 }, 12), /非负/);
  assert.throws(() => core.normalizeTravel({ windowSeconds: 4, distanceLimit: NaN }, 12), /非负/);
});

test('analyzeScene：携带 travel 时逐人返回；缺省时 travel 为 null（旧场景兼容）', () => {
  const scene = {
    duration: 10,
    performers: [
      { id: 'a', name: '甲', radius: 0, keys: [{ t: 0, x: 0, y: 0 }, { t: 10, x: 1000, y: 0 }] },
      { id: 'b', name: '乙', radius: 0, keys: [{ t: 0, x: 0, y: 0 }, { t: 10, x: 10, y: 0 }] }
    ],
    travel: { windowSeconds: 5, distanceLimit: 200 }
  };
  const r = core.analyzeScene(scene);
  assert.ok(r.travel);
  assert.deepEqual(r.travel.config, { windowSeconds: 5, distanceLimit: 200 });
  assert.equal(r.travel.travelResults.length, 2);
  const a = r.travel.travelResults.find((x) => x.performerId === 'a');
  const b = r.travel.travelResults.find((x) => x.performerId === 'b');
  assert.equal(a.overloaded, true); // 500/5s
  assert.equal(b.overloaded, false); // 5/5s
  approx(a.maxDistance, 500);
  approx(r.travel.maxDistance, 500);
  assert.equal(r.travel.maxDistancePerformerId, 'a');

  const old = core.analyzeScene({ duration: 10, performers: scene.performers });
  assert.equal(old.travel, null);

  assert.throws(() => core.analyzeScene(Object.assign({}, scene, { travel: { windowSeconds: 11, distanceLimit: 0 } })),
    /不能超过总时长/);
});

test('内置样例：travel 参数与各人奔波结果数值', () => {
  const s = core.builtinScene();
  assert.deepEqual(s.travel, { windowSeconds: 4, distanceLimit: 500 });
  const r = core.analyzeScene(s);
  assert.ok(r.travel);
  const byName = {};
  r.travel.travelResults.forEach((x) => { byName[x.performerName] = x; });
  // 甲 880/12≈73.33/s → 4s≈293.33，不超限
  approx(byName['甲·横贯'].maxDistance, 880 * 4 / 12, 1e-6);
  assert.equal(byName['甲·横贯'].overloaded, false);
  // 乙 520/12≈43.33/s → 4s≈173.33，不超限
  approx(byName['乙·纵穿'].maxDistance, 520 * 4 / 12, 1e-6);
  assert.equal(byName['乙·纵穿'].overloaded, false);
  // 丙存在超限范围，最大窗口路程在 s=0（两段各 2s），且其行最大 > 500
  assert.equal(byName['丙·错峰展伞'].overloaded, true);
  approx(byName['丙·错峰展伞'].maxStart, 0);
  assert.ok(byName['丙·错峰展伞'].maxDistance > 500);
});

test('随机轨迹稠密扫描：超限归属、范围端点与分段解析式独立参照逐点一致', () => {
  function rnd(a, b) { return a + Math.random() * (b - a); }
  // 独立参照：按规整后的关键帧直接分段积分
  function refDist(keys, T, W, s) {
    const ks = core.normalizeKeys(keys, 0);
    const pts = [{ t: 0, x: ks[0].x, y: ks[0].y }]
      .concat(ks.map((k) => ({ t: k.t, x: k.x, y: k.y })))
      .concat([{ t: T, x: ks[ks.length - 1].x, y: ks[ks.length - 1].y }]);
    let d = 0;
    for (let i = 0; i < pts.length - 1; i++) {
      const a = pts[i], b = pts[i + 1];
      const lo = Math.max(a.t, s), hi = Math.min(b.t, s + W);
      if (hi > lo) d += Math.hypot(b.x - a.x, b.y - a.y) / (b.t - a.t) * (hi - lo);
    }
    return d;
  }

  for (let trial = 0; trial < 40; trial++) {
    const T = rnd(2, 20);
    const nk = 2 + Math.floor(Math.random() * 6);
    const ts = [0];
    for (let i = 1; i < nk - 1; i++) ts.push(+rnd(0, T).toFixed(2));
    ts.push(T);
    ts.sort((a, b) => a - b);
    const keys = ts.map((t) => ({ t, x: rnd(-500, 500), y: rnd(-500, 500) }));
    const p = performer(keys, { radius: Math.floor(Math.random() * 30) });
    const W = Math.random() < 0.15 ? T : rnd(0.2, T);
    const total = refDist(keys, T, W, 0);
    const Ds = [0, total * 0.25, total * 0.5, total * 0.75, total, total * 1.1, rnd(0, Math.max(total, 1))];
    for (const D of Ds) {
      const r = core.analyzePerformerTravel(p, T, { windowSeconds: W, distanceLimit: D });
      const smax = T - W;
      const N = 2000;
      for (let i = 0; i <= N; i++) {
        const s = smax * i / N;
        const f = refDist(keys, T, W, s);
        if (Math.abs(f - D) <= 1e-6 * Math.max(1, Math.abs(D))) continue;
        assert.equal(core.isTravelOverAt(r, s), f > D,
          `trial=${trial} s=${s.toFixed(4)} F=${f} D=${D}`);
      }
      // 范围端点（非起点域边界）必为 F=D 的根
      for (const rg of r.ranges) {
        [[rg.start, 0], [rg.end, smax]].forEach(([s, bnd]) => {
          if (Math.abs(s - bnd) > 1e-6) {
            approx(refDist(keys, T, W, s), D, 1e-5 * Math.max(1, Math.abs(D)), '范围端点为根');
          }
        });
      }
      // 全局最大路程与独立参照一致：F 分段线性，独立枚举断点并集求精确最大
      const ksN = core.normalizeKeys(keys, 0);
      const candS = [0, smax];
      ksN.forEach((k) => { [k.t, k.t - W].forEach((t) => {
        if (t > 0 && t < smax) candS.push(t);
      }); });
      let refMax = 0;
      candS.forEach((s) => { refMax = Math.max(refMax, refDist(keys, T, W, s)); });
      approx(r.maxDistance, refMax, 1e-7 * Math.max(1, refMax), '最大路程');
      // 最早最大起点同样取最早达到精确最大值的断点
      candS.sort((a, b) => a - b);
      assert.ok(candS.some((s) =>
        Math.abs(s - r.maxStart) <= 1e-5 && Math.abs(refDist(keys, T, W, s) - refMax) <= 1e-5),
        '最大路程最早起点应是达到精确最大值的断点');
    }
  }
});
