'use strict';

/**
 * 滚动窗口奔走监控（travel load）引擎用例：
 *  - normalizeTravelLoad 规整（W>0 且 ≤duration、D≥0；未配置为 null；非法抛错）；
 *  - 匀速/分段轨迹的窗口路程 L(s)=A(s+W)−A(s) 与超限起点范围；
 *  - 恰等于上限不违规（严格超过）；首尾停驻不计路程；
 *  - 相接范围合并；范围端点为 L=D 的精确临界起点；
 *  - W=总时长时只检查起点 0（零长度范围）；
 *  - 最大窗口路程与最早起点（平台取最早）；
 *  - analyzeScene 的 travelLoad/travelResults 装配与旧场景（缺省/空）兼容；
 *  - 内置样例数值；与稠密采样在随机轨迹上逐点一致（实现本身解析、零采样）。
 */
const { test } = require('node:test');
const assert = require('node:assert/strict');
const core = require('../public/core.js');

function approx(actual, expected, tol = 1e-6, label) {
  assert.ok(
    Math.abs(actual - expected) <= tol,
    `${label || '值'} 期望 ${expected}，实际 ${actual}`
  );
}

function mover(keys, extra) {
  return Object.assign({ id: 'a', name: '甲', radius: 5, keys: keys }, extra || {});
}

// 稠密参考实现：沿关键帧分段直线逐段累计弧长表，再按细网格求窗口路程
function referenceL(keys, duration, W, sN) {
  const ks = core.normalizeKeys(keys, 0);
  const N = sN || 4000;
  const S = duration - W;
  const samples = [];
  for (let i = 0; i <= N; i++) {
    const s = (S * i) / N;
    samples.push({
      s,
      L: core.buildSegments(ks, duration, 0).reduce((acc, seg) => {
        const a = Math.max(seg.t0, s);
        const b = Math.min(seg.t1, s + W);
        if (b <= a) return acc;
        return acc + Math.hypot(seg.vx, seg.vy) * (b - a);
      }, 0)
    });
  }
  return samples;
}

test('奔走监控：normalizeTravelLoad 规整与缺省/非法', () => {
  assert.equal(core.normalizeTravelLoad(null, 12), null);
  assert.equal(core.normalizeTravelLoad(undefined, 12), null);
  assert.deepEqual(core.normalizeTravelLoad({ window: 4, distance: 600 }, 12),
    { window: 4, distance: 600 });
  // W 略超总时长被钳制
  approx(core.normalizeTravelLoad({ window: 12 + 1e-9, distance: 0 }, 12).window, 12, 1e-9);
  assert.throws(() => core.normalizeTravelLoad({ window: 0, distance: 1 }, 12), /window/);
  assert.throws(() => core.normalizeTravelLoad({ window: -2, distance: 1 }, 12), /window/);
  assert.throws(() => core.normalizeTravelLoad({ window: 'x', distance: 1 }, 12), /window/);
  assert.throws(() => core.normalizeTravelLoad({ window: 12.5, distance: 1 }, 12), /总时长/);
  assert.throws(() => core.normalizeTravelLoad({ window: 4 }, 12), /distance/);
  assert.throws(() => core.normalizeTravelLoad({ window: 4, distance: -1 }, 12), /distance/);
  assert.throws(() => core.normalizeTravelLoad(42, 12), /travelLoad/);
});

test('奔走监控：匀速 100/s、W=2、D=150 全程超限，范围 [0,8]，最大路程 200 自 0 起', () => {
  const p = mover([{ t: 0, x: 0, y: 0 }, { t: 10, x: 1000, y: 0 }]);
  const r = core.analyzeTravelLoad(p, { window: 2, distance: 150 }, 10);
  assert.equal(r.overloaded, true);
  assert.equal(r.ranges.length, 1);
  approx(r.ranges[0].start, 0);
  approx(r.ranges[0].end, 8);
  approx(r.firstStart, 0);
  approx(r.maxDistance, 200);
  approx(r.maxDistanceStart, 0);
});

test('奔走监控：路程恰等于上限不违规（严格超过）', () => {
  const p = mover([{ t: 0, x: 0, y: 0 }, { t: 10, x: 1000, y: 0 }]);
  const r = core.analyzeTravelLoad(p, { window: 2, distance: 200 }, 10);
  assert.equal(r.overloaded, false);
  assert.deepEqual(r.ranges, []);
  assert.equal(r.firstStart, null);
  approx(r.maxDistance, 200);
});

test('奔走监控：临界 D 给出精确起点边界（单段跨越后回落）', () => {
  // 0~5s 以 100/s 行进，之后静止。W=2，窗口路程
  //   s∈[0,3]：L(s)=100·(min(s+2,5)−s)，200→100；
  // D=150 的临界点：s∈[0,3] 时 s+2≤5 恒成立直到 s=3，故 L=200 常量？否——
  // s+2≤5 对 s≤3 成立，即整窗都在运动段内，L≡200。改用两段速度构造斜率。
  // 0~4s 100/s，4~8s 50/s，之后静止；W=4，D=300：
  //   s=0 时 L=100·4=400；s∈[0,4] 窗口跨两段，L(s)=100·(4−s)+50·s=400−50s；
  //   L=300 临界起点 s=2；s>4 时 L=50·4=200（s≤4）随后递减到 0。
  const p = mover([
    { t: 0, x: 0, y: 0 },
    { t: 4, x: 400, y: 0 },
    { t: 8, x: 600, y: 0 }
  ]);
  const r = core.analyzeTravelLoad(p, { window: 4, distance: 300 }, 10);
  assert.equal(r.overloaded, true);
  assert.equal(r.ranges.length, 1, '相接范围合并为一段');
  approx(r.ranges[0].start, 0, 1e-9);
  approx(r.ranges[0].end, 2, 1e-9, 'L=300 的临界起点为 s=2');
  approx(r.maxDistance, 400);
  approx(r.maxDistanceStart, 0);
});

test('奔走监控：首尾停驻不计路程（首停 2s、末停 2s，运动 100/s）', () => {
  // 关键帧 t=2 起、t=8 止，共 600 单位；W=2，D=150
  // 窗口起点域 [0,8]：s∈[0,?] 窗口与运动段 [2,8] 的重叠 = min(s+2,8)−max(s,2)
  //   s∈[0,2]：重叠 (s+2)−2 = s 秒 → L=100s，>150 ⇔ s>1.5
  //   s∈[2,6]：整窗在运动段 L=200
  //   s∈[6,8]：重叠 8−s 秒 → L=100(8−s)，>150 ⇔ s<6.5
  const p = mover([{ t: 2, x: 0, y: 0 }, { t: 8, x: 600, y: 0 }]);
  const r = core.analyzeTravelLoad(p, { window: 2, distance: 150 }, 10);
  assert.equal(r.ranges.length, 1, '三段相接合并');
  approx(r.ranges[0].start, 1.5, 1e-9);
  approx(r.ranges[0].end, 6.5, 1e-9);
  approx(r.maxDistance, 200);
  approx(r.maxDistanceStart, 2, 1e-9, '平台 [2,6] 取最早起点 2');
});

test('奔走监控：W=总时长时只检查起点 0（超限为零长度范围，恰等不违规）', () => {
  const p = mover([{ t: 0, x: 0, y: 0 }, { t: 10, x: 1000, y: 0 }]);
  const over = core.analyzeTravelLoad(p, { window: 10, distance: 900 }, 10);
  assert.equal(over.overloaded, true);
  assert.equal(over.ranges.length, 1);
  approx(over.ranges[0].start, 0);
  approx(over.ranges[0].end, 0, 1e-9, '零长度范围表示唯一合法起点 s=0');
  approx(over.firstStart, 0);
  approx(over.maxDistance, 1000);
  approx(over.maxDistanceStart, 0);

  const exact = core.analyzeTravelLoad(p, { window: 10, distance: 1000 }, 10);
  assert.equal(exact.overloaded, false);
  assert.deepEqual(exact.ranges, []);
});

test('奔走监控：折返折线按实际路程（非位移）累计', () => {
  // 0~5s 走到 (500,0)，5~10s 走回原点：总路程 1000，速度恒 100/s
  // W=4 任意窗口路程恒 400
  const p = mover([
    { t: 0, x: 0, y: 0 },
    { t: 5, x: 500, y: 0 },
    { t: 10, x: 0, y: 0 }
  ]);
  const r = core.analyzeTravelLoad(p, { window: 4, distance: 399 }, 10);
  assert.equal(r.ranges.length, 1);
  approx(r.ranges[0].start, 0);
  approx(r.ranges[0].end, 6);
  approx(r.maxDistance, 400);
  approx(r.maxDistanceStart, 0);
});

test('奔走监控：静止表演者 D=0 也不超限；运动者 D=0 除纯停驻窗口外全程超限', () => {
  const still = mover([{ t: 3, x: 100, y: 100 }]);
  const r0 = core.analyzeTravelLoad(still, { window: 2, distance: 0 }, 10);
  assert.equal(r0.overloaded, false);
  approx(r0.maxDistance, 0);
  approx(r0.maxDistanceStart, 0);

  // 仅在 t∈[2,8] 运动 100/s：与 150 阈值例同形，D=0 时 L>0 ⇔ 窗口与
  // 运动段有正重叠 ⇔ s∈(0,8)（端点 s=0、s=8 为零重叠，L=0 不违规）
  const p = mover([{ t: 2, x: 0, y: 0 }, { t: 8, x: 600, y: 0 }]);
  const r1 = core.analyzeTravelLoad(p, { window: 2, distance: 0 }, 10);
  assert.equal(r1.ranges.length, 1);
  approx(r1.ranges[0].start, 0, 1e-9, 's=0 时 L=0，但它是范围边界（紧邻内部超限）');
  approx(r1.ranges[0].end, 8, 1e-9);
});

test('奔走监控：多段不同速度的超限范围分段与合并', () => {
  // 0~2s 200/s（400）、2~6s 静止、6~10s 200/s（800）；W=3，D=300。
  // 窗口只在覆盖任一奔跑段的部分时超限；中段停驻隔开——两侧范围应分开
  const p = mover([
    { t: 0, x: 0, y: 0 },
    { t: 2, x: 400, y: 0 },
    { t: 6, x: 400, y: 0 },
    { t: 10, x: 1200, y: 0 }
  ]);
  const r = core.analyzeTravelLoad(p, { window: 3, distance: 300 }, 10);
  // 首段：s∈[0,2] 重叠 2−s 秒，L=200·(2−s)>300 ⇔ s<0.5
  // 末段 [6,10]：s∈[3,6] 重叠 s−3 秒（部分进入），s∈[6,7] 整窗在内 L=600，
  // s∈[7,...] 已超出起点域 S=7——故第二段 [4.5, 7]
  assert.equal(r.ranges.length, 2, '停驻隔开的两段不合并');
  approx(r.ranges[0].start, 0);
  approx(r.ranges[0].end, 0.5, 1e-9);
  approx(r.ranges[1].start, 4.5, 1e-9);
  approx(r.ranges[1].end, 7, 1e-9, '末段范围随起点域在 s=S=T−W=7 结束');
  approx(r.maxDistance, 600);
});

test('奔走监控：analyzeScene 装配结果，超限者按 firstStart 升序', () => {
  const scene = {
    duration: 10,
    travelLoad: { window: 2, distance: 150 },
    performers: [
      { id: 'late', name: '晚动', radius: 0, keys: [
        { t: 0, x: 0, y: 0 }, { t: 5, x: 0, y: 0 }, { t: 10, x: 0, y: 500 }
      ] }, // 5~10s 100/s：窗 [s,s+2] 与 [5,10] 重叠 (s−3) 秒（s∈[3,5]），
      // L>150 ⇔ s>4.5；整窗在内 s∈[5,8] L=200
      { id: 'early', name: '早动', radius: 0, keys: [
        { t: 0, x: 0, y: 0 }, { t: 10, x: 1000, y: 0 }
      ] } // 全程 100/s：[0,8] 全超限
    ]
  };
  const a = core.analyzeScene(scene);
  assert.deepEqual(a.travelLoad, { window: 2, distance: 150 });
  assert.equal(a.travelResults.length, 2);
  assert.equal(a.travelResults[0].performerId, 'early', 'firstStart=0 在前');
  assert.equal(a.travelResults[1].performerId, 'late');
  approx(a.travelResults[0].ranges[0].start, 0);
  approx(a.travelResults[0].ranges[0].end, 8);
  approx(a.travelResults[1].ranges[0].start, 4.5, 1e-9);
});

test('奔走监控：旧场景（缺省 travelLoad）完全兼容，travelLoad/travelResults 为 null', () => {
  const a = core.analyzeScene({
    duration: 10,
    performers: [
      { id: 'a', radius: 0, keys: [{ t: 0, x: 0, y: 0 }] },
      { id: 'b', radius: 0, keys: [{ t: 0, x: 100, y: 0 }] }
    ]
  });
  assert.equal(a.travelLoad, null);
  assert.equal(a.travelResults, null);
});

test('奔走监控：analyzeScene 对非法 travelLoad 抛错', () => {
  assert.throws(() => core.analyzeScene({
    duration: 10,
    travelLoad: { window: 11, distance: 1 },
    performers: [
      { id: 'a', radius: 0, keys: [{ t: 0, x: 0, y: 0 }] },
      { id: 'b', radius: 0, keys: [{ t: 0, x: 100, y: 0 }] }
    ]
  }), /总时长/);
});

test('奔走监控：内置样例数值（丙、丁超限；甲、乙不超限）', () => {
  const a = core.analyzeScene(core.builtinScene());
  assert.deepEqual(a.travelLoad, { window: 4, distance: 600 });
  const byName = {};
  a.travelResults.forEach((r) => { byName[r.performerName] = r; });
  assert.equal(byName['甲·横贯'].overloaded, false);
  assert.equal(byName['乙·纵穿'].overloaded, false);
  assert.equal(byName['丙·错峰展伞'].overloaded, true);
  assert.equal(byName['丁·跟进'].overloaded, true);
  // 丙两段各 √(440²+480²) ≈ 651.153
  approx(byName['丙·错峰展伞'].maxDistance, Math.hypot(440, 480), 1e-6);
  approx(byName['丙·错峰展伞'].maxDistanceStart, 0, 1e-9);
  approx(byName['丙·错峰展伞'].firstStart, 0, 1e-9);
  approx(byName['丙·错峰展伞'].ranges[0].end, 4.314229, 1e-4);
  // 丁首段 √(340²+450²)≈564.0、次段 √(440²+480²)≈651.2
  approx(byName['丁·跟进'].ranges[0].start, 1.652174, 1e-4);
  approx(byName['丁·跟进'].maxDistanceStart, 4, 1e-9);
});

// 可复现的简易伪随机
function makeRng(seed) {
  let s = seed >>> 0;
  return function () {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

test('奔走监控：40 组随机分段轨迹对稠密采样逐点一致（超限判定/最大值）', () => {
  const rng = makeRng(20260929);
  const duration = 12;
  for (let trial = 0; trial < 40; trial++) {
    const nKeys = 2 + Math.floor(rng() * 4);
    const times = [0];
    for (let i = 1; i < nKeys - 1; i++) times.push(rng() * duration);
    times.push(duration);
    times.sort((a, b) => a - b);
    const keys = times.map((t, i) => ({
      t: Math.round(t * 100) / 100,
      x: Math.round(rng() * 1000),
      y: Math.round(rng() * 600),
      // 偶发重合点（零速度段）与同位置停驻
      ...(i > 0 && rng() < 0.15 ? { x: 0, y: 0 } : {})
    }));
    keys[0] = { t: 0, x: Math.round(rng() * 1000), y: Math.round(rng() * 600) };
    keys[keys.length - 1] = {
      t: duration, x: Math.round(rng() * 1000), y: Math.round(rng() * 600)
    };

    const W = 0.5 + rng() * (duration - 0.5);
    const p = mover(keys);
    // 取多个 D：0、最大路程附近、中位路程、必然不超限的大值
    const samples = referenceL(keys, duration, W, 1600);
    const Ls = samples.map((q) => q.L);
    const maxL = Math.max.apply(null, Ls);
    const Ds = [0, maxL * 0.25, maxL * 0.5, maxL * 0.75, maxL, maxL + 50];

    Ds.forEach((D) => {
      const r = core.analyzeTravelLoad(p, { window: W, distance: D }, duration);

      // 逐点：采样点的超限判定必须与范围归属一致（容差内）；
      // 恰等于 D 的采样点允许在范围外（严格超过语义）
      samples.forEach((q) => {
        const inRange = core.isTravelOverAt(r, q.s);
        const trulyOver = q.L > D + 1e-7;
        const withinTol = Math.abs(q.L - D) <= (1e-5 + 1e-7 * Math.max(1, Math.abs(D)));
        if (trulyOver && !withinTol) {
          assert.ok(inRange, `trial ${trial} W=${W} D=${D} s=${q.s} L=${q.L} 应在超限范围内`);
        }
        if (q.L + 1e-7 < D) {
          assert.ok(!inRange, `trial ${trial} W=${W} D=${D} s=${q.s} L=${q.L} 不应超限`);
        }
      });

      // 最大值与稠密网格一致（网格只取离散起点，其最大值天然略低；
      // 容差取一个网格步长内的路程变化量级）
      const gridTol = (maxL / samples.length) * 4 + 1e-6;
      assert.ok(Math.abs(r.maxDistance - maxL) <= gridTol,
        `trial ${trial} maxDistance ${r.maxDistance} vs grid ${maxL}`);
      // 网格最大值不得超过解析最大值（解析结果是精确上界）
      assert.ok(r.maxDistance + 1e-9 >= maxL, '解析最大值应 ≥ 任何网格起点路程');
      // 最早最大起点：在该点直接计算路程须等于最大值，且稠密网格中没有
      // 更早的点达到最大值（平台取最早）
      const atPoint = core.buildSegments(core.normalizeKeys(keys, 0), duration, 0)
        .reduce((acc, seg) => {
          const a = Math.max(seg.t0, r.maxDistanceStart);
          const b = Math.min(seg.t1, r.maxDistanceStart + W);
          return b > a ? acc + Math.hypot(seg.vx, seg.vy) * (b - a) : acc;
        }, 0);
      assert.ok(Math.abs(atPoint - r.maxDistance) <= 1e-6, '最早最大起点处路程等于最大值');
      samples.filter((q) => q.s < r.maxDistanceStart - 1e-4).forEach((q) => {
        assert.ok(q.L < maxL - 1e-6,
          `trial ${trial} s=${q.s} 早于最早最大起点 ${r.maxDistanceStart} 却已达到最大值`);
      });
    });
  }
});
