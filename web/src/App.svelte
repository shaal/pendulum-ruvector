<script lang="ts">
  import { onMount } from 'svelte'
  import init, { FreeSwing, ruvector_smoke } from './wasm/pendulum_web.js'
  import { drawArm } from './lib/render'

  const DT = 0.005

  let canvas: HTMLCanvasElement
  let ready = $state(false)
  let links = $state(2)
  let damping = $state(0)
  let paused = $state(false)
  let energy = $state(0)
  let ruv = $state('starting…')
  let showDeep = $state(false)

  let sim: FreeSwing | null = null
  let trail: [number, number][] = []
  let acc = 0
  let lastT = 0

  function rebuild() {
    sim?.free()
    sim = new FreeSwing(links, damping)
    trail = []
    acc = 0
  }

  function setLinks(n: number) {
    links = n
    rebuild()
  }

  function onDamping(e: Event) {
    damping = parseFloat((e.target as HTMLInputElement).value)
    sim?.set_damping(damping)
  }

  function frame(t: number) {
    requestAnimationFrame(frame)
    if (!sim) return
    const dt = lastT ? Math.min((t - lastT) / 1000, 0.05) : 0
    lastT = t
    if (!paused) {
      acc += dt
      const steps = Math.floor(acc / DT)
      if (steps > 0) {
        sim.step(steps)
        acc -= steps * DT
      }
    }
    const pos = sim.positions()
    const n = pos.length / 2
    trail.push([pos[2 * (n - 1)], pos[2 * (n - 1) + 1]])
    if (trail.length > 80) trail.shift()
    energy = sim.energy()
    drawArm(canvas, pos, links, trail)
  }

  onMount(async () => {
    await init()
    ruv = ruvector_smoke()
    rebuild()
    ready = true
    requestAnimationFrame(frame)
  })
</script>

<header>
  <h1>Pendulum <span class="x">×</span> RuVector</h1>
  <p class="tagline">
    A chaotic pendulum that learns to balance itself — the physics, the learning, and a
    vector-database "brain" all run <strong>inside your browser</strong>. No server.
  </p>
  <div class="badge" class:on={ruv.startsWith('nearest')}>
    🧠 RuVector, running in this tab: <code>{ruv}</code>
  </div>
</header>

<section class="station">
  <div class="stage">
    <canvas bind:this={canvas}></canvas>
    {#if !ready}<div class="loading">loading the physics…</div>{/if}
    <div class="readout">energy: {energy.toFixed(2)} J</div>
  </div>

  <div class="panel">
    <span class="kicker">Station&nbsp;0</span>
    <h2>The toy</h2>
    <p class="label">A chain of sticks that swings. Tiny changes blow up fast — that's chaos.</p>

    <p class="seeing">
      Drag the controls and watch it swing. Add a second or third stick and the motion turns
      wild and unpredictable. Hit <em>tiny nudge</em> and the path changes completely from a
      change you can barely see — that sensitivity is exactly why <em>remembering</em> past
      motion (what RuVector does in the later stations) is so valuable.
    </p>

    <div class="controls">
      <div class="group">
        <span class="glabel">Sticks</span>
        {#each [1, 2, 3, 4] as n}
          <button class="ghost" class:sel={links === n} onclick={() => setLinks(n)}>{n}</button>
        {/each}
      </div>

      <div class="group">
        <span class="glabel">Stickiness (friction)</span>
        <input type="range" min="0" max="0.5" step="0.01" value={damping} oninput={onDamping} />
        <span class="val">{damping.toFixed(2)}</span>
      </div>

      <div class="group buttons">
        <button onclick={() => sim?.nudge(0.6)}>✨ tiny nudge</button>
        <button class="ghost" onclick={() => (paused = !paused)}>{paused ? '▶ play' : '⏸ pause'}</button>
        <button class="ghost" onclick={rebuild}>↺ reset</button>
      </div>
    </div>

    <button class="deep-toggle" onclick={() => (showDeep = !showDeep)}>
      {showDeep ? '▾' : '▸'} How it really works
    </button>
    {#if showDeep}
      <div class="deep">
        <p>
          This is a planar chain of point masses on massless rods, with absolute joint angles
          measured from straight-down. Its motion obeys the <strong>manipulator equation</strong>
          <code>M(θ)·θ̈ = τ − C(θ,ω)·ω² − G(θ) − b·ω</code>: an inertia matrix
          <code>M</code> (whose coupling terms go as <code>cos(θᵢ−θⱼ)</code> weighted by the mass
          carried below each joint), the centrifugal/Coriolis term <code>C·ω²</code> (going as
          <code>sin(θᵢ−θⱼ)</code>), gravity <code>G</code>, and viscous joint friction <code>b·ω</code>.
        </p>
        <p>
          Each frame solves that linear system for the angular accelerations (dense Gaussian
          elimination, no linear-algebra dependency) and integrates with fixed-step
          <strong>RK4</strong> — fourth-order, the accurate choice for a chaotic system. With no
          torque and no friction the passive system <strong>conserves total mechanical energy</strong>;
          a unit test pins the drift below 0.1% over 20 s, and the same check is how we verify the
          browser physics matches the native build bit-for-bit.
        </p>
        <p>
          Two links is already a classic chaotic system: nearby starting states diverge
          exponentially (positive Lyapunov exponent), which is why the <em>tiny nudge</em> button
          sends it somewhere new. This exact <code>Pendulum</code> type is what every later
          station is built on — balancing it, recovering it, and recognising it with RuVector.
        </p>
      </div>
    {/if}
  </div>
</section>

<footer>
  <p>
    M0 spike — one station, proving the pipeline: Rust → WebAssembly → Canvas2D → Cloudflare Pages.
    Physics &amp; RuVector both run client-side. WASM bundle ≈ 59&nbsp;KB gzipped.
  </p>
</footer>

<style>
  header { text-align: center; margin-bottom: 28px; }
  h1 { font-size: clamp(2rem, 6vw, 3.2rem); margin: 0.2em 0 0.1em; letter-spacing: -0.02em; }
  h1 .x { color: var(--orange); }
  .tagline { max-width: 640px; margin: 0 auto 14px; color: var(--ink-soft); font-size: 1.05rem; line-height: 1.5; }
  .badge {
    display: inline-flex; gap: 8px; align-items: center;
    background: #fff; border-radius: 999px; padding: 8px 16px;
    box-shadow: var(--shadow); font-size: 0.9rem; color: var(--ink-soft);
  }
  .badge.on { color: var(--teal); box-shadow: inset 0 0 0 2px #99f6e4, var(--shadow); }
  .badge code { color: var(--teal); font-weight: 700; }

  .station {
    display: grid; grid-template-columns: 1.1fr 1fr; gap: 22px;
    background: var(--card); border-radius: var(--radius); padding: 18px;
    box-shadow: var(--shadow);
  }
  @media (max-width: 820px) { .station { grid-template-columns: 1fr; } }

  .stage {
    position: relative; background: linear-gradient(180deg, #fffdfa, #fff3e6);
    border-radius: 16px; overflow: hidden; min-height: 380px;
  }
  canvas { width: 100%; height: 100%; display: block; min-height: 380px; touch-action: none; }
  .loading { position: absolute; inset: 0; display: grid; place-items: center; color: var(--ink-soft); }
  .readout {
    position: absolute; right: 12px; bottom: 10px; font-variant-numeric: tabular-nums;
    background: #ffffffcc; border-radius: 999px; padding: 4px 12px; font-size: 0.82rem; color: var(--ink-soft);
  }

  .panel { padding: 6px 8px; }
  .kicker { font-size: 0.72rem; font-weight: 800; letter-spacing: 0.12em; text-transform: uppercase; color: var(--orange); }
  h2 { margin: 4px 0 6px; font-size: 1.7rem; }
  .label { font-size: 1.12rem; font-weight: 700; margin: 0 0 10px; }
  .seeing { color: var(--ink-soft); line-height: 1.55; margin: 0 0 16px; }

  .controls { display: flex; flex-direction: column; gap: 14px; margin-bottom: 16px; }
  .group { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
  .group.buttons { gap: 10px; }
  .glabel { font-weight: 700; font-size: 0.9rem; min-width: 100%; }
  @media (min-width: 480px) { .glabel { min-width: 150px; } }
  .group button.sel { background: var(--teal); color: #fff; box-shadow: inset 0 0 0 2px var(--teal); }
  input[type='range'] { accent-color: var(--orange); flex: 1; min-width: 140px; }
  .val { font-variant-numeric: tabular-nums; color: var(--ink-soft); min-width: 34px; }

  .deep-toggle {
    background: none; color: var(--teal); box-shadow: none; padding: 6px 0; font-weight: 800;
  }
  .deep-toggle:active { transform: none; }
  .deep { border-left: 3px solid #99f6e4; padding: 4px 0 4px 14px; margin-top: 6px; color: var(--ink); }
  .deep p { line-height: 1.6; margin: 0 0 12px; font-size: 0.95rem; }
  code { background: #f1f5f9; border-radius: 6px; padding: 1px 6px; font-size: 0.88em; }

  footer { text-align: center; color: var(--ink-soft); font-size: 0.85rem; margin-top: 26px; line-height: 1.5; }
</style>
