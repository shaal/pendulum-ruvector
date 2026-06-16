<script lang="ts">
  import { onMount } from 'svelte'
  import { FreeSwing } from '../wasm/pendulum_web.js'
  import { drawArm } from '../lib/render'

  const DT = 0.005
  let canvas: HTMLCanvasElement
  let links = $state(2)
  let damping = $state(0)
  let paused = $state(false)
  let energy = $state(0)
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

  onMount(() => {
    rebuild()
    requestAnimationFrame(frame)
  })
</script>

<section class="station" id="toy">
  <div class="stage">
    <canvas bind:this={canvas}></canvas>
    <div class="readout">energy: {energy.toFixed(2)} J</div>
  </div>

  <div class="panel">
    <span class="kicker">Station&nbsp;0</span>
    <h2>The toy</h2>
    <p class="label">A chain of sticks that swings. Tiny changes blow up fast — that's chaos.</p>
    <p class="seeing">
      Add a second or third stick and the motion turns wild and unpredictable. Press
      <em>tiny nudge</em> and the path changes completely from a kick you can barely see — that
      sensitivity is exactly why <em>remembering</em> past motion (what RuVector does in the next
      stations) is so useful.
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
          A planar chain of point masses on massless rods, absolute angles from straight-down.
          Motion obeys the manipulator equation
          <code>M(θ)·θ̈ = τ − C(θ,ω)·ω² − G(θ) − b·ω</code>: inertia matrix <code>M</code> with
          coupling <code>∝ cos(θᵢ−θⱼ)</code>, centrifugal/Coriolis <code>∝ sin(θᵢ−θⱼ)·ωⱼ²</code>,
          gravity, and viscous friction. Each step solves that linear system for the accelerations
          and integrates with fixed-step <strong>RK4</strong> — the same code, compiled to wasm.
        </p>
        <p>
          With no torque and no friction the passive system conserves energy (a unit test pins the
          drift below 0.1% over 20 s; it's also how we check the browser physics matches native).
          Two links is already chaotic: nearby states diverge exponentially, so a 0.6 rad/s kick to
          the tip rewrites the future.
        </p>
      </div>
    {/if}
  </div>
</section>
