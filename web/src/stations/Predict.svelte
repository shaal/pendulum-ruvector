<script lang="ts">
  import { onMount, onDestroy } from 'svelte'
  import { Predict } from '../wasm/pendulum_web.js'
  import { drawDuel, RED, GREEN } from '../lib/render'

  const DT = 0.005
  let { active = false }: { active?: boolean } = $props()
  let canvas: HTMLCanvasElement
  let showDeep = $state(false)

  let kinds = $state<string[]>([])
  let kind = $state(7)
  let pop = $state(16)
  let useMemory = $state(true)

  // live readouts
  let rRev = $state(0)
  let pRev = $state(0)
  let rOut = $state(0)
  let pOut = $state(0)
  let rCatch = $state(-1)
  let pCatch = $state(-1)
  let memCount = $state(0)

  let sim: Predict | null = null
  let acc = 0
  let lastT = 0
  let raf = 0

  function knock(i: number) {
    kind = i
    sim?.knock(i)
  }
  function setBudget(v: number) {
    pop = v
    sim?.set_budget(v)
  }
  function toggleMemory() {
    useMemory = !useMemory
    sim?.set_memory(useMemory)
  }

  function frame(t: number) {
    raf = requestAnimationFrame(frame)
    if (!sim || !active) {
      lastT = 0
      return
    }
    const dt = lastT ? Math.min((t - lastT) / 1000, 0.05) : 0
    lastT = t
    acc += dt
    const want = Math.floor(acc / DT)
    if (want > 0) {
      sim.step(Math.min(want, 30))
      acc -= want * DT
      if (acc < 0) acc = 0
    }
    drawDuel(
      canvas,
      sim.reactive_positions(),
      sim.predictive_positions(),
      2,
      RED,
      GREEN,
      'reactive — energy-shaping',
      'predictive — MPC',
    )
    rRev = sim.reactive_reversals()
    pRev = sim.predictive_reversals()
    rOut = sim.reactive_outcome()
    pOut = sim.predictive_outcome()
    rCatch = sim.reactive_catch()
    pCatch = sim.predictive_catch()
    memCount = sim.mem_count()
    kind = sim.kind()
  }

  onMount(() => {
    sim = new Predict()
    const n = sim.num_kinds()
    kinds = Array.from({ length: n }, (_, i) => sim!.name_at(i))
    pop = sim.pop()
    useMemory = sim.use_memory()
    kind = sim.kind()
    raf = requestAnimationFrame(frame)
  })
  onDestroy(() => {
    cancelAnimationFrame(raf)
    sim?.free()
  })

  const outcomeText = (o: number) =>
    o === 1 ? 'recovered ✅' : o === 2 ? "didn't catch ❌" : 'recovering…'
  const catchText = (c: number) => (c >= 0 ? `${c.toFixed(1)}s` : '—')
</script>

<section class="station" id="predict">
  <div class="stage">
    <canvas bind:this={canvas}></canvas>
    <div class="readout">{kinds[kind] ?? ''} · reactive {outcomeText(rOut)} · predictive {outcomeText(pOut)}</div>
  </div>

  <div class="panel">
    <span class="kicker">Station&nbsp;7</span>
    <h2>Reactive vs. predictive</h2>
    <p class="label">Same knockdown, two brains. One flails up; one plans up.</p>
    <p class="seeing">
      Both arms get the identical knockdown and the identical catch at the top. The
      <strong style="color:#b91c1c">red</strong> arm uses the evolved energy-shaping swing-up — it
      only sees the current state, so it pumps the motor back and forth. The
      <strong style="color:#15803d">green</strong> arm <em>predicts</em>: it rolls the real physics
      forward, plans a smooth pump, and commits. Watch the <strong>reversal counters</strong> —
      that's how many times each one slammed the motor in the opposite direction.
    </p>

    <div class="status">
      <span class="chip red">reactive: {rRev} reversals · {catchText(rCatch)}</span>
      <span class="chip green">predictive: {pRev} reversals · {catchText(pCatch)}</span>
      {#if rRev > 0}
        <span class="chip">predictive made {(rRev / Math.max(pRev, 1)).toFixed(0)}× fewer moves</span>
      {/if}
    </div>

    <div class="controls">
      <div class="group" style="gap:6px">
        <span class="glabel">Knock it down</span>
        {#each kinds as name, i}
          <button class="ghost small" class:sel={kind === i} onclick={() => knock(i)}>{name}</button>
        {/each}
      </div>

      <div class="group" style="margin-top:10px">
        <span class="glabel">Predictive planner budget: {pop}</span>
        <input
          type="range"
          min="4"
          max="48"
          step="4"
          value={pop}
          oninput={(e) => setBudget(+e.currentTarget.value)}
        />
      </div>

      <div class="group" style="margin-top:8px; align-items:center; gap:10px">
        <button class="ghost small" class:sel={useMemory} onclick={toggleMemory}>
          RuVector memory: {useMemory ? 'ON' : 'OFF'}
        </button>
        <span class="chip">remembered plans: {memCount}</span>
      </div>
    </div>

    <button class="deep-toggle" onclick={() => (showDeep = !showDeep)}>
      {showDeep ? '▾' : '▸'} How it really works
    </button>
    {#if showDeep}
      <div class="deep">
        <p>
          The green arm runs <strong>model-predictive control</strong>. Every few milliseconds it
          forks the live arm and rolls a small population of candidate torque plans forward through
          the <em>exact same RK4 dynamics</em> the simulator uses — no learned world model, because
          the simulator already <em>is</em> a perfect forward model. It scores each predicted
          trajectory (pump energy toward upright, arrive catchable) with the
          <strong>Cross-Entropy Method</strong>, commits the first move of the best plan, then
          re-plans. Because it can see where the arm is heading, it pumps once smoothly instead of
          reacting to every instant.
        </p>
        <p>
          Drop the <strong>planner budget</strong> and the green arm gets flaily — a tiny search
          can't find a good plan alone. Turn <strong>RuVector memory ON</strong> and each re-plan
          injects the nearest remembered plan as an extra candidate (strictly safe — it can only win
          or be ignored), so the same tiny budget plans well again. That's RuVector doing what it's
          actually good at: fast nearest-neighbour recall of what worked, buying back the compute the
          cheap planner gave up — which is exactly the trade a browser or embedded controller makes.
        </p>
      </div>
    {/if}
  </div>
</section>

<style>
  .status { display: flex; flex-wrap: wrap; gap: 8px; margin-bottom: 16px; }
  .chip { font-size: 0.8rem; font-weight: 700; padding: 4px 10px; border-radius: 999px; background: #f1f5f9; color: var(--ink-soft); }
  .chip.red { background: #fee2e2; color: #b91c1c; }
  .chip.green { background: #dcfce7; color: #15803d; }
  .ghost.small { padding: 6px 10px; font-size: 0.82rem; }
  .group { display: flex; flex-wrap: wrap; align-items: baseline; }
  .glabel { font-size: 0.8rem; font-weight: 700; color: var(--ink-soft); margin-right: 8px; }
  input[type='range'] { flex: 1; min-width: 160px; accent-color: var(--teal); }
</style>
