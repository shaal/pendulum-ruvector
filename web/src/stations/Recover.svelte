<script lang="ts">
  import { onMount, onDestroy } from 'svelte'
  import { Recover } from '../wasm/pendulum_web.js'
  import { drawRecover } from '../lib/render'

  const DT = 0.005
  let { active = false }: { active?: boolean } = $props()
  let canvas: HTMLCanvasElement
  let showDeep = $state(false)

  let kinds = $state<string[]>([])
  let kind = $state(0)
  let outcome = $state(0)
  let bestTip = $state(9.9)

  let sim: Recover | null = null
  let acc = 0
  let lastT = 0
  let raf = 0

  function knock(i: number) {
    kind = i
    sim?.knock(i)
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
    drawRecover(canvas, sim.positions(), sim.outcome())
    outcome = sim.outcome()
    bestTip = sim.best_tip()
    kind = sim.kind()
  }

  onMount(() => {
    sim = new Recover()
    const n = sim.num_kinds()
    kinds = Array.from({ length: n }, (_, i) => sim!.name_at(i))
    raf = requestAnimationFrame(frame)
  })
  onDestroy(() => {
    cancelAnimationFrame(raf)
    sim?.free()
  })

  const outcomeText = (o: number) =>
    o === 1 ? 'recovered ✅' : o === 2 ? "didn't catch ❌" : 'recovering…'
</script>

<section class="station" id="recover">
  <div class="stage">
    <canvas bind:this={canvas}></canvas>
    <div class="readout">{kinds[kind] ?? ''} · {outcomeText(outcome)}</div>
  </div>

  <div class="panel">
    <span class="kicker">Station&nbsp;3</span>
    <h2>Get back up</h2>
    <p class="label">Knock it down — watch it hoist itself back upright.</p>
    <p class="seeing">
      Pick a way to knock the arm over. It pumps energy to swing up, then catches itself at the top.
      Some starts it nails; a few are genuinely too hard and it misses — shown honestly (the same
      ones the project's own test harness fails). The dashed line is the upright goal.
    </p>

    <div class="status">
      <span class="chip" class:ok={outcome === 1} class:warn={outcome === 2}>{outcomeText(outcome)}</span>
      <span class="chip">closest approach: {bestTip.toFixed(2)} rad</span>
    </div>

    <div class="controls">
      <div class="group" style="gap:6px">
        <span class="glabel">Knock it down</span>
        {#each kinds as name, i}
          <button class="ghost small" class:sel={kind === i} onclick={() => knock(i)}>{name}</button>
        {/each}
      </div>
    </div>

    <button class="deep-toggle" onclick={() => (showDeep = !showDeep)}>
      {showDeep ? '▾' : '▸'} How it really works
    </button>
    {#if showDeep}
      <div class="deep">
        <p>
          The arm is underactuated (one motor, two joints), so it can't just push to upright from far
          away. It uses a <strong>collocated partial-feedback-linearization swing-up</strong>: the
          passive-joint equation lets it solve the elbow's acceleration from the motor's, so commanding
          the motor <em>feedback-linearizes</em> joint 0; an outer energy-pump law
          <code>v = k_e·(E_up − E)·ω₀</code> drives the total energy toward the upright value
          (near-bang-bang). When it gets close, the <strong>LQR</strong> catches and holds it.
        </p>
        <p>
          Honest result: this recovers about <strong>7 of 10</strong> diverse knockdowns — including a
          dead vertical hang — but worst cases (hard-sideways, both-folded) still defeat it; full
          swing-up of a double pendulum from <em>any</em> state is genuinely unsolved. These are the
          exact starts from <code>knockdown_starts()</code> / the <code>check</code> harness, same
          <code>recover_torque</code> controller as the native crate.
        </p>
      </div>
    {/if}
  </div>
</section>

<style>
  .status { display: flex; flex-wrap: wrap; gap: 8px; margin-bottom: 16px; }
  .chip { font-size: 0.8rem; font-weight: 700; padding: 4px 10px; border-radius: 999px; background: #f1f5f9; color: var(--ink-soft); }
  .chip.ok { background: #dcfce7; color: #15803d; }
  .chip.warn { background: #fef3c7; color: #b45309; }
  .ghost.small { padding: 6px 10px; font-size: 0.82rem; }
</style>
