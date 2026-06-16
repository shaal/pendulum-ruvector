<script lang="ts">
  import { onMount } from 'svelte'
  import { Recalibrator } from '../wasm/pendulum_web.js'
  import { drawDuel, RED, GREEN } from '../lib/render'

  const DT = 0.005
  let { active = false }: { active?: boolean } = $props()
  let canvas: HTMLCanvasElement
  let newLen = $state(2.2)
  let learning = $state(true)
  let paused = $state(false)
  let showDeep = $state(false)

  // HUD (read from the wasm handle each frame)
  let phase = $state('nominal')
  let lag = $state(-1)
  let lastLag = $state(-1)
  let recalledL1 = $state(0)
  let learned = $state(false)
  let dist = $state(0)
  let encounter = $state(1)

  let sim: Recalibrator | null = null
  let acc = 0
  let lastT = 0

  function rebuild() {
    sim?.free()
    sim = new Recalibrator(newLen)
    sim.set_learning(learning)
    acc = 0
  }
  function onLen(e: Event) {
    newLen = parseFloat((e.target as HTMLInputElement).value)
    sim?.set_new_len(newLen)
  }
  function onLearning(e: Event) {
    learning = (e.target as HTMLInputElement).checked
    sim?.set_learning(learning)
  }

  function frame(t: number) {
    requestAnimationFrame(frame)
    if (!sim || !active) {
      lastT = 0
      return
    }
    const dt = lastT ? Math.min((t - lastT) / 1000, 0.05) : 0
    lastT = t
    if (!paused) {
      acc += dt
      const want = Math.floor(acc / DT)
      if (want > 0) {
        sim.tick(Math.min(want, 40))
        acc -= want * DT
        if (acc < 0) acc = 0
      }
    }
    drawDuel(
      canvas,
      sim.naive_positions(),
      sim.adaptive_positions(),
      Math.max(1 + newLen, 2),
      RED,
      GREEN,
      'naive — stale gain',
      'adaptive — RuVector',
    )
    phase = sim.phase()
    lag = sim.lag()
    lastLag = sim.last_lag()
    recalledL1 = sim.recalled_l1()
    learned = sim.recalled_learned()
    dist = sim.recall_distance()
    encounter = sim.encounter()
  }

  onMount(() => {
    rebuild()
    requestAnimationFrame(frame)
  })

  const phaseText = (p: string) =>
    p === 'nominal' ? 'both balancing' : p === 'probing' ? 'probing…' : 'recognized ✓'
  const shrink = $derived(lastLag > 0 && lag > 0 ? Math.round((100 * (lastLag - lag)) / lastLag) : 0)
</script>

<section class="station" id="recognize">
  <div class="stage">
    <canvas bind:this={canvas}></canvas>
    <div class="readout">encounter {encounter} · {phaseText(phase)}</div>
  </div>

  <div class="panel">
    <span class="kicker">Station&nbsp;2 · the RuVector showcase</span>
    <h2>Recognize the change</h2>
    <p class="label">The arm changes. One of them <em>remembers</em> what to do. That memory is RuVector.</p>
    <p class="seeing">
      Both arms balance straight up. After a second, the lower link suddenly grows — a tool
      extending. The <strong style="color:#b91c1c">red</strong> arm keeps its old reflexes and
      topples. The <strong style="color:#15803d">green</strong> arm wiggles for a moment to feel out
      how it moves now, looks up the closest arm it has met before, borrows that arm's know-how, and
      saves itself.
    </p>

    <div class="memory">
      <strong>Where's the memory?</strong> The wiggle becomes a short list of numbers — a
      "fingerprint" of how this arm moves. RuVector stores 30 such fingerprints and finds the nearest
      one in an instant. After a save, the arm's <em>own</em> fingerprint is added back — so the next
      time the same thing happens, it's recognized faster.
    </div>

    <div class="status">
      <span class="chip">probe→recall→adopt</span>
      {#if phase === 'recognized'}
        <span class="chip ok">recognized in {lag.toFixed(2)}s</span>
        <span class="chip">nearest: l1≈{recalledL1.toFixed(1)}m · {learned ? 'learned recall' : 'cold grid'} · dist {dist.toFixed(1)}</span>
      {:else if phase === 'probing'}
        <span class="chip warn">probing…</span>
      {/if}
      {#if lastLag > 0 && lag > 0}
        <span class="chip ok">lag {lastLag.toFixed(2)}s → {lag.toFixed(2)}s ({shrink}% faster)</span>
      {/if}
    </div>

    <div class="controls">
      <div class="group">
        <span class="glabel">New length (the change)</span>
        <input type="range" min="1.2" max="2.6" step="0.05" value={newLen} oninput={onLen} />
        <span class="val">{newLen.toFixed(2)} m</span>
      </div>
      <div class="group">
        <label class="toggle"><input type="checkbox" checked={learning} onchange={onLearning} /> self-learning (remember each save)</label>
      </div>
      <div class="group buttons">
        <button onclick={() => sim?.next_encounter()}>↻ throw it again</button>
        <button class="ghost" onclick={() => sim?.forget()}>🧽 forget everything</button>
        <button class="ghost" onclick={() => (paused = !paused)}>{paused ? '▶ play' : '⏸ pause'}</button>
      </div>
    </div>

    <button class="deep-toggle" onclick={() => (showDeep = !showDeep)}>
      {showDeep ? '▾' : '▸'} How it really works
    </button>
    {#if showDeep}
      <div class="deep">
        <p>
          Both arms are <strong>underactuated</strong> (only joint 0 has a motor) and hold the
          unstable upright with an <strong>LQR</strong> gain derived from the linearized dynamics.
          The gain depends on the arm's mass/length model, so when link 2 grows the old gain is wrong
          — the naive arm keeps it and falls.
        </p>
        <p>
          The adaptive arm runs a <strong>dithered probe</strong>: it keeps the stale stabilizer but
          injects a small multi-sine torque, and regresses the measured accelerations against state
          and dither. That yields the <strong>closed-loop signature</strong> — the
          <code>A − b·K</code> coefficients under a fixed probe gain (the dither makes the input
          column identifiable; a pure stabilizer alone is collinear with the state). The seeds were
          fingerprinted under the same probe gain, so the measured and stored signatures live in the
          same whitened space.
        </p>
        <p>
          The signature is the query vector into RuVector (<code>VectorDB::search</code>, k=1, over
          z-scored signatures). A <em>cold</em> grid match must converge tightly or hold for several
          checks before it's trusted; a <em>learned</em> match — this arm's own measured fingerprint,
          inserted after a previous catch — is trusted from a rougher, earlier estimate. That
          asymmetry is why the recognition lag shrinks on a repeat. Recognition keys on structural
          (length) change up to ~2.2 m; beyond that the arm topples faster than the probe can
          identify it (that's the swing-up station's job). See
          <code>estimator.rs</code> / <code>memory.rs</code> and the
          <code>phase2_recognition</code> tests.
        </p>
      </div>
    {/if}
  </div>
</section>

<style>
  .status { display: flex; flex-wrap: wrap; gap: 8px; margin-bottom: 16px; }
  .chip {
    font-size: 0.8rem; font-weight: 700; padding: 4px 10px; border-radius: 999px;
    background: #f1f5f9; color: var(--ink-soft);
  }
  .chip.ok { background: #dcfce7; color: #15803d; }
  .chip.warn { background: #fef3c7; color: #b45309; }
</style>
