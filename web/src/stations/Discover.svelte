<script lang="ts">
  import { onMount, onDestroy } from 'svelte'
  import { PopArms } from '../wasm/pendulum_web.js'
  import { drawDiscover } from '../lib/render'
  import PopWorker from '../lib/popWorker.ts?worker'

  const DT = 0.005
  let { active = false }: { active?: boolean } = $props()
  let canvas: HTMLCanvasElement
  let showDeep = $state(false)
  let gen = $state(0)
  let bestFit = $state(0)

  let arm: PopArms | null = null
  let worker: Worker | null = $state(null)
  let champions = new Float64Array(0)
  let fitness = -Infinity
  let history: number[] = []
  let lastGen = -1
  let acc = 0
  let lastT = 0
  let raf = 0
  let paused = $state(false)

  function frame(t: number) {
    raf = requestAnimationFrame(frame)
    if (!arm || !active) {
      lastT = 0
      return
    }
    const dt = lastT ? Math.min((t - lastT) / 1000, 0.05) : 0
    lastT = t
    if (!paused) {
      acc += dt
      const want = Math.floor(acc / DT)
      if (want > 0) {
        arm.tick(Math.min(want, 20), champions)
        acc -= want * DT
        if (acc < 0) acc = 0
      }
    }
    drawDiscover(canvas, arm.positions_all(), fitness, history)
  }

  $effect(() => {
    worker?.postMessage({ cmd: active && !paused ? 'run' : 'pause' })
  })

  onMount(() => {
    arm = new PopArms(1)
    const w = new PopWorker()
    w.onerror = (e) => console.error('discoverWorker:', e.message)
    w.onmessage = (e: MessageEvent) => {
      const d = e.data
      if (d.error) {
        console.error('discoverWorker:', d.error)
        return
      }
      champions = d.champions
      fitness = (d.fitnesses as Float64Array)[0]
      gen = d.generation
      if (gen !== lastGen) {
        lastGen = gen
        history.push(fitness)
        if (history.length > 400) history.shift()
        if (isFinite(fitness)) bestFit = Math.max(bestFit, fitness)
      }
    }
    // single island, no sharing — one searcher discovering a controller
    w.postMessage({ cmd: 'start', sharing: false, islands: 1 })
    worker = w
    raf = requestAnimationFrame(frame)
  })
  onDestroy(() => {
    cancelAnimationFrame(raf)
    worker?.terminate()
    arm?.free()
  })

  function restart() {
    history = []
    lastGen = -1
    bestFit = 0
    fitness = -Infinity
    worker?.postMessage({ cmd: 'restart' })
  }
</script>

<section class="station" id="discover">
  <div class="stage">
    <canvas bind:this={canvas}></canvas>
    <div class="readout">generation {gen} · best fit {isFinite(bestFit) ? bestFit.toFixed(0) : '…'}</div>
  </div>

  <div class="panel">
    <span class="kicker">Station&nbsp;4</span>
    <h2>Discover a trick</h2>
    <p class="label">Hundreds of attempts compete; the best way to swing up is found, not coded.</p>
    <p class="seeing">
      No one programmed this controller. A search tries many random control "recipes," keeps the ones
      that recover knockdowns best, and nudges its guesses toward them — over and over. Watch the
      <strong>fitness curve climb</strong> on the right as the arm (left) gets visibly better at
      hoisting itself upright, turning from <strong style="color:#b91c1c">red</strong> toward
      <strong style="color:#15803d">green</strong>.
    </p>

    <div class="controls">
      <div class="group buttons">
        <button class="ghost" onclick={() => (paused = !paused)}>{paused ? '▶ play' : '⏸ pause'}</button>
        <button class="ghost" onclick={restart}>↺ start over</button>
      </div>
    </div>

    <button class="deep-toggle" onclick={() => (showDeep = !showDeep)}>
      {showDeep ? '▾' : '▸'} How it really works
    </button>
    {#if showDeep}
      <div class="deep">
        <p>
          This is a gradient-free <strong>cross-entropy method</strong>. Each generation samples a
          population of candidate <code>EnergyShapingPolicy</code> parameter vectors from a Gaussian,
          scores each over several knockdown starts (<code>fitness</code> rewards catching, and
          getting closer when it misses), keeps the elite few, and re-fits the Gaussian toward them.
          No gradients, no GPU, no ML libraries — a few hundred lines of dependency-light Rust.
        </p>
        <p>
          The hand-tuned baseline recovers ~7/10; an evolved champion can reach 10/10 on the held-out
          harness, using feature combinations the hand-tuning never tried (passive-joint pump,
          posture-sin term, velocity damping). It's the leap from <em>adapting</em> (recalling a known
          controller) to <em>discovering</em> a new one. The <strong>Compete</strong> station then
          runs many of these searchers at once and shares their discoveries through RuVector.
        </p>
        <p>
          Like Compete, the search runs on a <strong>Web Worker</strong> so it never stutters the
          page; this thread just animates the current champion and draws the curve.
        </p>
      </div>
    {/if}
  </div>
</section>
