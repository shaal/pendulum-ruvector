<script lang="ts">
  import { onMount, onDestroy } from 'svelte'
  import { PopArms } from '../wasm/pendulum_web.js'
  import { drawPopulation } from '../lib/render'
  import PopWorker from '../lib/popWorker.ts?worker'

  const DT = 0.005
  let { active = false }: { active?: boolean } = $props()
  let canvas: HTMLCanvasElement
  let sharing = $state(true)
  let paused = $state(false)
  let showDeep = $state(false)
  let gen = $state(0)
  let rollouts = $state(0)

  let arms: PopArms | null = null
  // `worker` is reactive so the run/pause effect fires once it's created (avoids a
  // startup race where the effect runs before onMount and never sees the worker).
  let worker: Worker | null = $state(null)
  // Latest snapshot from the worker (champions drive the arms; the rest is HUD).
  let champions = new Float64Array(0)
  let fitnesses: number[] = []
  let bestIsland = 0
  let nIslands = 8
  let flash = 0
  let acc = 0
  let lastT = 0
  let raf = 0

  function frame(t: number) {
    raf = requestAnimationFrame(frame)
    if (!arms || !active) {
      lastT = 0
      return
    }
    const dt = lastT ? Math.min((t - lastT) / 1000, 0.05) : 0
    lastT = t
    if (!paused) {
      acc += dt
      const want = Math.floor(acc / DT)
      if (want > 0) {
        arms.tick(Math.min(want, 20), champions)
        acc -= want * DT
        if (acc < 0) acc = 0
      }
      if (flash > 0) flash -= dt
    }
    drawPopulation(canvas, arms.positions_all(), nIslands, fitnesses, bestIsland, flash)
  }

  // Tell the worker to run only when this tab is active and not paused (so it
  // doesn't peg a core in the background on other tabs).
  $effect(() => {
    worker?.postMessage({ cmd: active && !paused ? 'run' : 'pause' })
  })

  onMount(() => {
    arms = new PopArms(8)
    nIslands = arms.n_islands()
    const w = new PopWorker()
    w.onerror = (e) => console.error('popWorker onerror:', e.message, e.filename, e.lineno)
    w.onmessage = (e: MessageEvent) => {
      const d = e.data
      if (d.error) {
        console.error('popWorker:', d.error)
        return
      }
      champions = d.champions
      fitnesses = Array.from(d.fitnesses as Float64Array)
      bestIsland = d.bestIsland
      nIslands = d.nIslands
      gen = d.generation
      rollouts = d.rollouts
      if (d.migrated) flash = 0.6
    }
    w.postMessage({ cmd: 'start', sharing, islands: 8 })
    worker = w // assigning the reactive var fires the run/pause effect
    raf = requestAnimationFrame(frame)
  })

  onDestroy(() => {
    cancelAnimationFrame(raf)
    worker?.terminate()
    arms?.free()
  })

  function toggleSharing() {
    sharing = !sharing
    worker?.postMessage({ cmd: 'sharing', on: sharing })
  }
  function restart() {
    worker?.postMessage({ cmd: 'restart' })
  }
</script>

<section class="station" id="compete">
  <div class="stage">
    <canvas bind:this={canvas}></canvas>
    <div class="readout">gen {gen} · {rollouts.toLocaleString()} practice tries · sharing {sharing ? 'ON' : 'off'}</div>
  </div>

  <div class="panel">
    <span class="kicker">Station&nbsp;5 · the RuVector showcase</span>
    <h2>A crowd that shares</h2>
    <p class="label">Many learners racing — and when one finds something good, RuVector hands it to the rest.</p>
    <p class="seeing">
      Each box is a separate little learner ("island") trying to discover how to swing the arm up
      and balance it. They start clumsy and turn from <strong style="color:#b91c1c">red</strong>
      (weak) toward <strong style="color:#15803d">green</strong> (strong) as they improve. The
      <strong>gold box</strong> is the best one right now.
    </p>

    <div class="memory">
      <strong>Where's the sharing?</strong> Every few rounds, each island posts its best find into
      RuVector and copies the overall best back out. Flip <em>sharing off</em> and the islands
      struggle alone; flip it <em>on</em> and a stuck island can suddenly jump ahead the moment a
      good idea is passed around (watch for the green flash). Same starting point — sharing is the
      only difference, and it reaches a strong crowd in far fewer total tries.
    </div>

    <div class="controls">
      <div class="group">
        <label class="toggle"><input type="checkbox" checked={sharing} onchange={toggleSharing} /> share discoveries through RuVector</label>
      </div>
      <div class="group buttons">
        <button class="ghost" onclick={() => (paused = !paused)}>{paused ? '▶ play' : '⏸ pause'}</button>
        <button class="ghost" onclick={restart}>↺ restart</button>
      </div>
    </div>

    <button class="deep-toggle" onclick={() => (showDeep = !showDeep)}>
      {showDeep ? '▾' : '▸'} How it really works
    </button>
    {#if showDeep}
      <div class="deep">
        <p>
          Each island runs a gradient-free <strong>cross-entropy method</strong>: it samples a
          population of candidate swing-up policies (<code>EnergyShapingPolicy</code>), scores each
          over several knockdown starts, keeps the elite, and re-fits its sampling distribution
          toward them. The islands are deliberately weak so the effect of sharing is visible.
        </p>
        <p>
          <strong>Sharing is RuVector-mediated migration.</strong> Every <code>migrate_every</code>
          generations each island inserts its champion (parameters as the vector, fitness in the
          payload) into a shared <code>SharedPolicyStore</code> (a RuVector <code>VectorDB</code>);
          it reads the global best back and, if it beats its own, adopts it and re-widens. Native
          benchmarks show this reaches population-wide competence in up to ~80% fewer total
          rollouts (test <code>ruvector_sharing_accelerates_the_population</code>).
        </p>
        <p>
          <strong>Why it's now smooth.</strong> The evolution runs on a <strong>Web Worker</strong>
          (a real background CPU thread): it evolves continuously and publishes champion snapshots,
          while this thread only steps the live arms from those champions and draws them. So the
          render stays at 60&nbsp;fps no matter how heavy the search gets — the browser analogue of
          the native build's background evolution thread.
        </p>
      </div>
    {/if}
  </div>
</section>
