<script lang="ts">
  import { onMount } from 'svelte'
  import { Population } from '../wasm/pendulum_web.js'
  import { drawPopulation } from '../lib/render'

  const DT = 0.005
  let { active = false }: { active?: boolean } = $props()
  let canvas: HTMLCanvasElement
  let sharing = $state(true)
  let paused = $state(false)
  let showDeep = $state(false)

  let gen = $state(0)
  let rollouts = $state(0)
  let bestFit = $state(0)

  let sim: Population | null = null
  let flash = 0
  let acc = 0
  let lastT = 0
  let evo = 0

  function rebuild() {
    sim?.free()
    sim = new Population(sharing)
    acc = 0
    flash = 0
  }
  function toggleSharing() {
    sharing = !sharing
    sim?.set_sharing(sharing)
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
        sim.tick_arms(Math.min(want, 20)) // arms every frame (cheap, smooth)
        acc -= want * DT
        if (acc < 0) acc = 0
      }
      // evolution is heavy — one island every other frame, decoupled from rendering
      evo++
      if (evo % 2 === 0) sim.evolve_islands(1)
      if (sim.take_migrated()) flash = 0.6
      if (flash > 0) flash -= dt
    }
    const fits = Array.from(sim.fitnesses())
    drawPopulation(canvas, sim.positions_all(), sim.n_islands(), fits, sim.best_island(), flash)
    gen = sim.generation()
    rollouts = sim.rollouts()
    bestFit = fits.reduce((a, b) => (isFinite(b) && b > a ? b : a), -Infinity)
  }

  onMount(() => {
    rebuild()
    requestAnimationFrame(frame)
  })
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
        <button class="ghost" onclick={rebuild}>↺ restart</button>
      </div>
    </div>

    <button class="deep-toggle" onclick={() => (showDeep = !showDeep)}>
      {showDeep ? '▾' : '▸'} How it really works
    </button>
    {#if showDeep}
      <div class="deep">
        <p>
          Each island runs a gradient-free <strong>cross-entropy method</strong>: it samples a
          population of candidate swing-up policies (a linear energy-shaping controller,
          <code>EnergyShapingPolicy</code>), scores each over several knockdown starts, keeps the
          elite, and re-fits its sampling distribution toward them. The islands are deliberately
          weak (small populations) so the effect of sharing is visible.
        </p>
        <p>
          <strong>Sharing is RuVector-mediated migration.</strong> Every <code>migrate_every</code>
          generations each island inserts its champion (parameters as the vector, fitness in the
          payload) into a shared <code>SharedPolicyStore</code> (a RuVector
          <code>VectorDB</code>); it then reads the global best back and, if that beats its own,
          adopts it and re-widens to explore around it. Without sharing the islands are independent
          (same seed — sharing is the only difference). Native benchmarks show RuVector-mediated
          sharing reaches population-wide competence in up to ~80% fewer total rollouts; the test
          <code>ruvector_sharing_accelerates_the_population</code> pins it.
        </p>
        <p>
          In the browser the evolution is sliced one island per animation frame
          (<code>step_island</code> / <code>finish_generation</code>) so the live arms stay smooth
          while the population improves in the background — the single-threaded analogue of the
          native build's background evolution thread.
        </p>
      </div>
    {/if}
  </div>
</section>
