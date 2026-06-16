/// <reference lib="webworker" />
// Runs the competing-population evolution on a background thread, so the heavy
// CEM rollouts never block the main thread's rendering. Publishes champion +
// fitness snapshots back to the page; the page drives the live arms from them.

import init, { Evolver } from '../wasm/pendulum_web.js'

let ev: Evolver | null = null
let ready = false
let want = false // whether the page wants evolution running (active tab, not paused)

function snapshot() {
  if (!ev) return
  postMessage({
    champions: ev.champions_flat(),
    fitnesses: ev.fitnesses(),
    generation: ev.generation(),
    rollouts: ev.rollouts(),
    bestIsland: ev.best_island(),
    nIslands: ev.n_islands(),
    migrated: ev.take_migrated(),
  })
}

function loop() {
  if (!ready || !want || !ev) return
  // Evolve in a ~40 ms budget, then yield ~16 ms so messages are processed and
  // we don't peg a core at 100%. This is a background thread, so even a long
  // burst here never stutters the page.
  const start = performance.now()
  while (performance.now() - start < 40) {
    ev.evolve_islands(1)
  }
  snapshot()
  setTimeout(loop, 16)
}

onmessage = async (e: MessageEvent) => {
  const m = e.data
  switch (m?.cmd) {
    case 'start':
      try {
        await init()
        ev = new Evolver(m.sharing ?? true)
        ready = true
        snapshot()
        if (want) loop()
      } catch (err) {
        postMessage({ error: 'worker init: ' + String(err) })
      }
      break
    case 'run':
      if (!want) {
        want = true
        if (ready) loop()
      }
      break
    case 'pause':
      want = false
      break
    case 'sharing':
      ev?.set_sharing(!!m.on)
      break
    case 'restart':
      ev?.restart()
      snapshot()
      break
  }
}
