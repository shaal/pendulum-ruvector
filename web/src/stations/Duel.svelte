<script lang="ts">
  import { onMount, onDestroy } from 'svelte'
  import { Duel } from '../wasm/pendulum_web.js'
  import { drawDuel, RED, GREEN } from '../lib/render'

  const DT = 0.004
  let { active = false }: { active?: boolean } = $props()
  let canvas: HTMLCanvasElement
  let showDeep = $state(false)

  // input state (keyboard + touch both feed these)
  let leftHeld = $state(false)
  let rightHeld = $state(false)

  // HUD
  let youUp = $state(true)
  let autoUp = $state(true)
  let youBalanced = $state(0)
  let recogStatus = $state('')
  let recogActive = $state(false)
  let disturbed = $state(false)
  let windOn = $state(false)

  let sim: Duel | null = null
  let acc = 0
  let lastT = 0
  let raf = 0

  function frame(t: number) {
    raf = requestAnimationFrame(frame)
    if (!sim || !active) {
      lastT = 0
      return
    }
    const dt = lastT ? Math.min((t - lastT) / 1000, 0.05) : 0
    lastT = t
    const dir = (rightHeld ? 1 : 0) - (leftHeld ? 1 : 0)
    acc += dt
    const want = Math.floor(acc / DT)
    if (want > 0) {
      sim.step(Math.min(want, 40), dir)
      acc -= want * DT
      if (acc < 0) acc = 0
    }
    drawDuel(
      canvas,
      sim.you_positions(),
      sim.auto_positions(),
      3,
      RED,
      GREEN,
      'YOU — A / D',
      'RuVector — auto',
    )
    youUp = sim.you_up()
    autoUp = sim.auto_up()
    youBalanced = sim.you_balanced()
    recogStatus = sim.recog_status()
    recogActive = sim.recog_active()
    disturbed = sim.disturbed()
    windOn = sim.wind_on()
  }

  // keyboard (desktop) — only while this tab is active
  function onKeyDown(e: KeyboardEvent) {
    if (!active || !sim) return
    switch (e.key.toLowerCase()) {
      case 'a': leftHeld = true; break
      case 'd': rightHeld = true; break
      case ' ': e.preventDefault(); sim.disturb(); break
      case 'arrowleft': e.preventDefault(); sim.poke_auto(-1); break
      case 'arrowright': e.preventDefault(); sim.poke_auto(1); break
      case 'w': sim.toggle_wind(); break
      case 'm': sim.add_payload(); break
      case 'r': sim.reset(); break
    }
  }
  function onKeyUp(e: KeyboardEvent) {
    if (e.key.toLowerCase() === 'a') leftHeld = false
    if (e.key.toLowerCase() === 'd') rightHeld = false
  }

  onMount(() => {
    sim = new Duel()
    window.addEventListener('keydown', onKeyDown)
    window.addEventListener('keyup', onKeyUp)
    raf = requestAnimationFrame(frame)
  })
  onDestroy(() => {
    cancelAnimationFrame(raf)
    window.removeEventListener('keydown', onKeyDown)
    window.removeEventListener('keyup', onKeyUp)
    sim?.free()
  })
</script>

<section class="station" id="duel">
  <div class="stage">
    <canvas bind:this={canvas}></canvas>
    <div class="readout">you balanced {youBalanced.toFixed(1)}s · {disturbed ? 'disturbed' : 'steady'}</div>
    <!-- on-screen drive buttons (mobile + click); also reflect key state -->
    <div class="drive">
      <button
        class="big"
        class:on={leftHeld}
        aria-label="rotate left"
        onpointerdown={() => (leftHeld = true)}
        onpointerup={() => (leftHeld = false)}
        onpointerleave={() => (leftHeld = false)}
        onpointercancel={() => (leftHeld = false)}
      >◀ A</button>
      <button
        class="big"
        class:on={rightHeld}
        aria-label="rotate right"
        onpointerdown={() => (rightHeld = true)}
        onpointerup={() => (rightHeld = false)}
        onpointerleave={() => (rightHeld = false)}
        onpointercancel={() => (rightHeld = false)}
      >D ▶</button>
    </div>
  </div>

  <div class="panel">
    <span class="kicker">Station&nbsp;6 · the challenge</span>
    <h2>You vs RuVector</h2>
    <p class="label">You drive one arm with A / D. The other fixes itself. Try to keep up.</p>
    <p class="seeing">
      Hold <strong>A</strong> / <strong>D</strong> (or the on-screen buttons) to spin the
      <strong style="color:#b91c1c">red</strong> arm's base motor and keep it upright — it's brutal,
      because one motor has to control two linked joints. The
      <strong style="color:#15803d">green</strong> arm balances itself. Hit
      <strong>Fire disturbance</strong> to grow both arms' lower link: you'll flail, while the green
      arm briefly probes, <em>recognizes</em> its new shape via RuVector, and recovers.
    </p>

    <div class="status">
      <span class="chip" class:ok={youUp} class:warn={!youUp}>
        {youUp ? 'you: balancing' : 'you: DOWN — fight it up!'}
      </span>
      <span class="chip" class:ok={autoUp}>{autoUp ? 'RuVector: balancing' : 'RuVector: recovering…'}</span>
      {#if recogStatus}
        <span class="chip" class:warn={recogActive} class:ok={!recogActive}>{recogStatus}</span>
      {/if}
    </div>

    <div class="controls">
      <div class="group buttons">
        <button onclick={() => sim?.disturb()}>💥 Fire disturbance</button>
        <button class="ghost" onclick={() => sim?.reset()}>↺ reset (R)</button>
      </div>
      <div class="group">
        <span class="glabel">Bother RuVector</span>
        <button class="ghost" onclick={() => sim?.poke_auto(-1)}>◀ poke</button>
        <button class="ghost" onclick={() => sim?.poke_auto(1)}>poke ▶</button>
        <button class="ghost" class:sel={windOn} onclick={() => sim?.toggle_wind()}>🌬 wind</button>
        <button class="ghost" onclick={() => sim?.add_payload()}>＋ payload</button>
      </div>
    </div>

    <button class="deep-toggle" onclick={() => (showDeep = !showDeep)}>
      {showDeep ? '▾' : '▸'} How it really works
    </button>
    {#if showDeep}
      <div class="deep">
        <p>
          Both arms are <strong>underactuated</strong>: only joint 0 has a motor, and it has to
          control the passive joint 1 through the coupled dynamics — which is why balancing by hand
          is so hard. The green arm holds the unstable upright with an <strong>LQR</strong> gain and,
          when knocked far out, a collocated-PFL energy <strong>swing-up</strong> (the same
          <code>recover_torque</code> used across the project).
        </p>
        <p>
          On the length disturbance it runs the live <strong>RuVector recognition</strong> pipeline
          from the Recognize station — a dithered probe → closed-loop signature →
          <code>VectorDB::search</code> → adopt the recalled gain (falling back to a direct
          computation if it can't recognize within ~0.9&nbsp;s). The poke / wind / payload buttons
          throw disturbances of different kinds (velocity impulse, sustained force, changed inertia)
          so you can see what it shrugs off and what makes it work.
        </p>
        <p>
          It's the same <code>Pendulum</code> physics and the same in-tab RuVector store as every
          other station — only the controller and your keyboard differ.
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

  .drive {
    position: absolute; left: 0; right: 0; bottom: 12px;
    display: flex; justify-content: space-between; padding: 0 16px; pointer-events: none;
  }
  .drive .big {
    pointer-events: auto; touch-action: none; user-select: none;
    font-size: 1.3rem; font-weight: 800; padding: 14px 22px; border-radius: 16px;
    background: #fee2e2; color: #b91c1c; box-shadow: 0 4px 0 #fca5a5;
  }
  .drive .big.on { background: #b91c1c; color: #fff; box-shadow: 0 1px 0 #7f1d1d; transform: translateY(3px); }
</style>
