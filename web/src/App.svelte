<script lang="ts">
  import { onMount } from 'svelte'
  import init, { ruvector_smoke } from './wasm/pendulum_web.js'
  import Toy from './stations/Toy.svelte'
  import Recognize from './stations/Recognize.svelte'
  import Recover from './stations/Recover.svelte'
  import Predict from './stations/Predict.svelte'
  import Discover from './stations/Discover.svelte'
  import Crowd from './stations/Crowd.svelte'
  import Duel from './stations/Duel.svelte'

  let ready = $state(false)
  let ruv = $state('starting…')
  let tab = $state('toy')

  onMount(async () => {
    await init()
    ruv = ruvector_smoke()
    ready = true
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

{#if ready}
  <nav class="tabs">
    <button class:sel={tab === 'toy'} onclick={() => (tab = 'toy')}>The toy</button>
    <button class:sel={tab === 'recognize'} onclick={() => (tab = 'recognize')}>Recognize</button>
    <button class:sel={tab === 'recover'} onclick={() => (tab = 'recover')}>Recover</button>
    <button class:sel={tab === 'predict'} onclick={() => (tab = 'predict')}>Predict</button>
    <button class:sel={tab === 'discover'} onclick={() => (tab = 'discover')}>Discover</button>
    <button class:sel={tab === 'compete'} onclick={() => (tab = 'compete')}>Compete</button>
    <button class:sel={tab === 'duel'} onclick={() => (tab = 'duel')}>You vs RuVector</button>
  </nav>
  <!-- All stations stay mounted (state persists), but only the active one runs
       its simulation loop (each guards on its `active` prop). -->
  <div class="pane" class:hidden={tab !== 'toy'}><Toy active={tab === 'toy'} /></div>
  <div class="pane" class:hidden={tab !== 'recognize'}><Recognize active={tab === 'recognize'} /></div>
  <div class="pane" class:hidden={tab !== 'recover'}><Recover active={tab === 'recover'} /></div>
  <div class="pane" class:hidden={tab !== 'predict'}><Predict active={tab === 'predict'} /></div>
  <div class="pane" class:hidden={tab !== 'discover'}><Discover active={tab === 'discover'} /></div>
  <div class="pane" class:hidden={tab !== 'compete'}><Crowd active={tab === 'compete'} /></div>
  <div class="pane" class:hidden={tab !== 'duel'}><Duel active={tab === 'duel'} /></div>
{:else}
  <div class="boot">loading the physics…</div>
{/if}

<footer>
  <p>
    Physics &amp; RuVector both run client-side in WebAssembly. The same Rust core as the native
    crate; energy-conservation tests keep the browser physics honest.
  </p>
</footer>

<style>
  header { text-align: center; margin-bottom: 20px; }
  h1 { font-size: clamp(2rem, 6vw, 3.2rem); margin: 0.2em 0 0.1em; letter-spacing: -0.02em; }
  h1 .x { color: var(--orange); }
  .tagline { max-width: 640px; margin: 0 auto 14px; color: var(--ink-soft); font-size: 1.05rem; line-height: 1.5; }
  .badge {
    display: inline-flex; gap: 8px; align-items: center; flex-wrap: wrap; justify-content: center;
    background: #fff; border-radius: 999px; padding: 8px 16px; box-shadow: var(--shadow);
    font-size: 0.9rem; color: var(--ink-soft);
  }
  .badge.on { color: var(--teal); box-shadow: inset 0 0 0 2px #99f6e4, var(--shadow); }
  .badge code { color: var(--teal); font-weight: 700; }

  nav.tabs {
    display: flex; flex-wrap: wrap; gap: 10px; align-items: center; justify-content: center;
    margin: 0 auto 22px; font-weight: 700;
  }
  nav.tabs button {
    color: var(--teal); background: #fff; border-radius: 999px;
    padding: 8px 18px; box-shadow: var(--shadow); font-weight: 800;
  }
  nav.tabs button:active { transform: translateY(1px); }
  nav.tabs button.sel { background: var(--teal); color: #fff; box-shadow: inset 0 0 0 2px var(--teal); }
  nav .soon { color: var(--ink-soft); font-weight: 600; font-size: 0.82rem; }

  .pane.hidden { display: none; }

  .boot { text-align: center; color: var(--ink-soft); padding: 60px 0; }
  footer { text-align: center; color: var(--ink-soft); font-size: 0.85rem; margin-top: 26px; line-height: 1.5; }
</style>
