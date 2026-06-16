<script lang="ts">
  import { onMount } from 'svelte'
  import init, { ruvector_smoke } from './wasm/pendulum_web.js'
  import Toy from './stations/Toy.svelte'
  import Recognize from './stations/Recognize.svelte'

  let ready = $state(false)
  let ruv = $state('starting…')

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
  <nav>
    <a href="#toy">The toy</a>
    <a href="#recognize">Recognize</a>
    <span class="soon">recover · discover · compete · you vs RuVector — coming</span>
  </nav>
  <Toy />
  <Recognize />
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

  nav {
    display: flex; flex-wrap: wrap; gap: 10px; align-items: center; justify-content: center;
    margin: 0 auto 22px; font-weight: 700;
  }
  nav a {
    text-decoration: none; color: var(--teal); background: #fff; border-radius: 999px;
    padding: 6px 14px; box-shadow: var(--shadow);
  }
  nav .soon { color: var(--ink-soft); font-weight: 600; font-size: 0.82rem; }

  .boot { text-align: center; color: var(--ink-soft); padding: 60px 0; }
  footer { text-align: center; color: var(--ink-soft); font-size: 0.85rem; margin-top: 26px; line-height: 1.5; }
</style>
