# web — the in-browser Pendulum × RuVector exhibit

A single static page (Svelte + Vite) where the pendulum physics **and** RuVector's
vector database run entirely client-side in WebAssembly. No backend.

This is the **M0 spike**: one station (the free-swinging pendulum) that proves the
whole pipeline — Rust → WebAssembly → Canvas2D → Cloudflare Pages — and measures
the bundle. Result: **~83 KB gzipped total** (wasm ≈ 62 KB, js ≈ 19 KB, css ≈ 1.5 KB).
The remaining stations (balance, recognize, recover, discover, compete, the A/D
duel) are built on top of this same scaffold — see
[`../docs/plans/web-experience.md`](../docs/plans/web-experience.md).

## Develop

```bash
cd web
npm install
npm run dev          # http://localhost:5173
```

## Build

```bash
npm run build        # -> web/dist (what Cloudflare serves)
npm run preview      # serve the production build locally
```

## Regenerating the WebAssembly

The compiled WASM is **committed** under `src/wasm/` on purpose, so Cloudflare
Pages only runs the Vite build (no Rust toolchain in CI). Regenerate it whenever
the Rust changes (`pendulum_web/` or `pendulum_rs/`):

```bash
# needs Rust + wasm-pack (curl https://rustwasm.github.io/wasm-pack/installer/init.sh -sSf | sh)
npm run build:wasm   # wasm-pack build ../pendulum_web --target web --release --out-dir src/wasm
```

Then commit the updated `src/wasm/` files.

## Deploy on Cloudflare Pages (GitHub Action on merge to main)

Deploys are driven by [`.github/workflows/deploy.yml`](../.github/workflows/deploy.yml):
every push to `main` (i.e. when a PR is merged) builds `web/` and uploads
`web/dist` to the `pendulum-ruvector` Pages project via `wrangler pages deploy`.
Pure static hosting — no Workers, no Functions. Because the WASM is pre-built and
committed, CI runs only `npm ci` + Vite (no Rust toolchain, no submodule).

**One-time setup (needs you):**

1. **Add two GitHub Actions secrets** (repo → Settings → Secrets and variables → Actions):
   - `CLOUDFLARE_API_TOKEN` — create at Cloudflare → My Profile → API Tokens →
     Create Token, with the **"Cloudflare Pages: Edit"** permission.
   - `CLOUDFLARE_ACCOUNT_ID` — Cloudflare dash → Workers & Pages (right sidebar),
     or `wrangler whoami`.
2. **Custom domain `pendulum.shaal.dev`:** Cloudflare dash → Workers & Pages →
   `pendulum-ruvector` → Custom domains → Set up a custom domain →
   `pendulum.shaal.dev`. Since `shaal.dev` is already a Cloudflare zone, the CNAME
   + TLS cert are provisioned automatically. (The Pages project already exists at
   `pendulum-ruvector.pages.dev`.)

After the secrets exist, merging to `main` publishes; `workflow_dispatch` also lets
you run it manually from the Actions tab.

> If you later switch to multithreaded WASM (for faster evolution in the
> population station), add a `web/public/_headers` file setting
> `Cross-Origin-Opener-Policy: same-origin` and
> `Cross-Origin-Embedder-Policy: require-corp`. Not needed for M0.
