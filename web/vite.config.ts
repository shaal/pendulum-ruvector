import { defineConfig } from 'vite'
import { svelte } from '@sveltejs/vite-plugin-svelte'

// `base: './'` keeps asset URLs relative so the site works at any path on
// Cloudflare Pages (and in `vite preview`). The wasm is loaded by wasm-pack's
// glue via `new URL(..., import.meta.url)`, which Vite rewrites to a hashed asset.
export default defineConfig({
  plugins: [svelte()],
  base: './',
})
