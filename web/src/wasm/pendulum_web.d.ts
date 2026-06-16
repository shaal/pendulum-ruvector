/* tslint:disable */
/* eslint-disable */

/**
 * Station 0 — a free-swinging n-link pendulum. Released from a sprawl and left
 * passive (no applied torque), it swings chaotically: the warm-up that motivates
 * why remembering past dynamics (RuVector) is worth anything.
 */
export class FreeSwing {
    free(): void;
    [Symbol.dispose](): void;
    /**
     * Total mechanical energy — used to show that the passive system conserves it
     * (and to compare native vs wasm: it should match the native reference).
     */
    energy(): number;
    /**
     * Number of links.
     */
    links(): number;
    /**
     * `links` ∈ [1, 6]; `damping` is per-joint viscous friction (0 = frictionless).
     */
    constructor(links: number, damping: number);
    /**
     * A tiny kick to the tip joint — the "chaos" button. Two identical arms given
     * this nudge diverge within seconds.
     */
    nudge(delta: number): void;
    /**
     * Flat `[x0, y0, x1, y1, …]` joint positions including the anchor (n+1 points),
     * in physics units. The Canvas2D renderer scales these to pixels. Returned as
     * a `Float64Array` to JS.
     */
    positions(): Float64Array;
    /**
     * Live-tune per-joint damping from a slider.
     */
    set_damping(d: number): void;
    /**
     * Advance the physics by `steps` fixed timesteps (passive — zero torque).
     */
    step(steps: number): void;
}

/**
 * Proof that RuVector's in-memory vector DB runs in the browser. Creates a tiny
 * in-memory store, inserts two vectors, and returns the id of the nearest match
 * to a query — entirely client-side, no server. Also keeps `ruvector-core` linked
 * into the wasm bundle so M0's size measurement reflects the real page.
 */
export function ruvector_smoke(): string;

export function start(): void;

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
    readonly memory: WebAssembly.Memory;
    readonly __wbg_freeswing_free: (a: number, b: number) => void;
    readonly freeswing_energy: (a: number) => number;
    readonly freeswing_links: (a: number) => number;
    readonly freeswing_new: (a: number, b: number) => number;
    readonly freeswing_nudge: (a: number, b: number) => void;
    readonly freeswing_positions: (a: number) => [number, number];
    readonly freeswing_set_damping: (a: number, b: number) => void;
    readonly freeswing_step: (a: number, b: number) => void;
    readonly ruvector_smoke: () => [number, number];
    readonly start: () => void;
    readonly __wbindgen_free: (a: number, b: number, c: number) => void;
    readonly __wbindgen_malloc: (a: number, b: number) => number;
    readonly __wbindgen_realloc: (a: number, b: number, c: number, d: number) => number;
    readonly __wbindgen_externrefs: WebAssembly.Table;
    readonly __wbindgen_start: () => void;
}

export type SyncInitInput = BufferSource | WebAssembly.Module;

/**
 * Instantiates the given `module`, which can either be bytes or
 * a precompiled `WebAssembly.Module`.
 *
 * @param {{ module: SyncInitInput }} module - Passing `SyncInitInput` directly is deprecated.
 *
 * @returns {InitOutput}
 */
export function initSync(module: { module: SyncInitInput } | SyncInitInput): InitOutput;

/**
 * If `module_or_path` is {RequestInfo} or {URL}, makes a request and
 * for everything else, calls `WebAssembly.instantiate` directly.
 *
 * @param {{ module_or_path: InitInput | Promise<InitInput> }} module_or_path - Passing `InitInput` directly is deprecated.
 *
 * @returns {Promise<InitOutput>}
 */
export default function __wbg_init (module_or_path?: { module_or_path: InitInput | Promise<InitInput> } | InitInput | Promise<InitInput>): Promise<InitOutput>;
