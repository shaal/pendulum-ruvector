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
 * Station 5 — a competing population that shares discoveries through RuVector.
 */
export class Population {
    free(): void;
    [Symbol.dispose](): void;
    best_island(): number;
    /**
     * Evolve `count` islands (round-robin), running the migration each time the
     * sweep wraps. This is the heavy part — the caller throttles how often it runs
     * so a generation is spread over several frames and never blocks rendering.
     */
    evolve_islands(count: number): void;
    fitnesses(): Float64Array;
    generation(): number;
    n_islands(): number;
    constructor(sharing: boolean);
    /**
     * Flat positions for every arm, concatenated: island 0's [x0,y0,x1,y1,x2,y2],
     * then island 1's, … (3 points per 2-link arm).
     */
    positions_all(): Float64Array;
    restart(): void;
    rollouts(): number;
    set_sharing(on: boolean): void;
    sharing(): boolean;
    /**
     * Read-and-clear the "a migration just happened" pulse (for the flash).
     */
    take_migrated(): boolean;
    /**
     * Advance the live arms by `arm_steps` (the cheap part — runs every frame so
     * the display stays smooth). Each arm is driven by its island's champion.
     */
    tick_arms(arm_steps: number): void;
}

/**
 * Station 2 — RuVector recognizes a changed arm and recalls its gain.
 */
export class Recalibrator {
    free(): void;
    [Symbol.dispose](): void;
    adaptive_positions(): Float64Array;
    committed(): boolean;
    disturbed(): boolean;
    encounter(): number;
    /**
     * Wipe everything RuVector learned and re-seed the cold grid.
     */
    forget(): void;
    lag(): number;
    last_lag(): number;
    naive_positions(): Float64Array;
    constructor(new_l1: number);
    new_len(): number;
    /**
     * Throw the same disturbance again, keeping what RuVector has learned —
     * the lag should shrink on a repeat.
     */
    next_encounter(): void;
    /**
     * "nominal" | "probing" | "recognized"
     */
    phase(): string;
    recall_distance(): number;
    recalled_id(): string;
    recalled_l1(): number;
    recalled_learned(): boolean;
    set_learning(on: boolean): void;
    /**
     * Set the disturbance length (link-2's new length); applied on the next
     * encounter. If still pre-disturbance this encounter, it takes effect here.
     */
    set_new_len(l1: number): void;
    /**
     * Advance the scenario by `steps` control timesteps.
     */
    tick(steps: number): void;
    time(): number;
    tip_error_adaptive(): number;
    tip_error_naive(): number;
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
    readonly __wbg_population_free: (a: number, b: number) => void;
    readonly __wbg_recalibrator_free: (a: number, b: number) => void;
    readonly freeswing_energy: (a: number) => number;
    readonly freeswing_links: (a: number) => number;
    readonly freeswing_new: (a: number, b: number) => number;
    readonly freeswing_nudge: (a: number, b: number) => void;
    readonly freeswing_positions: (a: number) => [number, number];
    readonly freeswing_set_damping: (a: number, b: number) => void;
    readonly freeswing_step: (a: number, b: number) => void;
    readonly population_best_island: (a: number) => number;
    readonly population_evolve_islands: (a: number, b: number) => void;
    readonly population_fitnesses: (a: number) => [number, number];
    readonly population_generation: (a: number) => number;
    readonly population_n_islands: (a: number) => number;
    readonly population_new: (a: number) => number;
    readonly population_positions_all: (a: number) => [number, number];
    readonly population_restart: (a: number) => void;
    readonly population_rollouts: (a: number) => number;
    readonly population_set_sharing: (a: number, b: number) => void;
    readonly population_sharing: (a: number) => number;
    readonly population_take_migrated: (a: number) => number;
    readonly population_tick_arms: (a: number, b: number) => void;
    readonly recalibrator_adaptive_positions: (a: number) => [number, number];
    readonly recalibrator_committed: (a: number) => number;
    readonly recalibrator_disturbed: (a: number) => number;
    readonly recalibrator_encounter: (a: number) => number;
    readonly recalibrator_forget: (a: number) => void;
    readonly recalibrator_naive_positions: (a: number) => [number, number];
    readonly recalibrator_new: (a: number) => number;
    readonly recalibrator_next_encounter: (a: number) => void;
    readonly recalibrator_phase: (a: number) => [number, number];
    readonly recalibrator_recalled_id: (a: number) => [number, number];
    readonly recalibrator_recalled_learned: (a: number) => number;
    readonly recalibrator_set_learning: (a: number, b: number) => void;
    readonly recalibrator_set_new_len: (a: number, b: number) => void;
    readonly recalibrator_tick: (a: number, b: number) => void;
    readonly recalibrator_tip_error_adaptive: (a: number) => number;
    readonly recalibrator_tip_error_naive: (a: number) => number;
    readonly ruvector_smoke: () => [number, number];
    readonly start: () => void;
    readonly recalibrator_time: (a: number) => number;
    readonly recalibrator_lag: (a: number) => number;
    readonly recalibrator_last_lag: (a: number) => number;
    readonly recalibrator_new_len: (a: number) => number;
    readonly recalibrator_recall_distance: (a: number) => number;
    readonly recalibrator_recalled_l1: (a: number) => number;
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
