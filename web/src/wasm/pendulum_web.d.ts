/* tslint:disable */
/* eslint-disable */

/**
 * Station 6 — You vs RuVector.
 */
export class Duel {
    free(): void;
    [Symbol.dispose](): void;
    add_payload(): void;
    auto_positions(): Float64Array;
    auto_up(): boolean;
    /**
     * Fire the length disturbance (both arms' second link extends). The auto arm
     * starts a live RuVector recognition probe.
     */
    disturb(): void;
    disturbed(): boolean;
    constructor();
    poke_auto(dir: number): void;
    recog_active(): boolean;
    recog_status(): string;
    reset(): void;
    /**
     * Advance `steps` timesteps. `human_dir` ∈ {-1, 0, 1} (A / nothing / D).
     */
    step(steps: number, human_dir: number): void;
    time(): number;
    toggle_wind(): void;
    wind_on(): boolean;
    you_balanced(): number;
    you_positions(): Float64Array;
    you_up(): boolean;
}

/**
 * Worker-side: the evolving population (no display arms).
 */
export class Evolver {
    free(): void;
    [Symbol.dispose](): void;
    best_island(): number;
    /**
     * Flat champion parameters for every island (`n_islands * NP`) — what the main
     * thread needs to drive its display arms.
     */
    champions_flat(): Float64Array;
    /**
     * Evolve `count` islands (round-robin); migrate when the sweep wraps.
     */
    evolve_islands(count: number): void;
    fitnesses(): Float64Array;
    generation(): number;
    n_islands(): number;
    /**
     * `islands` = how many competing CEM searchers (8 for Compete; 1 for the
     * single-searcher Discover station).
     */
    constructor(sharing: boolean, islands: number);
    restart(): void;
    rollouts(): number;
    set_sharing(on: boolean): void;
    sharing(): boolean;
    take_migrated(): boolean;
}

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
 * Main-thread: the live display arms, driven by champion parameters from the
 * worker. Cheap enough to step every frame.
 */
export class PopArms {
    free(): void;
    [Symbol.dispose](): void;
    n_islands(): number;
    /**
     * `n` display arms (8 for Compete; 1 for the single-searcher Discover).
     */
    constructor(n: number);
    /**
     * Flat positions for every arm: island 0's `[x0,y0,x1,y1,x2,y2]`, then 1's, …
     */
    positions_all(): Float64Array;
    /**
     * Step every arm `steps` times, driven by its island's champion. `champions`
     * is the flat `n_islands * NP` array from `Evolver::champions_flat`.
     */
    tick(steps: number, champions: Float64Array): void;
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
 * Station 3 — recover from a knockdown.
 */
export class Recover {
    free(): void;
    [Symbol.dispose](): void;
    best_tip(): number;
    current_name(): string;
    kind(): number;
    /**
     * Reset to knockdown start `i` and begin a fresh recovery attempt.
     */
    knock(i: number): void;
    name_at(i: number): string;
    constructor();
    num_kinds(): number;
    /**
     * 0 = recovering · 1 = recovered · 2 = didn't catch
     */
    outcome(): number;
    positions(): Float64Array;
    step(steps: number): void;
    tip(): number;
}

/**
 * Number of evolvable policy parameters per island champion (the stride of
 * `Evolver::champions_flat`).
 */
export function np(): number;

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
    readonly __wbg_duel_free: (a: number, b: number) => void;
    readonly __wbg_evolver_free: (a: number, b: number) => void;
    readonly __wbg_freeswing_free: (a: number, b: number) => void;
    readonly __wbg_poparms_free: (a: number, b: number) => void;
    readonly __wbg_recalibrator_free: (a: number, b: number) => void;
    readonly __wbg_recover_free: (a: number, b: number) => void;
    readonly duel_add_payload: (a: number) => void;
    readonly duel_auto_positions: (a: number) => [number, number];
    readonly duel_auto_up: (a: number) => number;
    readonly duel_disturb: (a: number) => void;
    readonly duel_disturbed: (a: number) => number;
    readonly duel_new: () => number;
    readonly duel_poke_auto: (a: number, b: number) => void;
    readonly duel_recog_active: (a: number) => number;
    readonly duel_recog_status: (a: number) => [number, number];
    readonly duel_reset: (a: number) => void;
    readonly duel_step: (a: number, b: number, c: number) => void;
    readonly duel_toggle_wind: (a: number) => void;
    readonly duel_wind_on: (a: number) => number;
    readonly duel_you_positions: (a: number) => [number, number];
    readonly duel_you_up: (a: number) => number;
    readonly evolver_best_island: (a: number) => number;
    readonly evolver_champions_flat: (a: number) => [number, number];
    readonly evolver_evolve_islands: (a: number, b: number) => void;
    readonly evolver_fitnesses: (a: number) => [number, number];
    readonly evolver_generation: (a: number) => number;
    readonly evolver_n_islands: (a: number) => number;
    readonly evolver_new: (a: number, b: number) => number;
    readonly evolver_restart: (a: number) => void;
    readonly evolver_rollouts: (a: number) => number;
    readonly evolver_set_sharing: (a: number, b: number) => void;
    readonly evolver_sharing: (a: number) => number;
    readonly evolver_take_migrated: (a: number) => number;
    readonly freeswing_energy: (a: number) => number;
    readonly freeswing_links: (a: number) => number;
    readonly freeswing_new: (a: number, b: number) => number;
    readonly freeswing_nudge: (a: number, b: number) => void;
    readonly freeswing_positions: (a: number) => [number, number];
    readonly freeswing_set_damping: (a: number, b: number) => void;
    readonly freeswing_step: (a: number, b: number) => void;
    readonly np: () => number;
    readonly poparms_n_islands: (a: number) => number;
    readonly poparms_new: (a: number) => number;
    readonly poparms_positions_all: (a: number) => [number, number];
    readonly poparms_tick: (a: number, b: number, c: number, d: number) => void;
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
    readonly recover_current_name: (a: number) => [number, number];
    readonly recover_kind: (a: number) => number;
    readonly recover_knock: (a: number, b: number) => void;
    readonly recover_name_at: (a: number, b: number) => [number, number];
    readonly recover_new: () => number;
    readonly recover_num_kinds: (a: number) => number;
    readonly recover_outcome: (a: number) => number;
    readonly recover_positions: (a: number) => [number, number];
    readonly recover_step: (a: number, b: number) => void;
    readonly recover_tip: (a: number) => number;
    readonly ruvector_smoke: () => [number, number];
    readonly start: () => void;
    readonly recalibrator_time: (a: number) => number;
    readonly duel_time: (a: number) => number;
    readonly duel_you_balanced: (a: number) => number;
    readonly recalibrator_lag: (a: number) => number;
    readonly recalibrator_last_lag: (a: number) => number;
    readonly recalibrator_new_len: (a: number) => number;
    readonly recalibrator_recall_distance: (a: number) => number;
    readonly recalibrator_recalled_l1: (a: number) => number;
    readonly recover_best_tip: (a: number) => number;
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
