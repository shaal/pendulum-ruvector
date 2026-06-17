/* @ts-self-types="./pendulum_web.d.ts" */

/**
 * Station 6 — You vs RuVector.
 */
export class Duel {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        DuelFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_duel_free(ptr, 0);
    }
    add_payload() {
        wasm.duel_add_payload(this.__wbg_ptr);
    }
    /**
     * @returns {Float64Array}
     */
    auto_positions() {
        const ret = wasm.duel_auto_positions(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * @returns {boolean}
     */
    auto_up() {
        const ret = wasm.duel_auto_up(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * Fire the length disturbance (both arms' second link extends). The auto arm
     * starts a live RuVector recognition probe.
     */
    disturb() {
        wasm.duel_disturb(this.__wbg_ptr);
    }
    /**
     * @returns {boolean}
     */
    disturbed() {
        const ret = wasm.duel_disturbed(this.__wbg_ptr);
        return ret !== 0;
    }
    constructor() {
        const ret = wasm.duel_new();
        this.__wbg_ptr = ret;
        DuelFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @param {number} dir
     */
    poke_auto(dir) {
        wasm.duel_poke_auto(this.__wbg_ptr, dir);
    }
    /**
     * @returns {boolean}
     */
    recog_active() {
        const ret = wasm.duel_recog_active(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @returns {string}
     */
    recog_status() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.duel_recog_status(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    reset() {
        wasm.duel_reset(this.__wbg_ptr);
    }
    /**
     * Advance `steps` timesteps. `human_dir` ∈ {-1, 0, 1} (A / nothing / D).
     * @param {number} steps
     * @param {number} human_dir
     */
    step(steps, human_dir) {
        wasm.duel_step(this.__wbg_ptr, steps, human_dir);
    }
    /**
     * @returns {number}
     */
    time() {
        const ret = wasm.duel_time(this.__wbg_ptr);
        return ret;
    }
    toggle_wind() {
        wasm.duel_toggle_wind(this.__wbg_ptr);
    }
    /**
     * @returns {boolean}
     */
    wind_on() {
        const ret = wasm.duel_wind_on(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @returns {number}
     */
    you_balanced() {
        const ret = wasm.duel_you_balanced(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {Float64Array}
     */
    you_positions() {
        const ret = wasm.duel_you_positions(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * @returns {boolean}
     */
    you_up() {
        const ret = wasm.duel_you_up(this.__wbg_ptr);
        return ret !== 0;
    }
}
if (Symbol.dispose) Duel.prototype[Symbol.dispose] = Duel.prototype.free;

/**
 * Worker-side: the evolving population (no display arms).
 */
export class Evolver {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        EvolverFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_evolver_free(ptr, 0);
    }
    /**
     * @returns {number}
     */
    best_island() {
        const ret = wasm.evolver_best_island(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Flat champion parameters for every island (`n_islands * NP`) — what the main
     * thread needs to drive its display arms.
     * @returns {Float64Array}
     */
    champions_flat() {
        const ret = wasm.evolver_champions_flat(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Evolve `count` islands (round-robin); migrate when the sweep wraps.
     * @param {number} count
     */
    evolve_islands(count) {
        wasm.evolver_evolve_islands(this.__wbg_ptr, count);
    }
    /**
     * @returns {Float64Array}
     */
    fitnesses() {
        const ret = wasm.evolver_fitnesses(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * @returns {number}
     */
    generation() {
        const ret = wasm.evolver_generation(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * @returns {number}
     */
    n_islands() {
        const ret = wasm.evolver_n_islands(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * @param {boolean} sharing
     */
    constructor(sharing) {
        const ret = wasm.evolver_new(sharing);
        this.__wbg_ptr = ret;
        EvolverFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    restart() {
        wasm.evolver_restart(this.__wbg_ptr);
    }
    /**
     * @returns {number}
     */
    rollouts() {
        const ret = wasm.evolver_rollouts(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * @param {boolean} on
     */
    set_sharing(on) {
        wasm.evolver_set_sharing(this.__wbg_ptr, on);
    }
    /**
     * @returns {boolean}
     */
    sharing() {
        const ret = wasm.evolver_sharing(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @returns {boolean}
     */
    take_migrated() {
        const ret = wasm.evolver_take_migrated(this.__wbg_ptr);
        return ret !== 0;
    }
}
if (Symbol.dispose) Evolver.prototype[Symbol.dispose] = Evolver.prototype.free;

/**
 * Station 0 — a free-swinging n-link pendulum. Released from a sprawl and left
 * passive (no applied torque), it swings chaotically: the warm-up that motivates
 * why remembering past dynamics (RuVector) is worth anything.
 */
export class FreeSwing {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        FreeSwingFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_freeswing_free(ptr, 0);
    }
    /**
     * Total mechanical energy — used to show that the passive system conserves it
     * (and to compare native vs wasm: it should match the native reference).
     * @returns {number}
     */
    energy() {
        const ret = wasm.freeswing_energy(this.__wbg_ptr);
        return ret;
    }
    /**
     * Number of links.
     * @returns {number}
     */
    links() {
        const ret = wasm.freeswing_links(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * `links` ∈ [1, 6]; `damping` is per-joint viscous friction (0 = frictionless).
     * @param {number} links
     * @param {number} damping
     */
    constructor(links, damping) {
        const ret = wasm.freeswing_new(links, damping);
        this.__wbg_ptr = ret;
        FreeSwingFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * A tiny kick to the tip joint — the "chaos" button. Two identical arms given
     * this nudge diverge within seconds.
     * @param {number} delta
     */
    nudge(delta) {
        wasm.freeswing_nudge(this.__wbg_ptr, delta);
    }
    /**
     * Flat `[x0, y0, x1, y1, …]` joint positions including the anchor (n+1 points),
     * in physics units. The Canvas2D renderer scales these to pixels. Returned as
     * a `Float64Array` to JS.
     * @returns {Float64Array}
     */
    positions() {
        const ret = wasm.freeswing_positions(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Live-tune per-joint damping from a slider.
     * @param {number} d
     */
    set_damping(d) {
        wasm.freeswing_set_damping(this.__wbg_ptr, d);
    }
    /**
     * Advance the physics by `steps` fixed timesteps (passive — zero torque).
     * @param {number} steps
     */
    step(steps) {
        wasm.freeswing_step(this.__wbg_ptr, steps);
    }
}
if (Symbol.dispose) FreeSwing.prototype[Symbol.dispose] = FreeSwing.prototype.free;

/**
 * Main-thread: the live display arms, driven by champion parameters from the
 * worker. Cheap enough to step every frame.
 */
export class PopArms {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        PopArmsFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_poparms_free(ptr, 0);
    }
    /**
     * @returns {number}
     */
    n_islands() {
        const ret = wasm.poparms_n_islands(this.__wbg_ptr);
        return ret >>> 0;
    }
    constructor() {
        const ret = wasm.poparms_new();
        this.__wbg_ptr = ret;
        PopArmsFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * Flat positions for every arm: island 0's `[x0,y0,x1,y1,x2,y2]`, then 1's, …
     * @returns {Float64Array}
     */
    positions_all() {
        const ret = wasm.poparms_positions_all(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * Step every arm `steps` times, driven by its island's champion. `champions`
     * is the flat `n_islands * NP` array from `Evolver::champions_flat`.
     * @param {number} steps
     * @param {Float64Array} champions
     */
    tick(steps, champions) {
        const ptr0 = passArrayF64ToWasm0(champions, wasm.__wbindgen_malloc);
        const len0 = WASM_VECTOR_LEN;
        wasm.poparms_tick(this.__wbg_ptr, steps, ptr0, len0);
    }
}
if (Symbol.dispose) PopArms.prototype[Symbol.dispose] = PopArms.prototype.free;

/**
 * Station 2 — RuVector recognizes a changed arm and recalls its gain.
 */
export class Recalibrator {
    __destroy_into_raw() {
        const ptr = this.__wbg_ptr;
        this.__wbg_ptr = 0;
        RecalibratorFinalization.unregister(this);
        return ptr;
    }
    free() {
        const ptr = this.__destroy_into_raw();
        wasm.__wbg_recalibrator_free(ptr, 0);
    }
    /**
     * @returns {Float64Array}
     */
    adaptive_positions() {
        const ret = wasm.recalibrator_adaptive_positions(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * @returns {boolean}
     */
    committed() {
        const ret = wasm.recalibrator_committed(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @returns {boolean}
     */
    disturbed() {
        const ret = wasm.recalibrator_disturbed(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @returns {number}
     */
    encounter() {
        const ret = wasm.recalibrator_encounter(this.__wbg_ptr);
        return ret >>> 0;
    }
    /**
     * Wipe everything RuVector learned and re-seed the cold grid.
     */
    forget() {
        wasm.recalibrator_forget(this.__wbg_ptr);
    }
    /**
     * @returns {number}
     */
    lag() {
        const ret = wasm.recalibrator_lag(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    last_lag() {
        const ret = wasm.recalibrator_last_lag(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {Float64Array}
     */
    naive_positions() {
        const ret = wasm.recalibrator_naive_positions(this.__wbg_ptr);
        var v1 = getArrayF64FromWasm0(ret[0], ret[1]).slice();
        wasm.__wbindgen_free(ret[0], ret[1] * 8, 8);
        return v1;
    }
    /**
     * @param {number} new_l1
     */
    constructor(new_l1) {
        const ret = wasm.recalibrator_new(new_l1);
        this.__wbg_ptr = ret;
        RecalibratorFinalization.register(this, this.__wbg_ptr, this);
        return this;
    }
    /**
     * @returns {number}
     */
    new_len() {
        const ret = wasm.recalibrator_new_len(this.__wbg_ptr);
        return ret;
    }
    /**
     * Throw the same disturbance again, keeping what RuVector has learned —
     * the lag should shrink on a repeat.
     */
    next_encounter() {
        wasm.recalibrator_next_encounter(this.__wbg_ptr);
    }
    /**
     * "nominal" | "probing" | "recognized"
     * @returns {string}
     */
    phase() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.recalibrator_phase(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    /**
     * @returns {number}
     */
    recall_distance() {
        const ret = wasm.recalibrator_recall_distance(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {string}
     */
    recalled_id() {
        let deferred1_0;
        let deferred1_1;
        try {
            const ret = wasm.recalibrator_recalled_id(this.__wbg_ptr);
            deferred1_0 = ret[0];
            deferred1_1 = ret[1];
            return getStringFromWasm0(ret[0], ret[1]);
        } finally {
            wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
        }
    }
    /**
     * @returns {number}
     */
    recalled_l1() {
        const ret = wasm.recalibrator_recalled_l1(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {boolean}
     */
    recalled_learned() {
        const ret = wasm.recalibrator_recalled_learned(this.__wbg_ptr);
        return ret !== 0;
    }
    /**
     * @param {boolean} on
     */
    set_learning(on) {
        wasm.recalibrator_set_learning(this.__wbg_ptr, on);
    }
    /**
     * Set the disturbance length (link-2's new length); applied on the next
     * encounter. If still pre-disturbance this encounter, it takes effect here.
     * @param {number} l1
     */
    set_new_len(l1) {
        wasm.recalibrator_set_new_len(this.__wbg_ptr, l1);
    }
    /**
     * Advance the scenario by `steps` control timesteps.
     * @param {number} steps
     */
    tick(steps) {
        wasm.recalibrator_tick(this.__wbg_ptr, steps);
    }
    /**
     * @returns {number}
     */
    time() {
        const ret = wasm.recalibrator_time(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    tip_error_adaptive() {
        const ret = wasm.recalibrator_tip_error_adaptive(this.__wbg_ptr);
        return ret;
    }
    /**
     * @returns {number}
     */
    tip_error_naive() {
        const ret = wasm.recalibrator_tip_error_naive(this.__wbg_ptr);
        return ret;
    }
}
if (Symbol.dispose) Recalibrator.prototype[Symbol.dispose] = Recalibrator.prototype.free;

/**
 * Number of evolvable policy parameters per island champion (the stride of
 * `Evolver::champions_flat`).
 * @returns {number}
 */
export function np() {
    const ret = wasm.np();
    return ret >>> 0;
}

/**
 * Proof that RuVector's in-memory vector DB runs in the browser. Creates a tiny
 * in-memory store, inserts two vectors, and returns the id of the nearest match
 * to a query — entirely client-side, no server. Also keeps `ruvector-core` linked
 * into the wasm bundle so M0's size measurement reflects the real page.
 * @returns {string}
 */
export function ruvector_smoke() {
    let deferred1_0;
    let deferred1_1;
    try {
        const ret = wasm.ruvector_smoke();
        deferred1_0 = ret[0];
        deferred1_1 = ret[1];
        return getStringFromWasm0(ret[0], ret[1]);
    } finally {
        wasm.__wbindgen_free(deferred1_0, deferred1_1, 1);
    }
}

export function start() {
    wasm.start();
}
function __wbg_get_imports() {
    const import0 = {
        __proto__: null,
        __wbg___wbindgen_throw_ea4887a5f8f9a9db: function(arg0, arg1) {
            throw new Error(getStringFromWasm0(arg0, arg1));
        },
        __wbg_error_a6fa202b58aa1cd3: function(arg0, arg1) {
            let deferred0_0;
            let deferred0_1;
            try {
                deferred0_0 = arg0;
                deferred0_1 = arg1;
                console.error(getStringFromWasm0(arg0, arg1));
            } finally {
                wasm.__wbindgen_free(deferred0_0, deferred0_1, 1);
            }
        },
        __wbg_new_227d7c05414eb861: function() {
            const ret = new Error();
            return ret;
        },
        __wbg_stack_3b0d974bbf31e44f: function(arg0, arg1) {
            const ret = arg1.stack;
            const ptr1 = passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
            const len1 = WASM_VECTOR_LEN;
            getDataViewMemory0().setInt32(arg0 + 4 * 1, len1, true);
            getDataViewMemory0().setInt32(arg0 + 4 * 0, ptr1, true);
        },
        __wbindgen_init_externref_table: function() {
            const table = wasm.__wbindgen_externrefs;
            const offset = table.grow(4);
            table.set(0, undefined);
            table.set(offset + 0, undefined);
            table.set(offset + 1, null);
            table.set(offset + 2, true);
            table.set(offset + 3, false);
        },
    };
    return {
        __proto__: null,
        "./pendulum_web_bg.js": import0,
    };
}

const DuelFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_duel_free(ptr, 1));
const EvolverFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_evolver_free(ptr, 1));
const FreeSwingFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_freeswing_free(ptr, 1));
const PopArmsFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_poparms_free(ptr, 1));
const RecalibratorFinalization = (typeof FinalizationRegistry === 'undefined')
    ? { register: () => {}, unregister: () => {} }
    : new FinalizationRegistry(ptr => wasm.__wbg_recalibrator_free(ptr, 1));

function getArrayF64FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getFloat64ArrayMemory0().subarray(ptr / 8, ptr / 8 + len);
}

let cachedDataViewMemory0 = null;
function getDataViewMemory0() {
    if (cachedDataViewMemory0 === null || cachedDataViewMemory0.buffer.detached === true || (cachedDataViewMemory0.buffer.detached === undefined && cachedDataViewMemory0.buffer !== wasm.memory.buffer)) {
        cachedDataViewMemory0 = new DataView(wasm.memory.buffer);
    }
    return cachedDataViewMemory0;
}

let cachedFloat64ArrayMemory0 = null;
function getFloat64ArrayMemory0() {
    if (cachedFloat64ArrayMemory0 === null || cachedFloat64ArrayMemory0.byteLength === 0) {
        cachedFloat64ArrayMemory0 = new Float64Array(wasm.memory.buffer);
    }
    return cachedFloat64ArrayMemory0;
}

function getStringFromWasm0(ptr, len) {
    return decodeText(ptr >>> 0, len);
}

let cachedUint8ArrayMemory0 = null;
function getUint8ArrayMemory0() {
    if (cachedUint8ArrayMemory0 === null || cachedUint8ArrayMemory0.byteLength === 0) {
        cachedUint8ArrayMemory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8ArrayMemory0;
}

function passArrayF64ToWasm0(arg, malloc) {
    const ptr = malloc(arg.length * 8, 8) >>> 0;
    getFloat64ArrayMemory0().set(arg, ptr / 8);
    WASM_VECTOR_LEN = arg.length;
    return ptr;
}

function passStringToWasm0(arg, malloc, realloc) {
    if (realloc === undefined) {
        const buf = cachedTextEncoder.encode(arg);
        const ptr = malloc(buf.length, 1) >>> 0;
        getUint8ArrayMemory0().subarray(ptr, ptr + buf.length).set(buf);
        WASM_VECTOR_LEN = buf.length;
        return ptr;
    }

    let len = arg.length;
    let ptr = malloc(len, 1) >>> 0;

    const mem = getUint8ArrayMemory0();

    let offset = 0;

    for (; offset < len; offset++) {
        const code = arg.charCodeAt(offset);
        if (code > 0x7F) break;
        mem[ptr + offset] = code;
    }
    if (offset !== len) {
        if (offset !== 0) {
            arg = arg.slice(offset);
        }
        ptr = realloc(ptr, len, len = offset + arg.length * 3, 1) >>> 0;
        const view = getUint8ArrayMemory0().subarray(ptr + offset, ptr + len);
        const ret = cachedTextEncoder.encodeInto(arg, view);

        offset += ret.written;
        ptr = realloc(ptr, len, offset, 1) >>> 0;
    }

    WASM_VECTOR_LEN = offset;
    return ptr;
}

let cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
cachedTextDecoder.decode();
const MAX_SAFARI_DECODE_BYTES = 2146435072;
let numBytesDecoded = 0;
function decodeText(ptr, len) {
    numBytesDecoded += len;
    if (numBytesDecoded >= MAX_SAFARI_DECODE_BYTES) {
        cachedTextDecoder = new TextDecoder('utf-8', { ignoreBOM: true, fatal: true });
        cachedTextDecoder.decode();
        numBytesDecoded = len;
    }
    return cachedTextDecoder.decode(getUint8ArrayMemory0().subarray(ptr, ptr + len));
}

const cachedTextEncoder = new TextEncoder();

if (!('encodeInto' in cachedTextEncoder)) {
    cachedTextEncoder.encodeInto = function (arg, view) {
        const buf = cachedTextEncoder.encode(arg);
        view.set(buf);
        return {
            read: arg.length,
            written: buf.length
        };
    };
}

let WASM_VECTOR_LEN = 0;

let wasmModule, wasmInstance, wasm;
function __wbg_finalize_init(instance, module) {
    wasmInstance = instance;
    wasm = instance.exports;
    wasmModule = module;
    cachedDataViewMemory0 = null;
    cachedFloat64ArrayMemory0 = null;
    cachedUint8ArrayMemory0 = null;
    wasm.__wbindgen_start();
    return wasm;
}

async function __wbg_load(module, imports) {
    if (typeof Response === 'function' && module instanceof Response) {
        if (typeof WebAssembly.instantiateStreaming === 'function') {
            try {
                return await WebAssembly.instantiateStreaming(module, imports);
            } catch (e) {
                const validResponse = module.ok && expectedResponseType(module.type);

                if (validResponse && module.headers.get('Content-Type') !== 'application/wasm') {
                    console.warn("`WebAssembly.instantiateStreaming` failed because your server does not serve Wasm with `application/wasm` MIME type. Falling back to `WebAssembly.instantiate` which is slower. Original error:\n", e);

                } else { throw e; }
            }
        }

        const bytes = await module.arrayBuffer();
        return await WebAssembly.instantiate(bytes, imports);
    } else {
        const instance = await WebAssembly.instantiate(module, imports);

        if (instance instanceof WebAssembly.Instance) {
            return { instance, module };
        } else {
            return instance;
        }
    }

    function expectedResponseType(type) {
        switch (type) {
            case 'basic': case 'cors': case 'default': return true;
        }
        return false;
    }
}

function initSync(module) {
    if (wasm !== undefined) return wasm;


    if (module !== undefined) {
        if (Object.getPrototypeOf(module) === Object.prototype) {
            ({module} = module)
        } else {
            console.warn('using deprecated parameters for `initSync()`; pass a single object instead')
        }
    }

    const imports = __wbg_get_imports();
    if (!(module instanceof WebAssembly.Module)) {
        module = new WebAssembly.Module(module);
    }
    const instance = new WebAssembly.Instance(module, imports);
    return __wbg_finalize_init(instance, module);
}

async function __wbg_init(module_or_path) {
    if (wasm !== undefined) return wasm;


    if (module_or_path !== undefined) {
        if (Object.getPrototypeOf(module_or_path) === Object.prototype) {
            ({module_or_path} = module_or_path)
        } else {
            console.warn('using deprecated parameters for the initialization function; pass a single object instead')
        }
    }

    if (module_or_path === undefined) {
        module_or_path = new URL('pendulum_web_bg.wasm', import.meta.url);
    }
    const imports = __wbg_get_imports();

    if (typeof module_or_path === 'string' || (typeof Request === 'function' && module_or_path instanceof Request) || (typeof URL === 'function' && module_or_path instanceof URL)) {
        module_or_path = fetch(module_or_path);
    }

    const { instance, module } = await __wbg_load(await module_or_path, imports);

    return __wbg_finalize_init(instance, module);
}

export { initSync, __wbg_init as default };
