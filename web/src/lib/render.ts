// Canvas2D rendering for the arms. Positions arrive from WASM as a flat
// [x0,y0, x1,y1, …] array in physics units (metres), anchor first, +y is up.

type Pt = [number, number]

export interface ArmStyle {
  rod: string
  joint: string
  tip: string
  tipRing?: string
  trail?: string
}

export const TEAL: ArmStyle = { rod: '#0f766e', joint: '#14b8a6', tip: '#f97316', tipRing: '#ea580c', trail: '249,115,22' }
export const RED: ArmStyle = { rod: '#b91c1c', joint: '#ef4444', tip: '#ef4444', tipRing: '#7f1d1d', trail: '239,68,68' }
export const GREEN: ArmStyle = { rod: '#15803d', joint: '#22c55e', tip: '#f97316', tipRing: '#ea580c', trail: '34,197,94' }

function setup(canvas: HTMLCanvasElement) {
  const ctx = canvas.getContext('2d')!
  const dpr = window.devicePixelRatio || 1
  const w = canvas.clientWidth
  const h = canvas.clientHeight
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr)
    canvas.height = Math.round(h * dpr)
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx.clearRect(0, 0, w, h)
  return { ctx, w, h }
}

function groundLine(ctx: CanvasRenderingContext2D, w: number, y: number) {
  ctx.strokeStyle = 'rgba(154,52,18,0.08)'
  ctx.lineWidth = 1
  ctx.beginPath()
  ctx.moveTo(0, y)
  ctx.lineTo(w, y)
  ctx.stroke()
}

function paintArm(
  ctx: CanvasRenderingContext2D,
  positions: ArrayLike<number>,
  ox: number,
  oy: number,
  scale: number,
  style: ArmStyle,
  trail?: Pt[],
) {
  const sx = (x: number) => ox + x * scale
  const sy = (y: number) => oy - y * scale

  if (trail && style.trail) {
    for (let i = 1; i < trail.length; i++) {
      const a = i / trail.length
      ctx.strokeStyle = `rgba(${style.trail},${a * 0.5})`
      ctx.lineWidth = 3
      ctx.beginPath()
      ctx.moveTo(sx(trail[i - 1][0]), sy(trail[i - 1][1]))
      ctx.lineTo(sx(trail[i][0]), sy(trail[i][1]))
      ctx.stroke()
    }
  }

  const n = positions.length / 2
  ctx.lineCap = 'round'
  ctx.lineJoin = 'round'
  ctx.strokeStyle = style.rod
  ctx.lineWidth = 8
  ctx.beginPath()
  ctx.moveTo(sx(positions[0]), sy(positions[1]))
  for (let i = 1; i < n; i++) ctx.lineTo(sx(positions[2 * i]), sy(positions[2 * i + 1]))
  ctx.stroke()

  ctx.fillStyle = '#9a3412'
  ctx.beginPath()
  ctx.arc(sx(0), sy(0), 7, 0, Math.PI * 2)
  ctx.fill()

  for (let i = 1; i < n; i++) {
    const isTip = i === n - 1
    ctx.fillStyle = isTip ? style.tip : style.joint
    ctx.beginPath()
    ctx.arc(sx(positions[2 * i]), sy(positions[2 * i + 1]), isTip ? 11 : 7, 0, Math.PI * 2)
    ctx.fill()
    if (isTip && style.tipRing) {
      ctx.strokeStyle = style.tipRing
      ctx.lineWidth = 3
      ctx.stroke()
    }
  }
}

function label(ctx: CanvasRenderingContext2D, text: string, color: string, x: number, y: number) {
  ctx.font = '700 15px ui-rounded, system-ui, sans-serif'
  ctx.textAlign = 'center'
  ctx.fillStyle = color
  ctx.fillText(text, x, y)
}

/** Single centered arm (the toy / free-swing station). */
export function drawArm(canvas: HTMLCanvasElement, positions: ArrayLike<number>, reach: number, trail: Pt[]) {
  const { ctx, w, h } = setup(canvas)
  const oy = h * 0.42
  groundLine(ctx, w, oy)
  const scale = (Math.min(w, h) * 0.4) / Math.max(reach, 1)
  paintArm(ctx, positions, w / 2, oy, scale, TEAL, trail)
}

/** Two arms side by side (the recognize / duel stations). */
export function drawDuel(
  canvas: HTMLCanvasElement,
  left: ArrayLike<number>,
  right: ArrayLike<number>,
  reach: number,
  leftStyle: ArmStyle,
  rightStyle: ArmStyle,
  leftLabel: string,
  rightLabel: string,
) {
  const { ctx, w, h } = setup(canvas)
  const oy = h * 0.46
  groundLine(ctx, w, oy)
  const scale = (Math.min(w / 2, h) * 0.36) / Math.max(reach, 1)
  paintArm(ctx, left, w * 0.27, oy, scale, leftStyle)
  paintArm(ctx, right, w * 0.73, oy, scale, rightStyle)
  label(ctx, leftLabel, leftStyle.rod, w * 0.27, h - 14)
  label(ctx, rightLabel, rightStyle.rod, w * 0.73, h - 14)
}
