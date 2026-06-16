// Canvas2D renderer for an n-link arm. Positions arrive from WASM as a flat
// [x0,y0, x1,y1, …] array in physics units (metres), anchor first, +y is up.

type Pt = [number, number]

export function drawArm(
  canvas: HTMLCanvasElement,
  positions: ArrayLike<number>,
  reach: number,
  trail: Pt[],
) {
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  const dpr = window.devicePixelRatio || 1
  const w = canvas.clientWidth
  const h = canvas.clientHeight
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr)
    canvas.height = Math.round(h * dpr)
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  ctx.clearRect(0, 0, w, h)

  const cx = w / 2
  const cy = h * 0.42
  const scale = (Math.min(w, h) * 0.4) / Math.max(reach, 1)
  const sx = (x: number) => cx + x * scale
  const sy = (y: number) => cy - y * scale

  // faint ground line through the pivot
  ctx.strokeStyle = 'rgba(154,52,18,0.08)'
  ctx.lineWidth = 1
  ctx.beginPath()
  ctx.moveTo(0, sy(0))
  ctx.lineTo(w, sy(0))
  ctx.stroke()

  // fading tip trail
  for (let i = 1; i < trail.length; i++) {
    const a = i / trail.length
    ctx.strokeStyle = `rgba(249,115,22,${a * 0.5})`
    ctx.lineWidth = 3
    ctx.beginPath()
    ctx.moveTo(sx(trail[i - 1][0]), sy(trail[i - 1][1]))
    ctx.lineTo(sx(trail[i][0]), sy(trail[i][1]))
    ctx.stroke()
  }

  const n = positions.length / 2

  // rods
  ctx.lineCap = 'round'
  ctx.lineJoin = 'round'
  ctx.strokeStyle = '#0f766e'
  ctx.lineWidth = 8
  ctx.beginPath()
  ctx.moveTo(sx(positions[0]), sy(positions[1]))
  for (let i = 1; i < n; i++) ctx.lineTo(sx(positions[2 * i]), sy(positions[2 * i + 1]))
  ctx.stroke()

  // pivot
  ctx.fillStyle = '#9a3412'
  ctx.beginPath()
  ctx.arc(sx(0), sy(0), 7, 0, Math.PI * 2)
  ctx.fill()

  // joints + tip bob
  for (let i = 1; i < n; i++) {
    const isTip = i === n - 1
    ctx.fillStyle = isTip ? '#f97316' : '#14b8a6'
    ctx.beginPath()
    ctx.arc(sx(positions[2 * i]), sy(positions[2 * i + 1]), isTip ? 11 : 7, 0, Math.PI * 2)
    ctx.fill()
    if (isTip) {
      ctx.strokeStyle = '#ea580c'
      ctx.lineWidth = 3
      ctx.stroke()
    }
  }
}
