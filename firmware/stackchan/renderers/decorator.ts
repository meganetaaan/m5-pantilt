import parseBMF from 'commodetto/parseBMF'
import { type FaceDecoratorFactory } from 'renderer-base'
import { Outline } from 'commodetto/outline'

export const createBalloonDecorator: FaceDecoratorFactory<
  {
    left: number
    top: number
    bottom: number
    right: number
    width: number
    height: number
    font: ReturnType<typeof parseBMF>
    text: string
  },
  boolean
> = ({ left, top, bottom, right, width, height, font, text }) => {
  const outline = Outline.fill(Outline.RoundRectPath(0, 0, width, height, 6))
  let textX = 0
  const space = 20
  return (tick, poco, { theme }, end = false) => {
    const x = left ?? (right != null ? poco.width - right - width : (poco.width - width) / 2)
    const y = top ?? (bottom != null ? poco.height - bottom - height : (poco.height - height) / 2)
    const textWidth = poco.getTextWidth(text, font)
    const bg = poco.makeColor(...theme.secondary)
    const white = poco.makeColor(0xff, 0xff, 0xff)
    const black = poco.makeColor(0x00, 0x00, 0x00)
    poco.begin(x, y, width, height)
    poco.fillRectangle(bg, 0, 0, poco.width, poco.height)
    if (end) {
      poco.end()
      return
    }
    poco.blendOutline(white, 255, outline, x, y)
    poco.drawText(text, font, black, x - textX, y)
    if (textWidth > width) {
      if (textWidth + space >= Math.floor(textX)) {
        poco.drawText(text, font, black, x - textX + textWidth + space, y)
      }
      textX = textX >= textWidth + space ? 2 : textX + tick / 30
    }
    poco.end()
  }
}

export const createBubbleDecorator: FaceDecoratorFactory<{
  x: number
  y: number
  width: number
  height: number
}> = ({ x, y, width, height }) => {
  const bubbles: { x: number; vx: number; y: number; r: number }[] = []
  for (let i = 0; i < 4; i++) {
    bubbles.push({
      x: Math.random() * width,
      vx: 0,
      y: Math.random() * height,
      r: 4 + Math.random() * 3,
    })
  }
  let count = 0
  return (tick, poco, { theme }, end = false) => {
    poco.begin(x, y, width, height)
    const fg = poco.makeColor(...theme.primary)
    const bg = poco.makeColor(...theme.secondary)
    poco.fillRectangle(bg, x, y, width, height)
    const path = new Outline.CanvasPath()
    count = (count + tick) % 1000
    for (const b of bubbles) {
      // 上昇速度を泡の半径に依存させる
      const upwardSpeed = 1 - b.r / 12

      // 水平方向の動きは微妙にする
      b.vx = b.vx * 0.85 + 0.1 * (Math.random() - 0.5)
      b.x += b.vx

      // x座標の範囲を考慮
      b.x = Math.max(b.r, Math.min(width - b.r, b.x))

      // y座標（上昇）
      b.y = b.y + upwardSpeed * 2
      if (b.y > height - b.r) {
        b.y = b.r
        b.x = width * (1 - Math.random() * 0.2)
        b.vx = -3
      }

      // 半径を微妙に変化させる（オプション）
      b.r = Math.max(3, Math.min(12, b.r + 0.2 * (Math.random() - 0.5)))

      // 描画
      path.arc(x + b.x, y + height - b.y, b.r, 0, 2 * Math.PI)
    }
    poco.blendOutline(fg, 255, Outline.stroke(path, 2), 0, 0)
    poco.end()
  }
}
