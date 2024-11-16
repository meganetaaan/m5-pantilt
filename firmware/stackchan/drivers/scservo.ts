import Serial from 'embedded:io/serial'
import config from 'mc/config'
import Timer from 'timer'

type Maybe<T> =
  | {
      success: true
      value: T
    }
  | {
      success: false
      reason?: string
    }

// utilities
function clamp(v: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, v))
}
function le(v: number): [number, number] {
  return [(v & 0xff00) >> 8, v & 0xff]
}
function el(h: number, l: number) {
  return ((h << 8) & 0xff00) + (l & 0xff)
}

const BROADCAST_ID = 0xfe // 254
const MAX_ID = 0xfc // 252
const SCS_END = 0

const COMMAND = {
  RESPONSE: 0x00,
  // NOTE: Some servo returns response with command 0x01. Dunno why.
  RESPONSE_ALT: 0x01,
  WRITE: 0x03,
  READ: 0x02,
} as const
type Command = (typeof COMMAND)[keyof typeof COMMAND]

const ADDRESS = {
  ID: 5,
  OFFSET: 31,
  TORQUE_ENABLE: 40,
  GOAL_ACC: 41,
  GOAL_POSITION: 42,
  GOAL_TIME: 44,
  LOCK: 48,
  PRESENT_POSITION: 56,
} as const
type Address = (typeof ADDRESS)[keyof typeof ADDRESS]

const RX_STATE = {
  SEEK: 0,
  HEAD: 1,
  BODY: 2,
} as const
type RxState = (typeof RX_STATE)[keyof typeof RX_STATE]

class PacketHandler extends Serial {
  #callbacks: Map<number, (bytes: number[]) => void>
  #rxBuffer: Uint8Array
  #idx: number
  #state: RxState
  #count: number
  constructor(option) {
    const onReadable = function (this: PacketHandler, bytes: number) {
      const rxBuf = this.#rxBuffer
      while (bytes > 0) {
        // NOTE: We can safely read a number
        rxBuf[this.#idx++] = this.read() as number
        bytes -= 1
        switch (this.#state) {
          case RX_STATE.SEEK:
            if (this.#idx >= 2) {
              // see header
              if (rxBuf[0] === 0xff && rxBuf[1] === 0xff) {
                // packet found
                this.#state = RX_STATE.HEAD
              } else {
                // reset seek
                // trace('seeking failed. reset\n')
                this.#idx = 0
              }
            }
            break
          case RX_STATE.HEAD:
            if (this.#idx >= 4) {
              this.#count = rxBuf[3]
              this.#state = RX_STATE.BODY
            }
            break
          case RX_STATE.BODY:
            this.#count -= 1
            if (this.#count === 0) {
              // trace('received packet!\n')
              const cs = checksum(rxBuf.slice(0, this.#idx - 1)) & 0xff
              const id = rxBuf[2]
              const command = rxBuf[4] as Command
              if (command === COMMAND.READ || command === COMMAND.WRITE) {
                // trace(`got echo.  ... ${rxBuf.slice(0, this.#idx)} ignoring\n`)
              } else if (cs === rxBuf[this.#idx - 1] && this.#callbacks.has(id)) {
                // trace(`got response for ${id}. triggering callback \n`)
                this.#callbacks.get(id)(Array.from(rxBuf.slice(5, this.#idx - 1)))
              } else {
                trace(`unknown packet for ${id} ... ${rxBuf.slice(0, this.#idx)}. ignoring\n`)
              }
              this.#idx = 0
              this.#state = RX_STATE.SEEK
            }
            break
          default:
            // @ts-ignore 6113
            let _state: never
        }
        // noop
      }
    }
    super({
      ...option,
      format: 'number',
      onReadable,
    })
    this.#callbacks = new Map<number, () => void>()
    this.#rxBuffer = new Uint8Array(64)
    this.#idx = 0
    this.#state = RX_STATE.SEEK
  }
  hasCallbackOf(id: number): boolean {
    return this.#callbacks.has(id)
  }
  registerCallback(id: number, callback: (bytes: number[]) => void) {
    this.#callbacks.set(id, callback)
  }
  removeCallback(id: number) {
    this.#callbacks.delete(id)
  }
}

/**
 * calculates checksum of the SCS packets
 * @param arr packet array except checksum
 * @returns checksum number
 */
function checksum(arr: number[] | Uint8Array): number {
  let sum = 0
  for (const n of arr.slice(2)) {
    sum += n
  }
  const cs = ~(sum & 0xff)
  // trace(`>>>checksum is ${new Uint8Array([cs])[0]}: ${arr}\n`)
  return cs
}

type SCServoConstructorParam = {
  id: number
}

let packetHandler: PacketHandler = null
class SCServo {
  #id: number
  #onCommandRead: (values: number[]) => void
  #txBuf: Uint8Array
  #promises: Array<[(values: number[]) => void, Timer]>
  #offset: number
  constructor({ id }: SCServoConstructorParam) {
    this.#id = id
    this.#promises = []
    this.#offset = 0
    this.#onCommandRead = (values) => {
      if (this.#promises.length > 0) {
        const [resolver, timeoutId] = this.#promises.shift()
        Timer.clear(timeoutId)
        resolver(values)
      }
    }
    this.#txBuf = new Uint8Array(64)
    if (packetHandler == null) {
      packetHandler = new PacketHandler({
        receive: config.serial?.receive ?? 16,
        transmit: config.serial?.transmit ?? 17,
        baud: 1_000_000,
        port: 2,
      })
    }
    if (packetHandler.hasCallbackOf(id)) {
      throw new Error('This id is already instantiated')
    }
    packetHandler.registerCallback(this.#id, this.#onCommandRead)
  }
  teardown(): void {
    packetHandler.removeCallback(this.#id)
  }
  get id(): number {
    return this.#id
  }

  async #sendCommand(command: Command, address: Address, ...values: number[]): Promise<number[]> {
    this.#txBuf[0] = 0xff
    this.#txBuf[1] = 0xff
    this.#txBuf[2] = this.#id
    this.#txBuf[3] = values.length + 3
    this.#txBuf[4] = command // write or read
    this.#txBuf[5] = address
    let idx = 6
    for (const v of values) {
      this.#txBuf[idx] = v
      idx++
    }
    this.#txBuf[idx] = checksum(this.#txBuf.slice(0, idx))
    idx++
    // trace(`writing: ${this.#txBuf.slice(0, idx)}\n`)
    for (let i = 0; i < idx; i++) {
      packetHandler.write(this.#txBuf[i])
    }
    return new Promise((resolve, _reject) => {
      const id = Timer.set(() => {
        this.#promises.shift()
        trace(`timeout. ${this.#promises.length}\n`)
        resolve(undefined)
      }, 40)
      this.#promises.push([resolve, id])
    })
  }

  async #lock(): Promise<unknown> {
    return this.#sendCommand(COMMAND.WRITE, ADDRESS.LOCK, 1)
  }

  async #unlock(): Promise<unknown> {
    return this.#sendCommand(COMMAND.WRITE, ADDRESS.LOCK, 0)
  }

  /**
   * reads offset angle
   * @note SCS series does not have zero position calibration function.
   *  The offset value should be handled by the application.
   */
  async readOffsetAngle(): Promise<number> {
    const values = await this.#sendCommand(COMMAND.READ, ADDRESS.OFFSET, 2)
    const isCcw = Boolean(values[0] & 0x8000)
    let offset = ((values[0] & 0x7fff) << 8) | values[1]
    if (isCcw) {
      offset *= -1
    }
    return offset
  }

  /**
   * sets offset angle
   * @param angle offset angle (-2000 to 2000)
   */
  async setOffsetAngle(angle: number): Promise<unknown> {
    this.#offset = angle
    const isCcw = angle < 0
    if (isCcw) {
      angle *= -1
    }
    const value = (Number(isCcw) << 15) | (angle & 0x7fff)
    return this.#sendCommand(COMMAND.WRITE, ADDRESS.OFFSET, ...le(value))
  }

  /**
   * load settings from the servo
   */
  async loadSettings(): Promise<unknown> {
    // Offset angle
    this.#offset = await this.readOffsetAngle()

    // Further configuration to be loaded below
    return
  }

  /**
   * save settings to the servo
   */
  async saveSettings(): Promise<unknown> {
    // Offset angle
    await this.#unlock()
    await this.setOffsetAngle(this.#offset)
    await this.#lock()

    // Further configuration to be loaded below
    return
  }

  async flashId(id: number): Promise<unknown> {
    if (packetHandler.hasCallbackOf(id)) {
      throw new Error(`id(${id}) is already used\n`)
    }
    // trace('unlocking\n')
    await this.#unlock()
    // trace('setting new id\n')
    const promise = this.#sendCommand(COMMAND.WRITE, ADDRESS.ID, id)
    const oldId = this.#id
    this.#id = id
    packetHandler.registerCallback(this.#id, this.#onCommandRead)
    // trace(`now we use new id(${id}\n`)
    await promise
    // trace('locking\n')
    await this.#lock()
    // trace(`now we use new id(${id}\n`)
    packetHandler.removeCallback(oldId)
    return
  }

  /**
   * sets angle immediately
   * @param angle angle(degree)
   * @returns TBD
   */
  async setAngle(angle: number): Promise<unknown> {
    const a = Math.floor(clamp(((angle + this.#offset) * 1024) / 200, 0, 0x03ff))
    return this.#sendCommand(COMMAND.WRITE, ADDRESS.GOAL_POSITION, ...le(a))
  }

  /**
   * sets angle within goal time
   * @param angle angle(degree)
   * @param goalTime time(millisecond)
   * @returns TBD
   */
  async setAngleInTime(angle: number, goalTime: number): Promise<unknown> {
    // 0 <= a <= 1023
    const a = Math.floor(clamp(((angle + this.#offset) * 1024) / 200, 0, 0x03ff))
    const res = await this.#sendCommand(COMMAND.WRITE, ADDRESS.GOAL_POSITION, ...le(a), ...le(goalTime))
    return res
  }

  /**
   * sets torque
   * @param enable enable
   * @returns TBD
   */
  async setTorque(enable: boolean): Promise<unknown> {
    return this.#sendCommand(COMMAND.WRITE, ADDRESS.TORQUE_ENABLE, Number(enable))
  }

  /**
   * reads servo's present status
   * @returns angle(degree)
   */
  async readStatus(): Promise<Maybe<{ angle: number }>> {
    const values = await this.#sendCommand(COMMAND.READ, ADDRESS.PRESENT_POSITION, 15)
    if (values == null || values.length < 15) {
      return {
        success: false,
        reason: 'response corrupted.',
      }
    }
    const angle = (el(values[0], values[1]) * 200) / 1024
    return {
      success: true,
      value: { angle },
    }
  }
}

export default SCServo
