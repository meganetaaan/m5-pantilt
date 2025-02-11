/* eslint-disable prefer-const */
import AudioOut from 'pins/audioout'
import WavStreamer from 'wavstreamer'
import calculatePower from 'calculate-power'
import type HTTPClient from 'embedded:network/http/client'

/* global trace, SharedArrayBuffer */
declare const device: {
  network: {
    http: typeof HTTPClient.constructor & {
      io: typeof HTTPClient
      socket: unknown
      dns: unknown
    }
  }
}

export type TTSProperty = {
  onPlayed?: (number) => void
  onDone?: () => void
  host: string
  port: number
  sampleRate?: number
  volume?: number
}

export class TTS {
  audio?: AudioOut
  onPlayed?: (number) => void
  onDone?: () => void
  host: string
  port: number
  sampleRate: number
  volume: number
  streaming: boolean
  constructor(props: TTSProperty) {
    this.onPlayed = props.onPlayed
    this.onDone = props.onDone
    this.streaming = false
    this.host = props.host
    this.port = props.port
    this.sampleRate = props.sampleRate ?? 24000
    this.volume = props.volume ?? 0.5
  }
  async stream(key: string, volume?: number): Promise<void> {
    if (this.streaming) {
      throw new Error('already playing')
    }
    this.streaming = true
    const { onPlayed, onDone } = this
    return new Promise((resolve, reject) => {
      this.audio = new AudioOut({ streams: 1, sampleRate: this.sampleRate })
      this.audio.enqueue(0, AudioOut.Volume, Math.round((volume ?? this.volume) * 256))
      const audio = this.audio
      const streamer = new WavStreamer({
        http: device.network.http,
        host: this.host,
        path: key,
        port: this.port,
        bufferDuration: 600,
        audio: {
          out: audio,
          stream: 0,
        },
        onPlayed(buffer) {
          const power = calculatePower(buffer)
          onPlayed?.(power)
        },
        onReady(state) {
          trace(`Ready: ${state}\n`)
          if (state) {
            audio.start()
          } else {
            audio.stop()
          }
        },
        onError: (e) => {
          trace('ERROR: ', e, '\n')
          this.streaming = false
          streamer?.close()
          this.audio?.close()
          this.audio = undefined
          reject(e)
        },
        onDone: () => {
          trace('DONE\n')
          this.streaming = false
          streamer?.close()
          this.audio?.close()
          this.audio = undefined
          onDone?.()
          resolve()
        },
      })
    })
  }
}
