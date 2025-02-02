/* eslint-disable prefer-const */
import AudioOut from 'pins/audioout'
import OpenAIStreamer from 'openaistreamer'
import calculatePower from 'calculate-power'

/* global trace, SharedArrayBuffer */

export type TTSProperty = {
  onPlayed?: (number) => void
  onDone?: () => void
  token: string
  model?: string
  voice?: string
  speed?: number
  volume?: number
}

export class TTS {
  audio?: AudioOut
  onPlayed?: (number) => void
  onDone?: () => void
  token: string
  model: string
  voice: string
  speed: number
  volume: number
  streaming: boolean
  constructor(props: TTSProperty) {
    this.onPlayed = props.onPlayed
    this.onDone = props.onDone
    this.streaming = false
    this.token = props.token
    this.model = props.model ?? 'tts-1'
    this.voice = props.voice ?? 'alloy'
    this.speed = props.speed ?? 1
    this.volume = props.volume ?? 100
  }
  async stream(text: string, volume?: number): Promise<void> {
    if (this.streaming) {
      throw new Error('already playing')
    }
    this.streaming = true
    const { onPlayed, onDone } = this
    return new Promise((resolve, reject) => {
      this.audio = new AudioOut({ streams: 1, bitsPerSample: 16, sampleRate: 24000 })
      this.audio.enqueue(0, AudioOut.Volume, Math.round((volume ?? this.volume) * 256))
      const audio = this.audio
      const streamer = new OpenAIStreamer({
        input: text,
        key: this.token,
        model: this.model,
        voice: this.voice,
        speed: this.speed,
        response_format: 'wav',
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
