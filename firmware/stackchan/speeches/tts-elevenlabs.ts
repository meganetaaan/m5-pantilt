/* eslint-disable prefer-const */
import AudioOut from 'pins/audioout'
import ElevenLabsStreamer from 'elevenlabsstreamer'
import calculatePower from 'calculate-power'

/* global trace, SharedArrayBuffer */

type voiceSettings = {
  similarity_boost: number
  stability: number
  style?: number
  use_speaker_boost?: boolean
}

export type TTSProperty = {
  onPlayed?: (number) => void
  onDone?: () => void
  token: string
  voice?: string
  latency?: number
  format?: string
  model?: string
  voice_settings?: voiceSettings
  volume?: number
}

export class TTS {
  audio?: AudioOut
  onPlayed?: (number) => void
  onDone?: () => void
  token: string
  model: string
  voice: string
  latency: number
  format: string
  voice_settings: voiceSettings
  volume: number
  streaming: boolean
  constructor(props: TTSProperty) {
    this.onPlayed = props.onPlayed
    this.onDone = props.onDone
    this.streaming = false
    this.token = props.token
    this.latency = props.latency ?? 2
    this.format = props.format ?? 'mp3_44100_64'
    this.model = props.model ?? 'eleven_monolingual_v1'
    this.voice = props.voice ?? 'AZnzlk1XvdvUeBnXmlld'
    this.voice_settings = props.voice_settings
    this.volume = props.volume ?? 0.5
  }
  async stream(text: string, volume: number): Promise<void> {
    if (this.streaming) {
      throw new Error('already playing')
    }
    this.streaming = true

    const { onPlayed, onDone } = this
    return new Promise((resolve, reject) => {
      this.audio = new AudioOut({ streams: 1, bitsPerSample: 16, sampleRate: 44100 })
      this.audio.enqueue(0, AudioOut.Volume, Math.round((volume ?? this.volume) * 256))
      const audio = this.audio
      const streamer = new ElevenLabsStreamer({
        key: this.token,
        voice: this.voice,
        model: this.model,
        latency: this.latency,
        format: this.format,
        voice_settings: this.voice_settings,
        text,
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
