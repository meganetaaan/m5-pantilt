/* eslint-disable prefer-const */
import AudioOut from 'pins/audioout'
import MP3Streamer from 'mp3streamer'
import calculatePower from 'calculate-power'
import { fetch } from 'fetch'
import { URL } from 'url'
import type HTTPClient from 'embedded:network/http/client'

/* global trace, SharedArrayBuffer */
declare const device: {
  network: {
    https: typeof HTTPClient.constructor & {
      io: typeof HTTPClient
      socket: unknown
      dns: unknown
    }
  }
}

export type TTSProperty = {
  onPlayed?: (number) => void
  onDone?: () => void
  token: string
  sampleRate?: number
  speakerId?: number
  volume?: number
}

export class TTS {
  audio?: AudioOut
  onPlayed?: (number) => void
  onDone?: () => void
  token: string
  streaming: boolean
  speakerId: number
  sampleRate?: number
  volume: number
  constructor(props: TTSProperty) {
    this.onPlayed = props.onPlayed
    this.onDone = props.onDone
    this.streaming = false
    this.speakerId = props.speakerId ?? 1
    this.token = props.token
    this.volume = props.volume ?? 0.5
  }

  async getQuery(text: string, speakerId = 1): Promise<string> {
    return fetch(
      encodeURI(`https://api.tts.quest/v3/voicevox/synthesis?key=${this.token}&text=${text}&speaker=${speakerId}`),
    )
      .then((response) => {
        if (response.status !== 200) {
          throw new Error(`response error:${response.status}`)
        }
        return response.json()
      })
      .then((data) => {
        trace(`isApiKeyValid: ${data.isApiKeyValid}\n`)
        trace(`mp3StreamingUrl: ${data.mp3StreamingUrl}\n`)
        return data.mp3StreamingUrl
      })
  }

  async stream(key: string, volume?: number): Promise<void> {
    if (this.streaming) {
      throw new Error('already playing')
    }
    this.streaming = true

    const speakerId = this.speakerId
    const streamUrl = await this.getQuery(key, speakerId).catch((error) => {
      throw new Error(`getQuery failed: ${error}`)
    })
    const url = new URL(streamUrl)
    const { onPlayed, onDone } = this

    return new Promise((resolve, reject) => {
      this.audio = new AudioOut({ streams: 1, bitsPerSample: 16, sampleRate: this.sampleRate ?? 22050 })
      this.audio.enqueue(0, AudioOut.Volume, Math.round((volume ?? this.volume) * 256))
      const audio = this.audio
      const streamer = new MP3Streamer({
        http: device.network.https,
        host: url.host,
        path: url.pathname,
        port: 443,
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
