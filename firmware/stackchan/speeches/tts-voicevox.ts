/* eslint-disable prefer-const */
import AudioOut from 'pins/audioout'
import WavStreamer from 'wavstreamer'
import calculatePower from 'calculate-power'
import type HTTPClient from 'embedded:network/http/client'
import Headers from 'headers'
import { File } from 'file'
import config from 'mc/config'

const QUERY_PATH = `${config.file.root}query.json`

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
  speakerId?: number
  volume?: number
}

export class TTS {
  audio?: AudioOut
  onPlayed?: (number) => void
  onDone?: () => void
  client: HTTPClient
  host: string
  port: number
  streaming: boolean
  file: File
  speakerId: number
  sampleRate: number
  volume: number
  constructor(props: TTSProperty) {
    this.onPlayed = props.onPlayed
    this.onDone = props.onDone
    this.streaming = false
    this.speakerId = props.speakerId ?? 1
    this.host = props.host
    this.port = props.port
    this.sampleRate = props.sampleRate ?? 11025
    this.volume = props.volume ?? 0.5
  }
  async getQuery(text: string, speakerId = 1): Promise<void> {
    return new Promise((resolve, reject) => {
      File.delete(QUERY_PATH)
      const file = new File(QUERY_PATH, true)
      const sampleRate = this.sampleRate
      const client = new device.network.http.io({
        ...device.network.http,
        host: this.host,
        port: this.port,
      })
      client.request({
        method: 'POST',
        path: encodeURI(`/audio_query?text=${text}&speaker=${speakerId}`),
        // TODO: https://github.com/Moddable-OpenSource/moddable/pull/1420
        // @ts-ignore
        headers: new Headers([['Content-Type', 'application/x-www-form-urlencoded']]),
        onHeaders(status) {
          if (status !== 200) {
            file.close()
            client.close()
            reject(`server returned ${status}`)
          }
        },
        onReadable(count) {
          file.write(this.read(count))
          // trace(`${count} bytes written. position: ${file.position}\n`)
        },
        onDone(error) {
          if (error) {
            file.close()
            client.close()
            reject(`unknown error occured:${error.message}`)
          } else {
            if (sampleRate !== 24000) {
              file.position = file.length - 1
              file.write(`, "outputSamplingRate": ${sampleRate}}`)
            }
            file.close()
            client.close()
            resolve()
          }
        },
      })
    })
  }
  async stream(key: string, volume?: number): Promise<void> {
    if (this.streaming) {
      throw new Error('already playing')
    }
    this.streaming = true

    const host = this.host
    const port = this.port
    const speakerId = this.speakerId
    try {
      await this.getQuery(key, speakerId)
    } catch (error) {
      throw new Error(error)
    }
    const { onPlayed, onDone } = this
    const file = new File(QUERY_PATH)
    trace(`file opened. length: ${file.length}, position: ${file.position}`)
    return new Promise((resolve, reject) => {
      this.audio = new AudioOut({ streams: 1, bitsPerSample: 16, sampleRate: this.sampleRate })
      this.audio.enqueue(0, AudioOut.Volume, Math.round((volume ?? this.volume) * 256))
      const audio = this.audio
      const streamer = new WavStreamer({
        http: device.network.http,
        host,
        port,
        path: encodeURI(`/synthesis?speaker=${speakerId}`),
        audio: {
          out: audio,
          stream: 0,
        },
        bufferDuration: 600,
        request: {
          method: 'POST',
          headers: new Headers([
            ['content-type', 'application/json'],
            ['content-length', `${file.length}`],
          ]),
          onWritable(count) {
            this.write(file.read(ArrayBuffer, count))
          },
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
          file.close()
          trace('ERROR: ', e, '\n')
          this.streaming = false
          streamer?.close()
          this.audio?.close()
          this.audio = undefined
          reject(e)
        },
        onDone: () => {
          file.close()
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
