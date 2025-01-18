# Getting Started

[日本語](./getting-started_ja.md)

## Prerequisites

* Host PC
    * Tested on Linux (Ubuntu 20.04)
* M5Stack Basic
* USB type-C cable
* [git](https://git-scm.com/)
* [Node.js](https://nodejs.org/en/)
    * Tested with v22

## Clone the repository

Clone this repository.

```console
$ git clone https://github.com/stack-chan/stack-chan.git
$ cd stack-chan/firmware
$ npm i
```

## Setting up ModdableSDK

On the host PC, install [ModdableSDK](https://github.com/Moddable-OpenSource/moddable) and
Install [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) on the host PC.
There are three ways to do this

- Using the CLI (recommended)
- Using a Docker image
- Set up manually

### Using xs-dev (recommended)

Stack-Chan has npm scripting of the setup procedure.
Run the following command in the `stack-chan/firmware` directory.

```console
$ npm run setup
$ npm run setup -- --device=esp32
```

The script internally uses [`xs-dev`](https://github.com/HipsterBrown/xs-dev) to automate the setup of ModdableSDK and ESP-IDF.

### Using Docker images (for Linux only)

This repository provides a Dockerfile build environment.
You can build, write and debug firmware inside a Docker container.

Note: This has been tested and confirmed to work on Linux (Ubuntu 24.04). It is not recommended for use on Windows (WSL) or MacOS, as there have been reported [issues](https://github.com/stack-chan/stack-chan/issues/144) with connecting to devices from the container side.

#### From terminal

Run the following command in the `stack-chan/firmware` directory.

```console
$ ./docker/build-container.sh
$ ./docker/launch-container.sh

# Run inside container
$ npm install
```

#### From VSCode

This project includes DevContainer preference for VSCode.
You can open the project in a container with the following commands

* Open the command palette (ctrl+shift+p)
* Run `>Remote-Containers: Reopen in Container`

### Manual

Follow the instructions on the [official website (English)](https://github.com/Moddable-OpenSource/moddable/blob/public/documentation/Moddable%20SDK%20-%20Getting%20Started.md) to install ModdableSDK and ESP-IDF.
If you cannot set up the CLI or Docker properly, please use this manual method.

## (Optional)Updating ModdableSDK

With the update of this repository, it may be necessary to update the ModdableSDK to add new features or resolve bugs.

### Using xs-dev (recommended)

Stack-Chan has also npm scripting of the update procedure.
Run the following command in the `stack-chan/firmware` directory.

```console
$ npm run update
$ npm run update -- --device=esp32
```

### Manual

Follow the instructions on the [official website (English)](https://github.com/Moddable-OpenSource/moddable/blob/public/documentation/Moddable%20SDK%20-%20Getting%20Started.md) to update ModdableSDK and ESP-IDF.

## Test the environment

You can test the environment with the `npm run doctor` command.
If the installation was successful, the version of Moddable SDK will be displayed as follows, and esp32 will be displayed in Supported target devices.

```console
$ npm run doctor

> stack-chan@0.2.1 doctor
> xs-dev doctor

xs-dev environment info:
  CLI Version                0.32.7                                                                 
  OS                         Linux                                                                  
  Arch                       x64                                                                    
  Shell                      /bin/bash                                                              
  NodeJS Version             v22.9.0 (/home/sskw/.nvm/versions/node/v22.9.0/bin/node)               
  Python Version             3.12.3 (/home/sskw/.espressif/python_env/idf5.3_py3.12_env/bin/python) 
  Moddable SDK Version       5.3.3 (/home/sskw/.local/share/moddable)                               
  Supported target devices   lin, esp32                                                             
  ESP32 IDF Directory        /home/sskw/.local/share/esp32/esp-idf 

If this is related to an error when using the CLI, please create an issue at "https://github.com/hipsterbrown/xs-dev/issues/new" with the above info.
```

## Next step

- [Build and Flash firmware](./flashing-firmware.md)
