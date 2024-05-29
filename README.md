# Pomodoro Repo

## Quick Start

After cloning this repo run `just embassy` to make sure all the prerequisits are
in the right places.

### Building Firmware
`cd` into the firmware directory and run 

```bash
cargo build
```

To program a target and run the program

```bash
cargo run --release
```

To run a specific binary inside the `src/bin/` directory

```bash
cargo run --bin usb_raw --release
```

## Requirements

1. Embassy source code must be cloned locally (for now).
