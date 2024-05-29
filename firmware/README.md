# Pomodoro-AF

This is a hobby project to create a simple Pomodoro device that can sit on a
desk and be quickly and trivially activated to start a focus session. Yes,
these exist and can be purchased from Amazon, but it doesn't mean they
**should** be purchased.

## Goals

The current stated goals for the project are

- Make a physical device I personally **love** to use every day
- Learn Embassy-rs and apply it to a useful project
- Design a Pomodoro that is
  - Simple and obvious to use
  - Open Source, both software and hardware designs
  - Hackable & extendable

And of course don't forget the cardinal axiom of all user interface design

> A user interface is well-designed when the program behaves exactly how the user thought it would.
>
> -- Joel on Software

## Principles of Pomodoro Method

Taken from the [Todist](https://todoist.com/productivity-methods/pomodoro-technique) website the method consists of the following

1. Pick a task
2. Set a 25-minute timer
3. Work on your task until the time is up
4. Take a 5-minute break
5. Every 4 pomodoros, take a longer 15-30 minute break

## User Activities

- Run a single Pomodoro from start to finish
- Run a single Pomodoro but pause the countdown because of an interruption, then either
  - Unpuase and continue the current countdown from where it left off
  - Restart the current countdown from the beginning
- Run a full series of 4 pomodoro each with a break in between and a longer break at the end
- Run a full series of 4 pomodoro each with a break in between and a longer break at the end
  - Pause and Unpuase as necessary for interruptions
  - Restart current countdown in a series
  - Restart entire series from the beginning

## Potentially Useful Links

[smart-leds-rs](https://github.com/smart-leds-rs/smart-leds-samples/blob/master/stm32f411-examples/examples/stm32f411_example_spi_rainbow.rs)

## Repository Layout

`software/usb-client/` - Application that can interface with the firmware
`firmware/` - Low-level code running on the micro

- `docs/` contains the documentation for the project

Not committed to the repository, but checked out along side it \
`embassy/` - Clone of the embassy-rs repository
