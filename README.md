# TinyAD9833 Frequency Generator"


Small frequency generator consisting of an AD9833 board and a DigiSpark Tiny85 board
with micronucleus bootloader. USB serial interface for commands.

## Usage
```
usage: <number>[STONED]" ) );
  <number>: 1..8 digits, opt. dot, k, M" ) );
  S:Sin, T:Tri, O:Off" ) );
  N: to reg, n*0.093 Hz Sin" ) );
  E: echo" ) );
  D: debug" ) );
```

## HW connections
```
DATA  - PB0
FSYNC - PB1
CLK   - PB2
```
