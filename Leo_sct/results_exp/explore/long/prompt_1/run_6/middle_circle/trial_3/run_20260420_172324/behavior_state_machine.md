# Robot Behavior State Machine

Source YAML: `explore_sup_gpt_long_1_run_6_104.yaml`

State labels are inferred from the controllable events enabled in each state.

```mermaid
stateDiagram-v2
    [*] --> S0
    state "S0: random walk" as S0
    state "S1: move forward" as S1
    state "S2: rotate clockwise" as S2
    state "S3: rotate clockwise" as S3
    state "S4: rotate counterclockwise" as S4
    state "S5: random walk" as S5
    state "S6: move forward" as S6
    state "S7: rotate clockwise" as S7
    state "S8: random walk" as S8
    state "S9: move forward" as S9
    state "S10: rotate clockwise" as S10
    state "S11: random walk" as S11
    state "S12: move forward" as S12
    state "S13: rotate counterclockwise" as S13
    S0 --> S7: obstacle front
    S0 --> S10: obstacle left
    S0 --> S13: obstacle right
    S0 --> S0: path clear
    S0 --> S1: random walk
    S1 --> S0: move forward
    S1 --> S7: obstacle front
    S1 --> S10: obstacle left
    S1 --> S13: obstacle right
    S1 --> S1: path clear
    S2 --> S7: obstacle front
    S2 --> S10: obstacle left
    S2 --> S13: obstacle right
    S2 --> S2: path clear
    S2 --> S0: rotate clockwise
    S3 --> S7: obstacle front
    S3 --> S10: obstacle left
    S3 --> S13: obstacle right
    S3 --> S3: path clear
    S3 --> S0: rotate clockwise
    S4 --> S7: obstacle front
    S4 --> S10: obstacle left
    S4 --> S13: obstacle right
    S4 --> S4: path clear
    S4 --> S0: rotate counterclockwise
    S5 --> S7: obstacle front
    S5 --> S10: obstacle left
    S5 --> S13: obstacle right
    S5 --> S0: path clear
    S5 --> S6: random walk
    S6 --> S5: move forward
    S6 --> S7: obstacle front
    S6 --> S10: obstacle left
    S6 --> S13: obstacle right
    S6 --> S1: path clear
    S7 --> S7: obstacle front
    S7 --> S10: obstacle left
    S7 --> S13: obstacle right
    S7 --> S2: path clear
    S7 --> S5: rotate clockwise
    S8 --> S7: obstacle front
    S8 --> S10: obstacle left
    S8 --> S13: obstacle right
    S8 --> S0: path clear
    S8 --> S9: random walk
    S9 --> S8: move forward
    S9 --> S7: obstacle front
    S9 --> S10: obstacle left
    S9 --> S13: obstacle right
    S9 --> S1: path clear
    S10 --> S7: obstacle front
    S10 --> S10: obstacle left
    S10 --> S13: obstacle right
    S10 --> S3: path clear
    S10 --> S8: rotate clockwise
    S11 --> S7: obstacle front
    S11 --> S10: obstacle left
    S11 --> S13: obstacle right
    S11 --> S0: path clear
    S11 --> S12: random walk
    S12 --> S11: move forward
    S12 --> S7: obstacle front
    S12 --> S10: obstacle left
    S12 --> S13: obstacle right
    S12 --> S1: path clear
    S13 --> S7: obstacle front
    S13 --> S10: obstacle left
    S13 --> S13: obstacle right
    S13 --> S4: path clear
    S13 --> S11: rotate counterclockwise
```

## State Summary

- `S0`: Initial state. Behavior `random walk`. Transitions: obstacle front -> S7, obstacle left -> S10, obstacle right -> S13, path clear -> S0, random walk -> S1.
- `S1`: Behavior `move forward`. Transitions: move forward -> S0, obstacle front -> S7, obstacle left -> S10, obstacle right -> S13, path clear -> S1.
- `S2`: Behavior `rotate clockwise`. Transitions: obstacle front -> S7, obstacle left -> S10, obstacle right -> S13, path clear -> S2, rotate clockwise -> S0.
- `S3`: Behavior `rotate clockwise`. Transitions: obstacle front -> S7, obstacle left -> S10, obstacle right -> S13, path clear -> S3, rotate clockwise -> S0.
- `S4`: Behavior `rotate counterclockwise`. Transitions: obstacle front -> S7, obstacle left -> S10, obstacle right -> S13, path clear -> S4, rotate counterclockwise -> S0.
- `S5`: Behavior `random walk`. Transitions: obstacle front -> S7, obstacle left -> S10, obstacle right -> S13, path clear -> S0, random walk -> S6.
- `S6`: Behavior `move forward`. Transitions: move forward -> S5, obstacle front -> S7, obstacle left -> S10, obstacle right -> S13, path clear -> S1.
- `S7`: Behavior `rotate clockwise`. Transitions: obstacle front -> S7, obstacle left -> S10, obstacle right -> S13, path clear -> S2, rotate clockwise -> S5.
- `S8`: Behavior `random walk`. Transitions: obstacle front -> S7, obstacle left -> S10, obstacle right -> S13, path clear -> S0, random walk -> S9.
- `S9`: Behavior `move forward`. Transitions: move forward -> S8, obstacle front -> S7, obstacle left -> S10, obstacle right -> S13, path clear -> S1.
- `S10`: Behavior `rotate clockwise`. Transitions: obstacle front -> S7, obstacle left -> S10, obstacle right -> S13, path clear -> S3, rotate clockwise -> S8.
- `S11`: Behavior `random walk`. Transitions: obstacle front -> S7, obstacle left -> S10, obstacle right -> S13, path clear -> S0, random walk -> S12.
- `S12`: Behavior `move forward`. Transitions: move forward -> S11, obstacle front -> S7, obstacle left -> S10, obstacle right -> S13, path clear -> S1.
- `S13`: Behavior `rotate counterclockwise`. Transitions: obstacle front -> S7, obstacle left -> S10, obstacle right -> S13, path clear -> S4, rotate counterclockwise -> S11.
