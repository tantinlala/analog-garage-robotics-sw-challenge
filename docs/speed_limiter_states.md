# Summary
The following diagram represents the speed limiting state machine.
Note the following behavior:

- The state machine assumes it is in the estopped state until it sees the first estop message.
- After estop has been cleared, it assumes it is in the stop state until it sees the first proximity message.
- On entry into each state, an identifier for the state is published as a message and logged.

```mermaid
stateDiagram-v2
    NotEstopped: Not Estopped

    [*] --> Estopped
    Estopped --> Stop: Estop Cleared
    NotEstopped --> Estopped: Estop Triggered

    state NotEstopped {
        Full: Full Speed

        Stop --> Slow: Distance > 400mm + Hysteresis
        Slow --> Stop: Distance < 400mm
        Slow --> Full: Distance > 800mm + Hysteresis
        Full --> Slow: Distance < 800mm
        Full --> Stop: Distance < 400mm
        Stop --> Full: Distance > 800mm + Hysteresis
    }

