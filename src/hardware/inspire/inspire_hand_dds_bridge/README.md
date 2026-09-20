# Inspire hand DDS bridge

Independent real-hardware output module:

```text
Nova2 -> ManusGlove -> Inspire direct retarget JointState
      -> MotorCmds DDS -> inspire_g1_service -> RS-485 hand
```

DDS topics:

- `rt/inspire/left/cmd`
- `rt/inspire/right/cmd`

The six channels are `[pinky, ring, middle, index, thumb_bend,
thumb_rotate]`. Simulation radians use `0=open`; the G1 service uses
normalized `1=open`, so this bridge performs `q = 1 - position / upper_limit`.

The bridge publishes nothing until it receives a complete, finite, named
Inspire `JointState`. Default normalized speed is `0.5`.
