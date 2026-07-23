from pydoover.tags import Tag, Tags


class ChannelGateControlTags(Tags):
    """Controller state, published live so the HMI (and peer apps) can read it.

    ``GateHeight`` mirrors the encoder reading the controller is acting on;
    ``TargetHeight`` is the operator setpoint. ``RaiseOutput`` / ``LowerOutput``
    reflect the actual solenoid drive so a simulator (or another app) can close
    the physical loop, and ``Fault`` / ``FaultReason`` surface any latched trip.
    """

    TargetHeight = Tag("number", default=0.0, live=True)     # setpoint, mm
    GateHeight = Tag("number", default=0.0, live=True)       # actual, mm
    Error = Tag("number", default=0.0, live=True)            # target - actual, mm
    Moving = Tag("string", default="idle", live=True)        # raising/lowering/idle
    RaiseOutput = Tag("boolean", default=False, live=True)   # raise solenoid energised
    LowerOutput = Tag("boolean", default=False, live=True)   # lower solenoid energised
    PumpOutput = Tag("boolean", default=False, live=True)    # pump energised
    EStopActive = Tag("boolean", default=False, live=True)   # top limit asserted
    Mode = Tag("string", default="hold", live=True)          # auto / hold
    Status = Tag("string", default="starting", live=True)    # human-readable state
    Fault = Tag("boolean", default=False, live=True)         # latched trip
    FaultReason = Tag("string", default="", live=True)       # why it tripped
    HeightValid = Tag("boolean", default=False, live=True)   # encoder signal present
