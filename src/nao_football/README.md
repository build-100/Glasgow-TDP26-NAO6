# Minimal planning scaffold

`planning/grid.py` defines a fixed field partition and legal adjacent targets.
`policy_stub.py` supplies the three-field-player movement interface and returns
WAIT targets. `contracts.py` defines the common robot skill vocabulary.

No football transition model, learned policy, reward tuning, credit assignment
or private research implementation is included in this initial model repository.
The manager can add those behind this interface later. Physical execution must
report completion or timeout; issuing a target does not mean it was reached.
