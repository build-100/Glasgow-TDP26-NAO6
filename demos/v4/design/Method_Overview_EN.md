# Method origin and evidence boundary

The local experiment chooses one legal destination region per field player per macro step.
Movement is restricted to the current or an immediately adjacent horizontal/vertical/diagonal region.
Both methods use the same observation/execution capabilities, within-region geometry and kick model.
Keepers remain inactive; a labelled kickoff reset follows a goal, never an unscored stalemate.

The cooperative team uses shared local action-value networks with additive VDN training and Double DQN
targets. Centralized training learns from joint experience and one team return; execution selects legal
local actions. This is implicit value decomposition, not a causal percentage-of-contribution estimate.
The full training/reward implementation and neural weights remain outside this public demo package.

The V4 baseline selects robot/region assignments sequentially by marginal improvement to a short-term
team objective. Coverage already supplied by a teammate gives less additional benefit; travel and
congestion also affect the choice. Remaining gains are recomputed after each assignment. It is a
coordinated non-learning heuristic, not three independent chasers and not a CTDE-trained model.
No approximation guarantee from a different application is claimed.

The original V3 cooperative network was frozen before comparison. All 128 standard and 64 perturbed
initial match cases remained unfinished against the new baseline. In contrast, the new baseline won
128/128 cases against the old independent greedy. Only then was the cooperative network adapted;
the selected baseline, environment and reward remained unchanged.

Three warm-start runs each added 120000 training steps. Separate full-match validation selected one
checkpoint before accessing the final test cases, with team colours and field directions swapped:

| Final held-out condition | Cooperative wins | Baseline wins | Unfinished |
| --- | ---: | ---: | ---: |
| Standard kickoff | 126 | 2 | 0 |
| Perturbed kickoff | 56 | 8 | 0 |
| Original frozen V3 on the same final standard seeds | 0 | 0 | 128 |

These are paired simulation cases, not 128 independent training runs. The results support adaptation
against this fixed baseline in this model, not a global-optimality claim. An equally trained independent
Double-DQN ablation was not performed, so the gain is not isolated causal evidence for credit assignment.
V3's 100% and V4's 98.44% standard rates concern different opponents/protocols and are not directly comparable.

All 24 V4 presentation candidates completed (23 red wins, one blue win). Selection used median completed
match length and then seed, with no preference for winner, margin or contest outcome. The resulting
3:1 sequence includes a blue goal and both successful and unsuccessful contests. The recording predates
the naming change, so its V3-SG label is retained as historical evidence for V4.

The published plan is a frozen export of one selected match. Running the public ROS executor reproduces
that plan, while rosbag playback reproduces recorded messages. Neither operation retrains or evaluates
the policy on new opponents. Animation is not validated physical NAO or Gazebo skill control.
