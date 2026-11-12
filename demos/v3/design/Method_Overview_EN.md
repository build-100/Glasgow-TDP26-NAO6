# Method origin and evidence boundary

The local experiment chooses one legal destination region per field player per macro step.
Movement is restricted to the current or an immediately adjacent horizontal/vertical/diagonal region.
Both methods use the same observation/execution capabilities, within-region geometry and kick model.
Keepers remain inactive; a labelled kickoff reset follows a goal, never an unscored stalemate.

The cooperative team uses shared local action-value networks with additive VDN training and Double DQN
targets. Centralized training learns from joint experience and one team return; execution selects legal
local actions. This is implicit value decomposition, not a causal percentage-of-contribution estimate.
The full training/reward implementation and neural weights remain outside this public demo package.

The V3 baseline independently scores each player's own opportunities and movement costs. There is
no common assignment reservation or marginal-coverage update, so multiple players can select the same
ball or landing opportunity. V3 therefore contains a comparatively weak, clustering-prone baseline.

Reported local held-out result: the selected cooperative policy won 120/120 complete first-to-three
cases (60 seeds, both field directions) against that fixed baseline. This is a narrow model/opponent
result, not general soccer ability or proof of global optimality. The V3 presentation pool prioritised
red victories; the 3:0 showcase cannot be used to estimate a success rate.

The published plan is a frozen export of one selected match. Running the public ROS executor reproduces
that plan, while rosbag playback reproduces recorded messages. Neither operation retrains or evaluates
the policy on new opponents. Animation is not validated physical NAO or Gazebo skill control.
