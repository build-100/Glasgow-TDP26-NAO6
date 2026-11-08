# Multi-Robot Active Cooperative Localization: PG & TD
> Core note:
> **TD (Temporal Difference)** and **PG (Policy Gradient)** here are not alternatives to EKF / particle filters for state fusion. They solve **active cooperative multi-robot localization**:
> - **Low-level estimator**: EKF / particle filter, fusing odometry and inter-robot relative measurements to estimate robot poses and uncertainty.
> - **High-level decision maker (TD / PG)**: Reinforcement learning agent that outputs motion, observation and communication decisions. It selects where robots should move and when to perform mutual observations to minimize the overall localization error as fast as possible.
>
> The common "PG + TD" setup in literature refers to **Actor-Critic**: the Actor updates the policy via policy gradient (PG), while the Critic estimates the value function using temporal difference (TD).

## 1. MDP Formulation
For $N$ robots: the ground-truth pose of robot $i$ is $\boldsymbol{x}_i^\text{gt} = [x_i,y_i,\theta_i]^T$. The filter produces estimated pose $\hat{\boldsymbol{x}}_i$ and uncertainty covariance $\boldsymbol{\Sigma}_i$.

We formulate active localization as a Dec-POMDP (decentralized partially observable Markov decision process):
- **State $s_t$**: Estimated poses, localization errors, covariance matrices and relative observations of all robots. In decentralized settings, each robot only receives its local observation $o_i$.
- **Action $a_t$**: Moving direction, triggering inter-robot relative observation, or enabling communication.
- **State transition**: Execute action → true robot motion with odometry noise → acquire observations → run localization filter to obtain the next state estimate.
- **Reward $r_t$**: Usually defined as negative total localization error, with optional communication penalty:
$$
r_t = -\left(\sum_{i}\|\hat{\boldsymbol{x}}_i - \boldsymbol{x}_i^\text{gt}\|_2^2 + \lambda\cdot c_\text{comm}\right)
$$
Smaller error and less communication yield higher reward.
- **Discounted return**:
$$
G_t = \sum_{k=t}^{\infty}\gamma^{k-t} r_k,\quad \gamma\in(0,1)
$$

## 2. TD (Temporal Difference)
TD is a value-based family of algorithms. Unlike Monte Carlo (MC), which waits until episode termination, TD bootstraps from the next state value for single-step updates.

### TD(0) for state value
Estimate state value $\(V(s)\)$: expected future return starting from state $s$.
$$
\delta_t = r_{t+1} + \gamma V(s_{t+1}) - V(s_t)
$$
$$
V(s_t) \leftarrow V(s_t) + \alpha \cdot \delta_t
$$
$\delta_t$ is the **TD error**, the central quantity of TD learning.

### TD control: Q-Learning
Estimate action-value function $\(Q(s,a)\)$: expected return after taking action $a$ at state $s$.
$$
\delta_t = r_{t+1} + \gamma \max_{a'} Q(s_{t+1},a') - Q(s_t,a_t)
$$
$$
Q(s_t,a_t) \leftarrow Q(s_t,a_t) + \alpha \cdot \delta_t
$$
Policy is typically $\varepsilon$-greedy: explore randomly with probability $\varepsilon$, otherwise select the action with maximum $Q$.

✅ Usage in multi-robot localization:
The Critic network is trained by TD. It evaluates whether the current localization state is good by predicting future reward given the uncertainty state.

> Drawback: Joint action space explodes with the number of robots; fitting value functions is difficult for high-dimensional continuous states, and policies are implicit.

## 3. PG (Policy Gradient)
Policy gradient methods do not learn values first. They parameterize a policy $\pi_\theta(a|s)$ that outputs a distribution over actions given a state, and maximize the expected return:
$$
J(\theta) = \mathbb{E}_{\tau\sim\pi_\theta}\left[G(\tau)\right]
$$

### REINFORCE (Monte Carlo PG)
$$
\nabla_\theta J = \mathbb{E}\left[\nabla_\theta \log\pi_\theta(a_t|s_t)\cdot G_t\right]
$$
Updates require complete trajectory returns. Simple but suffers from high variance.

### Actor-Critic: PG + TD
To reduce variance of vanilla PG, use TD-learned value as a baseline:
- **Actor (PG)**: Update the policy, using TD error $\delta_t$ instead of raw return.
- **Critic (TD)**: Learn $\(V(s)\)$ and compute TD error $\delta_t$.

$$
\nabla_\theta J \approx \mathbb{E}\left[\nabla_\theta \log\pi_\theta(a_t|s_t)\cdot \delta_t\right]
$$

👉 This is the widely adopted **PG (Actor) + TD (Critic)** framework for active multi-robot localization.

✅ Usage in multi-robot localization: Directly learn motion and observation policies. It naturally fits continuous action spaces and decentralized multi-agent scenarios.

## 4. TD vs PG Comparison
| Dimension | TD (value-based, e.g. Q-learning) | PG (policy gradient, REINFORCE / Actor) |
|---|---|---|
| Learning objective | Learn $\(V(s)\)/\(Q(s,a)\)$; policy derived indirectly | Learn policy $\pi_\theta(a|s)$ directly |
| Update information | Bootstrap, single-step update available | Vanilla REINFORCE needs full trajectory; AC uses TD error |
| Variance | Relatively low | High for vanilla REINFORCE; reduced by TD in AC |
| Action space | Good for discrete actions; joint action explosion | Natural for continuous control, suitable for robot velocity commands |
| Multi-robot | Decentralized Q-learning hard to converge | Decentralized policy gradient (DAPG, COMA) is more common |

## 5. Python Demo: Multi-Robot Active Localization
> Simplifications for readability:
> 1. 2 robots in a 2D plane
> 2. Lightweight cooperative measurement update instead of full EKF
> 3. Discrete actions: `0: up, 1: down, 2: left, 3: right`
> 4. If inter-robot distance is below threshold, relative range measurement corrects pose estimates
> 5. State: flattened estimated coordinates of all robots
> 6. Reward: negative total localization error

Dependencies: only `numpy`
```python
import numpy as np

# ====================== Multi-Robot Cooperative Localization Env ======================
class MultiRobotLocEnv:
    def __init__(self, n_robots=2, obs_range=2.5, motion_noise=0.1):
        self.n = n_robots
        self.obs_range = obs_range
        self.motion_noise = motion_noise
        self.action_set = np.array([[0,0.15],[0,-0.15],[-0.15,0],[0.15,0]]) # up, down, left, right
        self.gt_pos = None    # ground truth [n, 2]
        self.est_pos = None   # localization estimate [n, 2]

    def reset(self):
        self.gt_pos = np.random.uniform(-3, 3, size=(self.n, 2))
        self.est_pos = self.gt_pos + np.random.normal(0, 0.8, size=(self.n, 2))
        return self._get_state()

    def _get_state(self):
        return self.est_pos.flatten()

    def _cooperative_observation(self):
        """Use relative range to correct pose estimates when robots are within observation range"""
        for i in range(self.n):
            for j in range(i+1, self.n):
                d_gt = np.linalg.norm(self.gt_pos[i] - self.gt_pos[j])
                d_est = np.linalg.norm(self.est_pos[i] - self.est_pos[j])
                if d_gt < self.obs_range:
                    delta = (d_gt - d_est) * 0.3
                    dir_ij = (self.est_pos[j] - self.est_pos[i]) / (d_est + 1e-8)
                    self.est_pos[i] += dir_ij * delta * 0.5
                    self.est_pos[j] -= dir_ij * delta * 0.5

    def step(self, actions):
        """
        actions: list of action indices for each robot
        return: s, r, done
        """
        # True robot motion with odometry noise
        for i, a in enumerate(actions):
            move = self.action_set[a] + np.random.normal(0, self.motion_noise, 2)
            self.gt_pos[i] += move
            self.est_pos[i] += self.action_set[a] + np.random.normal(0, self.motion_noise*1.2, 2)

        self._cooperative_observation()

        err = np.sum(np.linalg.norm(self.est_pos - self.gt_pos, axis=1))
        reward = -err
        done = False
        return self._get_state(), reward, done

# ====================== Version 1: TD Q-Learning (TD Control) ======================
class TDQLearning:
    def __init__(self, state_dim, act_dim, lr=0.05, gamma=0.95, eps=0.2):
        self.lr = lr
        self.gamma = gamma
        self.eps = eps
        self.act_dim = act_dim
        self.w = np.zeros((act_dim, state_dim))

    def q_val(self, s, a):
        return self.w[a] @ s

    def select_action(self, s):
        if np.random.rand() < self.eps:
            return np.random.randint(self.act_dim)
        qs = np.array([self.q_val(s,a) for a in range(self.act_dim)])
        return int(np.argmax(qs))

    def update(self, s,a,r,s_next):
        q_next = max([self.q_val(s_next, ap) for ap in range(self.act_dim)])
        td_err = r + self.gamma * q_next - self.q_val(s,a)
        self.w[a] += self.lr * td_err * s
        return td_err

# ====================== Version 2: REINFORCE (Pure Monte-Carlo PG) ======================
class REINFORCE:
    def __init__(self, state_dim, act_dim, lr=0.03, gamma=0.95):
        self.lr = lr
        self.gamma = gamma
        self.act_dim = act_dim
        self.w = np.zeros((act_dim, state_dim))
        self.trajectory = []

    def pi(self, s):
        logits = self.w @ s
        exp = np.exp(logits - np.max(logits))
        return exp / exp.sum()

    def select_action(self, s):
        prob = self.pi(s)
        a = np.random.choice(self.act_dim, p=prob)
        self.trajectory.append((s,a))
        return a

    def update(self, rewards):
        G = 0
        grad_acc = np.zeros_like(self.w)
        for (s,a), r in zip(reversed(self.trajectory), reversed(rewards)):
            G = r + self.gamma * G
            prob = self.pi(s)
            grad = -prob.reshape(-1,1) * s
            grad[a] += s
            grad_acc += G * grad
        self.w += self.lr * grad_acc
        self.trajectory.clear()

# ====================== Version 3: Actor-Critic (PG Actor + TD Critic) ======================
class ActorCritic:
    def __init__(self, state_dim, act_dim, lr_a=0.02, lr_c=0.05, gamma=0.95):
        self.gamma = gamma
        self.act_dim = act_dim
        self.w_actor = np.zeros((act_dim, state_dim))
        self.w_critic = np.zeros(state_dim)
        self.lr_a = lr_a
        self.lr_c = lr_c

    def v(self, s):
        return self.w_critic @ s

    def pi(self, s):
        logits = self.w_actor @ s
        exp = np.exp(logits - np.max(logits))
        return exp / exp.sum()

    def select_action(self, s):
        prob = self.pi(s)
        return np.random.choice(self.act_dim, p=prob)

    def update(self, s,a,r,s_next):
        v_s = self.v(s)
        v_next = self.v(s_next)
        td_err = r + self.gamma * v_next - v_s
        # Critic TD update
        self.w_critic += self.lr_c * td_err * s
        # Actor PG update with TD error as baseline
        prob = self.pi(s)
        grad = -prob.reshape(-1,1) * s
        grad[a] += s
        self.w_actor += self.lr_a * td_err * grad
        return td_err

# ====================== Training Entry ======================
if __name__ == "__main__":
    env = MultiRobotLocEnv(n_robots=2)
    state_dim = 4   # 2 robots × (x,y)
    act_dim = 4     # up, down, left, right

    # Uncomment to switch agent
    # agent = TDQLearning(state_dim, act_dim)
    # agent = REINFORCE(state_dim, act_dim)
    agent = ActorCritic(state_dim, act_dim)

    EPISODES = 300
    for ep in range(EPISODES):
        s = env.reset()
        total_r = 0
        reward_list = []
        for step in range(80):
            a1 = agent.select_action(s)
            a2 = agent.select_action(s)
            s_next, r, done = env.step([a1,a2])
            total_r += r
            reward_list.append(r)

            if isinstance(agent, TDQLearning) or isinstance(agent, ActorCritic):
                agent.update(s, a1, r, s_next)
            s = s_next
            if done:
                break
        if isinstance(agent, REINFORCE):
            agent.update(reward_list)
        if ep % 20 == 0:
            print(f"Episode {ep:3d} | Total Reward: {total_r:.2f}")
```

## 6. Engineering Extensions for Real Multi-Robot Systems
This prototype is minimal. Typical upgrades in papers and real implementations:
1. **State augmentation**: Feed covariance matrices (localization uncertainty) and communication topology into the state vector.
2. **Low-level fusion**: Replace the simple measurement correction with a full EKF or particle filter.
3. **Decentralization**: Each robot maintains its own Actor; use COMA, DAPG for distributed policy gradient.
4. **Continuous actions**: Actor outputs Gaussian distributions to control linear and angular velocities.
5. **Neural network approximation**: Replace linear weights with MLPs for high-dimensional state inputs.
6. **Communication constraints**: Add bandwidth penalty in reward; only trigger inter-robot observation when the expected gain is positive.

## 7. Reading Keywords
- `Active Cooperative Localization + Reinforcement Learning`
- `Multi-robot localization policy gradient temporal difference`
- Dec-POMDP, COMA, MA-DDPG

---
