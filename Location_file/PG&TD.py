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
