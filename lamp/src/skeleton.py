import torch 
import torch.nn as nn
import torch.optim as optim 

from random import random

class PolicyNetwork(nn.Module):

    def __init__(self, scan_dims, observation_dims, n_actions):

        super(PolicyNetwork, self).__init__()

        leaky_slope = 0.1

        self.scan_network = nn.Sequential(
            nn.Conv1d(in_channels=1, out_channels=4, kernel_size=21, stride=20, padding=10, bias=True),
            nn.LeakyReLU(leaky_slope),

            nn.Flatten(),

            nn.Linear(128, 128, bias=True)
        )

        self.observation_network = nn.Sequential(
            nn.Linear(observation_dims, 128, bias=True)
        )

        self.base_network = nn.Sequential(
            nn.LeakyReLU(leaky_slope),
            
            nn.Linear(256, 256, bias=True),
            nn.LeakyReLU(leaky_slope),
        )

        self.mean_network = nn.Sequential(
            nn.Linear(256, n_actions, bias=True),
            nn.Tanh()
        )

        self.std_network = nn.Sequential(
            nn.Linear(256, n_actions, bias=True),
            nn.Softplus()
        )

    def forward(self, scan, observation):

        scan = self.scan_network(scan)
        observation = self.observation_network(observation)

        state = torch.cat([scan, observation], dim=1)

        base_out = self.base_network(state)
        mean = self.mean_network(base_out)
        std = self.std_network(base_out)

        return mean, std


class Agent():

    def __init__(self, 
        scan_dims,
        observation_dims,
        n_actions,
        alpha,
        gamma,
        reuse=False
    ):
        
        self.gamma = gamma
        self.scan_dins = scan_dims
        self.observation_dims = observation_dims
        self.n_actions = n_actions

        self.model = PolicyNetwork(scan_dims, observation_dims, n_actions)

        if (reuse):
            self.model.load_state_dict(torch.load(reuse))

        self.optimizer = optim.Adam(self.model.parameters(), lr=alpha)

        self.action_values = []
        self.means = []
        self.stds = []

    def choose_action(self, scan, observation):

        scan = torch.tensor([[scan]], dtype=torch.float32)
        observation = torch.tensor([observation], dtype=torch.float32)

        mean, std = self.model(scan, observation)
        action = torch.normal(mean, std)

        self.action_values.append(action[0])

        return mean, std, action[0]

    def learn(self, scans, observations, rewards):

        scans = torch.tensor(scans, dtype=torch.float32)
        observations = torch.tensor(observations, dtype=torch.float32)

        returns = []
        g = 0
        for r in reversed(rewards):
            g = r + self.gamma * g
            returns.append(g)

        returns = torch.tensor(returns, dtype=torch.float32)

        action_values = torch.stack(self.action_values)

        means, stds = self.model(scans, observations)
        log_probabilities = -(0.5 * ((action_values - means)/stds) ** 2 + torch.log(stds * 1.414 * 1.772)).sum(dim=1)

        loss = -(returns * log_probabilities).mean()

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        self.action_values = []

    def save(self, file_name):
        torch.save(self.model.state_dict(), file_name)

# agent = Agent(
#     scan_dims=360,
#     observation_dims=4,
#     n_actions=2,
#     alpha=0.01,
#     gamma=0.9,
#     reuse=False
# )

# scans = [[[random()]*360]]*600
# observations = [[random()]*4]*600

# rewards = [random()]*600

# for r in range(600):
#     scan = scans[r][0]
#     ob = observations[r]
#     agent.choose_action(scan, ob)

