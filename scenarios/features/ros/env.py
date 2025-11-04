from modules.base_env import BaseEnv
from scenarios.features.ros.ros_bridge import ROSBridge
import os
from modules.utils import config 
from modules.base_agent import BaseAgent

# Load behavior tree
behavior_tree_xml = f"{os.path.dirname(os.path.abspath(__file__))}/{config['agent']['behavior_tree_xml']}"

class Agent(BaseAgent):
    def __init__(self, agent_id, ros_bridge, ros_namespace=None):
        super().__init__()
        self.ros_bridge = ros_bridge
        self.ros_namespace = ros_namespace  # ROS 네임스페이스 저장

class Env(BaseEnv):
    def __init__(self, config):
        super().__init__(config)

        # ROS Bridge
        self.ros_bridge = ROSBridge.get()

        # Initialise
        self.reset()

    def reset(self):
        super().reset()

        ros_namespace = config['agent'].get('namespaces', [])

        # Initialize agent
        self.agent = Agent(0, self.ros_bridge, ros_namespace)

        # Provide global info and create BT
        self.agent.create_behavior_tree(behavior_tree_xml)        
