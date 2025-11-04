import os
from modules.utils import config 
from modules.base_agent import BaseAgent


# Load behavior tree
behavior_tree_xml = f"{os.path.dirname(os.path.abspath(__file__))}/{config['agent']['behavior_tree_xml']}"

class Agent(BaseAgent):
    def __init__(self, agent_id, ros_bridge, ros_namespace=None):
        super().__init__(agent_id, (0, 0), None)
        self.ros_bridge = ros_bridge
        self.ros_namespace = ros_namespace  # ROS 네임스페이스 저장

    def __repr__(self):
        return f"<Agent id={self.agent_id}, ns={self.ros_namespace}>"

def generate_agent(ros_bridge=None):
    ros_namespace = config['agent'].get('namespaces', [])

    # Initialize agent
    agent = Agent(0, ros_bridge, ros_namespace)

    # Provide global info and create BT
    agent.create_behavior_tree(behavior_tree_xml)

    return agent
