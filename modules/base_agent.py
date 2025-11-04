from modules.utils import config, optional_import
env_pkg = config.get('scenario').get('environment')
bt_module = optional_import(env_pkg + ".bt_nodes")

from modules.bt_constructor import build_behavior_tree

class BaseAgent:
    def __init__(self):
        self.blackboard = {}

    def create_behavior_tree(self, behavior_tree_xml):
        self.behavior_tree_xml = behavior_tree_xml
        self.tree = build_behavior_tree(self, behavior_tree_xml, env_pkg)

    def _reset_bt_action_node_status(self):
        self.tree.reset()
        BTNodeList = getattr(bt_module, "BTNodeList")        
        action_nodes = BTNodeList.ACTION_NODES
        self.blackboard = {key: None if key in action_nodes else value for key, value in self.blackboard.items()}

    async def run_tree(self):
        self._reset_bt_action_node_status()
        return await self.tree.run(self, self.blackboard)

