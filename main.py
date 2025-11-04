import asyncio
import argparse
import cProfile
import importlib
from threading import Thread

from modules.utils import set_config

# Parse command line arguments
parser = argparse.ArgumentParser(description='py_bt_ros')
parser.add_argument('--config', type=str, default='config.yaml', help='Path to the configuration file (default: --config=config.yaml)')
args = parser.parse_args()

# Load configuration and initialize the environment
set_config(args.config)
from modules.utils import config


# Dynamically import the environment module and Env class
env_module = importlib.import_module(config.get('scenario').get('environment') + ".env")
Env = getattr(env_module, "Env")
# Initialize Env instance
env = Env(config)

async def game_loop():
    while env.running:
        env.handle_keyboard_events()

        if not env.game_paused:
            await env.step()
            # Record data if time recording mode is enabled


    env.close()



def main():
    bt_viz_cfg = config['simulation'].get('bt_visualiser', {})
    if bt_viz_cfg.get('enabled', False):
        from modules.bt_visualiser import visualise_bt
        agent = env.agent
        Thread(
            target=visualise_bt, 
            args=(0, agent.tree), 
            daemon=True
        ).start()

    asyncio.run(game_loop())

if __name__ == "__main__":
    if config['simulation']['profiling_mode']:
        cProfile.run('main()', sort='cumulative')
    else:
        main()