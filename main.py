import asyncio
import argparse
import cProfile
import signal
import sys

from modules.utils import set_config

def signal_handler(sig, frame):
    """시그널 핸들러 (Ctrl+C 등)"""
    print("\n[INFO] 종료 신호 수신, 정리 중...")
    sys.exit(0)

# Parse command line arguments
parser = argparse.ArgumentParser(description='py_bt_ros')
parser.add_argument('--config', type=str, default='config.yaml', help='Path to the configuration file (default: --config=config.yaml)')
args = parser.parse_args()

# Load configuration and initialize the environment
set_config(args.config)
from modules.utils import config
from modules.bt_runner import BTRunner

# 시그널 핸들러 등록
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

bt_runner = BTRunner(config)

async def loop():
    try:
        while bt_runner.running:
            bt_runner.handle_keyboard_events()
            if not bt_runner.paused:
                await bt_runner.step()
            bt_runner.render()
    finally:
        bt_runner.close()

if __name__ == "__main__":
    try:
        if config['bt_runner']['profiling_mode']:
            cProfile.run('main()', sort='cumulative')
        else:
            asyncio.run(loop())
    except KeyboardInterrupt:
        print("\n[INFO] 프로그램 종료 중...")