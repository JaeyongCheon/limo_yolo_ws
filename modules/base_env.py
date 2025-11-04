import os
os.environ['SDL_VIDEO_WINDOW_POS'] = "0,30"  # top-left corner
import pygame

class BaseEnv:
    def __init__(self, config):
        self.config = config
        self.bt_tick_rate = config['simulation']['bt_tick_rate']
        self.screen_height = config['simulation']['screen_height']
        self.screen_width = config['simulation']['screen_width']        
        pygame.init()
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height), pygame.RESIZABLE)
        self.background_color = (224, 224, 224)
        self.clock = pygame.time.Clock()

    def reset(self):
        # Initialization        
        self.running = True
        self.game_paused = False   

        self.agent = None




    async def step(self):
        # Main simulation loop logic
        await self.agent.run_tree()
        self.clock.tick(self.bt_tick_rate)


    def close(self):
        pass


    def handle_keyboard_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    self.running = False
                elif event.key == pygame.K_p:
                    self.game_paused = not self.game_paused