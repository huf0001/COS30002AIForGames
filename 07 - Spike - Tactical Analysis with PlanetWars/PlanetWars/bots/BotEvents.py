from random import choice
from pyglet.event import EventDispatcher

class BotEvents(EventDispatcher):
    def __init__(self):
        self.register_event_type('on_new_fleet')

    def new_fleet(self, src, dest):
        self.dispatch_event('on_new_fleet', src, dest)