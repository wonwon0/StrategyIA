# Under MIT License, see LICENSE.txt

from RULEngine.Util.singleton import Singleton
from ai.STA.Tactic.TacticBook import TacticBook


class PlayState(object, metaclass=Singleton):

    def __init__(self):
        self.tactic_book = TacticBook()
        self.current_strategy = None
        self.current_tactic = {}
        self.current_ai_commands = {}

    def get_new_tactic(self, tactic_name):
        return self.tactic_book.get_tactic(tactic_name)
