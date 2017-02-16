# Under MIT License, see LICENSE.txt

from ai.executors.executor import Executor


class PlayExecutor(Executor):

    def __init__(self, p_world_state):
        super().__init__(p_world_state)

    def exec(self):
        for player_id, tactic in self.ws.play_state.current_tactic.items():
            self.ws.play_state.current_ai_commands[player_id] = tactic.exec()
        self._send_robots_status()

    # FIXME revise this function please
    def _send_robots_status(self):
        for tactic in self.ws.play_state.current_tactic.values():
            player_id = tactic.player_id
            tactic_name = str(tactic) + tactic.status_flag.name
            action_name = tactic.current_state.__name__
            target = (int(tactic.target.position.x), int(tactic.target.position.y))
            self.ws.debug_interface.send_robot_status(player_id,
                                                      tactic_name,
                                                      action_name,
                                                      target)


